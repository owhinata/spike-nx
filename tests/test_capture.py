"""Category K: capture pipeline tests (Issue #122).

Covers the kernel `/dev/btcap` chardev + apps/capture lib + apps/sensor
``capture`` verb + btsensor MODE CAPTURE forwarder.  Smoke cases run
without a connected sensor or BT pairing; the round-trip case is
``interactive`` because it needs a Color Sensor on a port and a BlueZ
host paired against the Hub.

Wire format (recap):

  BTCS(4B) + meta(40B = u16 schema_magic + u16 reserved + u32 total_bytes
                       + char[32] name)
  + payload(total_bytes B, .cap header "CAPB" + field descs + records)
  + (BTCE | BTAB)(4B)
"""

import re
import socket
import struct
import time

import pytest


# ---------------------------------------------------------------------------
# Wire-format constants (mirror apps/btsensor/btsensor_capture_mode.c)
# ---------------------------------------------------------------------------

BTCS = b"BTCS"
BTCE = b"BTCE"
BTAB = b"BTAB"
CAPB = b"CAPB"      # apps/capture/include/capture_format.h file header magic
META_LEN = 40       # struct btcap_session_meta_s; _Static_assert == 40
CAP_HEADER_LEN = 64 # capture_file_header_s
FIELD_DESC_LEN = 48 # capture_field_desc_s


# ---------------------------------------------------------------------------
# btsensor lifecycle helper.  rcS (Issue #111) auto-launches `btsensor start`
# at boot, so we only reboot + poll dmesg for the HCI WORKING banner — an
# explicit `btsensor start` would just race rcS and trip
# `btsensor: already running`.
# ---------------------------------------------------------------------------

_BDADDR_RE = re.compile(
    r"btsensor: HCI working, BD_ADDR "
    r"((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})"
)


def _btsensor_fresh_start(p):
    p.reboot(timeout=15)

    deadline = time.time() + 8.0
    bdaddr = None
    while time.time() < deadline:
        time.sleep(0.5)
        log = p.sendCommand("dmesg", timeout=10)
        m = _BDADDR_RE.search(log)
        if m:
            bdaddr = m.group(1)
            break
    assert bdaddr, "btsensor: HCI working banner not seen within 8 s"
    return bdaddr


# ---------------------------------------------------------------------------
# K-1 — kernel chardev is registered at boot
# ---------------------------------------------------------------------------


def test_capture_dev_node_present(p):
    """K-1: /dev/btcap is registered by the board-local chardev driver.

    The node is created in `boards/spike-prime-hub/src/stm32_btcap_chardev.c`
    via `register_driver` from board bringup.  If this regresses, the
    capture lib's `open(/dev/btcap)` returns ENOENT and the whole
    pipeline is dead.
    """
    out = p.sendCommand("ls /dev/btcap")
    assert "/dev/btcap" in out, f"/dev/btcap not registered: {out!r}"
    assert "No such file" not in out


# ---------------------------------------------------------------------------
# K-2 — `sensor` CLI exposes the capture verb
# ---------------------------------------------------------------------------


def test_sensor_capture_in_help(p):
    """K-2: `sensor` builtin lists the `capture` verb in its usage banner.

    Catches accidental `#ifdef CONFIG_APP_CAPTURE` removal in
    `apps/sensor/sensor_main.c` or a missed Kconfig dependency.
    """
    # Triggering with no args prints usage to stderr+stdout.
    out = p.sendCommand("sensor")
    assert "capture" in out, f"capture verb missing from `sensor` usage: {out!r}"
    # The usage line includes the cue about the BT-side drain.
    assert "btsensor mode capture" in out, (
        f"capture usage hint missing: {out!r}"
    )


# ---------------------------------------------------------------------------
# K-3 — capture rejects cleanly when the live mode has no schema
# ---------------------------------------------------------------------------


def test_capture_unmapped_mode_rejects(p):
    """K-3: `sensor color capture` exits ENOENT-cleanly for an unmapped mode.

    The uORB topic publishes a synthetic sample with `mode_id=0` even
    when no Color Sensor is plugged in, so do_capture() reaches the
    schema lookup; mode 0 has no entry in g_capture_schemas (only modes
    1=Reflection and 5=RGBI), so resolve_capture_schema returns NULL
    and the verb prints the documented error before exiting.

    What this guards: the verb dispatch is wired (CONFIG_APP_CAPTURE
    on), `sensor color` opens its uORB fd, the first sample is read,
    and the schema lookup runs — all without touching /dev/btcap, so
    no chardev session is leaked into the next test.
    """
    out = p.sendCommand("sensor color capture 200", timeout=5)
    assert "no schema for class=color mode=" in out, (
        f"expected schema-resolve error: {out!r}"
    )

    # Sanity: chardev was never registered, node still resolves clean.
    ls = p.sendCommand("ls /dev/btcap")
    assert "/dev/btcap" in ls and "No such" not in ls


# ---------------------------------------------------------------------------
# K-4 — btsensor MODE CAPTURE without a writer in flight rejects cleanly
# ---------------------------------------------------------------------------


def test_btsensor_mode_capture_without_session(p):
    """K-4: `btsensor mode capture` without a writer returns ENOENT path.

    `btsensor_capture_mode_enter()` opens /dev/btcap, queries the session
    meta via BTCAPIOC_GET_SESSION_META, sees state==IDLE -> ENOENT, and
    bails without parking the daemon.  The user-visible string is
    surfaced by `print_action_result(-ENOENT)` in btsensor_main.c.
    """
    _btsensor_fresh_start(p)

    try:
        out = p.sendCommand("btsensor mode capture", timeout=5)
        assert "no capture session in flight" in out, (
            f"expected ENOENT mapping: {out!r}"
        )

        # Bonus: `btsensor status` must still be reachable — the daemon
        # is not stuck on a partial session.
        st = p.sendCommand("btsensor status", timeout=5)
        assert "bt:" in st, f"btsensor status unhealthy after reject: {st!r}"
    finally:
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)


# ---------------------------------------------------------------------------
# K-5 — full BT round-trip (interactive: needs sensor + BlueZ pairing)
# ---------------------------------------------------------------------------


def _scan_session(buf):
    """Parse BTCS/meta/payload/(BTCE|BTAB) from a byte buffer.

    Returns (schema_magic, total_bytes, name, payload, terminator) or
    raises AssertionError.  Mirrors the host-side scanner described in
    the design plan (sanity check on total_bytes + terminator).
    """
    idx = buf.find(BTCS)
    assert idx >= 0, f"no BTCS marker in {len(buf)} bytes"

    meta_off = idx + len(BTCS)
    assert meta_off + META_LEN <= len(buf), (
        f"truncated session: meta does not fit ({len(buf)} bytes from BTCS@{idx})"
    )

    meta = buf[meta_off:meta_off + META_LEN]
    schema_magic, _reserved, total_bytes = struct.unpack_from("<HHI", meta, 0)
    name = meta[8:8 + 32].rstrip(b"\x00").decode(errors="replace")

    # Sanity range: a Reflection record run is dozens of bytes; cap at
    # APP_CAPTURE_MAX_HEAP_BYTES (default 64 KB) plus header.
    assert 64 <= total_bytes <= 100_000, (
        f"implausible total_bytes={total_bytes}"
    )

    payload_off = meta_off + META_LEN
    end_off = payload_off + total_bytes
    assert end_off + 4 <= len(buf), (
        f"truncated session: only {len(buf) - payload_off}/{total_bytes} "
        f"payload bytes received before terminator"
    )

    payload = buf[payload_off:end_off]
    terminator = buf[end_off:end_off + 4]
    assert terminator in (BTCE, BTAB), (
        f"expected BTCE/BTAB after payload, got {terminator!r}"
    )
    return schema_magic, total_bytes, name, payload, terminator


@pytest.mark.interactive
def test_capture_round_trip_via_rfcomm(p):
    """K-5 (semi-manual): full capture round-trip over BT SPP.

    Operator pairs the Hub once via bluetoothctl (same procedure as
    test_bt_spp's H-5/H-8) and connects a SPIKE Color Sensor to port A.
    The test then:

      1. Selects MODE 1 (Reflection, 9-byte schema).
      2. Kicks off `sensor color capture 1500 &` so the writer parks
         in capture_write() waiting for the reader to engage.
      3. Opens AF_BLUETOOTH/RFCOMM ch=1 directly (no BlueZ rfcomm bind).
      4. Issues `btsensor mode capture` — telemetry pauses, BTCS+meta
         is queued, /dev/btcap is drained back-to-back.
      5. Reads RFCOMM bytes for ~5 s and validates the framing,
         CAPB header, and that BTCE (clean end, not BTAB) closes the
         session.

    The host loopback gives us the same lossless guarantee the commit
    message asserts: `dropped_oldest=0 dropped_full=0` on the Hub side
    and a complete BTCS..BTCE on the wire.

    Requires CAP_NET_RAW (rerun pytest with sudo or grant the cap).
    Skipped automatically if AF_BLUETOOTH is unavailable.
    """
    bdaddr = _btsensor_fresh_start(p)

    sock = None
    try:
        # Bring up advertising so the PC can RFCOMM-connect.  IMU /
        # sensor BUNDLE streams stay *off*: the capture path pauses
        # them anyway, and leaving them off keeps the wire bytes
        # entirely capture-framed (easier to parse, and a non-zero
        # dropped_oldest at the end is unambiguously a capture-path
        # regression rather than crosstalk from telemetry).
        out = p.sendCommand("btsensor bt on", timeout=5)
        assert "OK" in out, f"btsensor bt on failed: {out!r}"

        p.waitUser(
            f"Hub is advertising as 'SPIKE-BT-Sensor' at {bdaddr}.\n\n"
            f"Setup checklist:\n"
            f"  1. Plug a SPIKE Color Sensor (45605) into ANY port (A-F).\n"
            f"  2. On Linux host, pair (and trust) the Hub once if not\n"
            f"     already (persists across reboots):\n"
            f"       bluetoothctl\n"
            f"         [bluetooth]# pair  {bdaddr}\n"
            f"         [bluetooth]# trust {bdaddr}\n"
            f"         [bluetooth]# quit\n\n"
            f"Press ENTER when ready.  Pytest will RFCOMM-connect\n"
            f"directly (no `rfcomm bind` needed)."
        )

        # Select Reflection mode and let the LEGO sensor settle.
        out = p.sendCommand("sensor color select 1", timeout=5)
        assert "ERR" not in out, f"select 1 failed: {out!r}"
        time.sleep(1.0)

        # Sanity: a foreground `sensor color watch` must see at least one
        # sample in mode 1 before we attempt a capture, else the operator
        # likely forgot to plug the sensor in.  The watch table prints
        # one row per sample with the mode in the 5th whitespace column
        # (after time_ms, port, gen, seq).
        watch_out = p.sendCommand("sensor color watch 300", timeout=5)
        m = re.search(r"received\s+(\d+)\s+samples", watch_out)
        assert m and int(m.group(1)) > 0, (
            f"Color Sensor not publishing; plug it in:\n{watch_out!r}"
        )
        assert re.search(
            r"^\s*\d+\s+[A-F]\s+\d+\s+\d+\s+1\s+INT8\b",
            watch_out, re.MULTILINE,
        ), (
            f"Color Sensor publishing but not in mode 1; "
            f"check `sensor color select 1`:\n{watch_out!r}"
        )

        # 2. background capture: writer parks waiting for the reader.
        # 1500 ms is short enough that even a hung path frees us inside
        # the test timeout, long enough to capture ~14 reflection
        # samples at the sensor's ~104 ms cadence.
        spawn = p.sendCommand("sensor color capture 1500 &", timeout=5)
        # NSH echoes "<cmd> &  [pid]"; we don't need the pid for the
        # success path but the line must come back without an error.
        assert "ERR" not in spawn and "command not found" not in spawn, spawn

        # Give the capture phase + capture_init() time to land — the
        # writer must reach the chardev REGISTER before we trip the
        # reader, otherwise btsensor sees ENOENT instead of engaging.
        time.sleep(2.0)

        # 3. RFCOMM connect.
        try:
            sock = socket.socket(socket.AF_BLUETOOTH,
                                 socket.SOCK_STREAM,
                                 socket.BTPROTO_RFCOMM)
        except OSError as exc:
            pytest.skip(f"AF_BLUETOOTH socket unavailable: {exc} "
                        f"(needs Linux + CAP_NET_RAW; rerun with sudo)")
        sock.settimeout(5.0)
        try:
            sock.connect((bdaddr, 1))
        except OSError as exc:
            pytest.fail(
                f"BTPROTO_RFCOMM connect to {bdaddr} ch=1 failed: {exc}. "
                f"Pair+trust the Hub in BlueZ first (see "
                f"docs/{{ja,en}}/development/pc-receive-spp.md)."
            )

        # 4. Trigger the drain.
        out = p.sendCommand("btsensor mode capture", timeout=10)
        assert "OK" in out and "ERR" not in out, (
            f"`btsensor mode capture` failed: {out!r}"
        )

        # 5. Drain RFCOMM bytes until BTCE/BTAB or 5 s elapses.
        buf = bytearray()
        sock.settimeout(0.5)
        deadline = time.time() + 5.0
        while time.time() < deadline:
            try:
                chunk = sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            buf += chunk
            # Stop early once we have a complete session.
            if BTCS in buf and (BTCE in buf or BTAB in buf):
                # Make sure terminator follows the payload — scan_session
                # double-checks layout.  Only break when scan succeeds.
                try:
                    _scan_session(buf)
                    break
                except AssertionError:
                    pass

        assert len(buf) >= len(BTCS) + META_LEN + CAP_HEADER_LEN + 4, (
            f"too few bytes received ({len(buf)})"
        )

        schema_magic, total_bytes, name, payload, terminator = _scan_session(buf)

        # color_reflection_run schema: magic 0x0010 in capture_schema_init.h.
        assert schema_magic == 0x0010, (
            f"unexpected schema_magic 0x{schema_magic:04x} (want 0x0010)"
        )
        assert "color_reflection_run" in name, f"unexpected schema name {name!r}"

        # .cap header sanity: magic == CAPB at offset 0, version == 1 at +4.
        assert payload[:4] == CAPB, (
            f"payload does not start with CAPB: {payload[:8]!r}"
        )
        version = struct.unpack_from("<H", payload, 4)[0]
        assert version == 1, f"unexpected .cap version {version}"

        # Header schema_magic must match the meta the host received.
        cap_schema_magic = struct.unpack_from("<H", payload, 6)[0]
        assert cap_schema_magic == schema_magic, (
            f".cap schema_magic mismatch: header 0x{cap_schema_magic:04x} "
            f"vs meta 0x{schema_magic:04x}"
        )

        record_size = struct.unpack_from("<I", payload, 16)[0]
        record_count = struct.unpack_from("<I", payload, 20)[0]
        field_count = payload[56]
        assert record_size > 0 and record_count > 0, (
            f"empty session: record_size={record_size} count={record_count}"
        )

        # Payload size must equal header + field_count*48 + records.
        expected_payload = (CAP_HEADER_LEN
                            + field_count * FIELD_DESC_LEN
                            + record_size * record_count)
        assert expected_payload == total_bytes == len(payload), (
            f"size mismatch: header expects {expected_payload}, "
            f"meta says {total_bytes}, got {len(payload)} on the wire"
        )

        # Clean end (BTCE) — BTAB means truncated/aborted session.
        assert terminator == BTCE, (
            f"session ended with {terminator!r}, not BTCE — drain was truncated"
        )

        # Hub-side counters must agree with the wire: dropped_full=0
        # is the lossless guarantee from commit 6a4740d (Issue #122).
        status = p.sendCommand("btsensor status", timeout=5)
        m = re.search(
            r"frames:\s*sent=(\d+)\s+dropped_oldest=(\d+)\s+dropped_full=(\d+)",
            status,
        )
        assert m, f"btsensor status frames line missing: {status!r}"
        dropped_oldest = int(m.group(2))
        dropped_full = int(m.group(3))
        assert dropped_full == 0, (
            f"capture path dropped frames (dropped_full={dropped_full}); "
            f"the back-pressure path regressed: {status!r}"
        )
        # dropped_oldest may be non-zero if telemetry was racing right
        # at the MODE switch boundary; the capture path itself uses
        # try_enqueue + back-pressure so the *capture* contribution is 0.
        # Surface the value in the failure message but don't gate on it.
        if dropped_oldest != 0:
            pytest.fail(
                f"unexpected dropped_oldest={dropped_oldest} "
                f"(expected 0 for a quiet capture run): {status!r}"
            )
    finally:
        if sock is not None:
            try:
                sock.close()
            except Exception:
                pass
        # apps/sensor terminates on its own once the drain completes.
        # If the test failed mid-flight, the daemon teardown below is
        # enough — chardev release fop reclaims any orphaned session.
        p.sendCommand("btsensor bt off", timeout=5)
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)
