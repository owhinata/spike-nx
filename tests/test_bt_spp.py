"""Category H: btstack-based Classic BT SPP (Issue #52, #56).

These tests verify the Hub-side half of the IMU→SPP stream:
the CC2564C brings HCI to WORKING on /dev/ttyBT, the btsensor NSH
builtin spawns its daemon, and the daemon reaches HCI_STATE_WORKING.

After Issue #56 btsensor is a start/stop daemon (no longer `btsensor &`)
and its banners go through syslog → RAMLOG instead of stdout, so the
HCI_WORKING check polls `dmesg` rather than the live console.

H-5/H-8 are semi-automated: the operator pairs the Hub once via
``bluetoothctl`` (covered in docs/{ja,en}/development/pc-receive-spp.md)
and pytest then opens BTPROTO_RFCOMM directly to drive the link — no
``rfcomm bind`` / ``cat /dev/rfcomm0`` step.  H-6/H-7 require a
physical BT button press and stay manual.
"""

import re
import socket
import time

import pytest


# ---------------------------------------------------------------------------
# Helpers shared across H-* tests
# ---------------------------------------------------------------------------

_BDADDR_RE = re.compile(
    r"btsensor: HCI working, BD_ADDR "
    r"((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})"
)


def _btsensor_fresh_start(p):
    """Reboot, start the daemon, and return the parsed BD_ADDR.

    Assumes the caller will issue cleanup (`btsensor stop`) in its
    own ``finally`` block.  Reboot is required because btstack restart
    is reliable only from a cold-boot chip state on this firmware.
    """
    p.reboot(timeout=15)
    out = p.sendCommand("btsensor start", timeout=5)
    assert "started (pid" in out, f"btsensor start failed: {out!r}"

    deadline = time.time() + 6.0
    bdaddr = None
    while time.time() < deadline:
        time.sleep(0.5)
        log = p.sendCommand("dmesg", timeout=10)
        m = _BDADDR_RE.search(log)
        if m:
            bdaddr = m.group(1)
            break
    assert bdaddr, "btsensor: HCI working banner not seen within 6 s"
    return bdaddr


def _wait_status_field(p, field_re, timeout=3.0, interval=0.5):
    """Poll ``btsensor status`` until ``field_re`` matches; return the
    full status text on success, raise AssertionError on timeout.
    """
    deadline = time.time() + timeout
    out = ""
    while time.time() < deadline:
        out = p.sendCommand("btsensor status", timeout=5)
        if re.search(field_re, out):
            return out
        time.sleep(interval)
    raise AssertionError(
        f"status field {field_re!r} not seen within {timeout} s; last: {out!r}"
    )


def test_bt_chardev_exists(p):
    """H-1: /dev/ttyBT is registered by the board-local chardev."""
    output = p.sendCommand("ls /dev/ttyBT")
    assert "/dev/ttyBT" in output, (
        f"/dev/ttyBT not registered: {output!r}"
    )
    assert "No such file" not in output


def test_bt_powered_banner(p):
    """H-2: dmesg shows the CC2564C power-on banner."""
    output = p.sendCommand("dmesg")
    assert "BT: CC2564C powered, /dev/ttyBT ready" in output, (
        f"CC2564C power banner missing: {output!r}"
    )


def test_bt_btsensor_builtin(p):
    """H-3: the btsensor NSH builtin is available.

    `grep` is not enabled in the usbnsh defconfig, so we read the full
    `help` listing and substring-match instead of piping.
    """
    output = p.sendCommand("help")
    assert "btsensor" in output, (
        f"btsensor builtin not registered: {output!r}"
    )


def test_bt_btsensor_hci_working(p):
    """H-4: `btsensor start` reaches HCI_STATE_WORKING within a few
    seconds and logs a valid BD address.

    Banners come from syslog → RAMLOG (CONFIG_RAMLOG_SYSLOG=y, no
    SYSLOG_CONSOLE), so we poll `dmesg` instead of expecting on the
    live serial console.  We reboot first so the btstack state is
    pristine — empirically, `btsensor start` after the rest of the
    test session has run sometimes fails to reach HCI_STATE_WORKING
    (the action queue then returns ``timed out``).  A clean boot
    consistently brings HCI up within ~1 s.
    """
    p.reboot(timeout=15)

    # Drain pre-existing RAMLOG content so the banner search is not
    # confused by leftovers from earlier sessions.  RAMLOG_NONBLOCK is
    # off so dmesg blocks-then-returns the buffered text.
    p.sendCommand("dmesg", timeout=10)

    out = p.sendCommand("btsensor start", timeout=5)
    assert "started (pid" in out, f"btsensor start failed: {out!r}"

    # HCI bring-up over the CC2564C UART completes in ~1 s on a fresh
    # boot; allow up to 6 s to absorb jitter.
    # bd_addr_to_str() prints uppercase hex (e.g. F8:2E:0C:...).
    pattern = re.compile(
        r"btsensor: HCI working, BD_ADDR "
        r"((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})"
    )
    bdaddr = None
    deadline = time.time() + 6.0
    try:
        while time.time() < deadline:
            time.sleep(0.5)
            log = p.sendCommand("dmesg", timeout=10)
            m = pattern.search(log)
            if m:
                bdaddr = m.group(1)
                break

        assert bdaddr is not None, (
            "btsensor: HCI working banner not seen within 6 s"
        )
        assert bdaddr != "00:00:00:00:00:00", f"BD address all zeros: {bdaddr}"
        assert bdaddr != "ff:ff:ff:ff:ff:ff", f"BD address all ones: {bdaddr}"
    finally:
        # Always stop so subsequent tests start from a clean state.
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)


@pytest.mark.interactive
def test_bt_pc_pair_and_stream(p):
    """H-5 (semi-manual): pair the Hub once, then let pytest receive
    the SPP stream itself via BTPROTO_RFCOMM.

    Issue #56 made BT/IMU off-by-default, so the test does the Hub-side
    bring-up automatically: ``reboot`` → ``btsensor start`` →
    ``btsensor bt on`` → ``btsensor imu on``.  The HCI WORKING banner
    in the RAMLOG also gives us the BD address to connect to.

    The operator's only job is the one thing that cannot be driven
    from the Hub: pair (and trust) the device once via ``bluetoothctl``
    so BlueZ has a link key.  After ENTER, pytest opens a raw
    ``AF_BLUETOOTH/SOCK_STREAM/BTPROTO_RFCOMM`` socket directly to the
    Hub (channel 1) — no ``rfcomm bind`` / ``cat /dev/rfcomm0`` dance —
    receives bytes for ~3 s, and validates both PC-side magic + frame
    count and Hub-side ``btsensor status`` counters.

    Requires CAP_NET_RAW (run pytest with sudo, or grant the
    capability to the python interpreter).  Skipped automatically if
    the socket cannot be created.

    See docs/{ja,en}/development/pc-receive-spp.md for the PC-side
    pairing commands.
    """
    bdaddr = _btsensor_fresh_start(p)

    try:
        out = p.sendCommand("btsensor bt on", timeout=5)
        assert "OK" in out, f"btsensor bt on failed: {out!r}"
        out = p.sendCommand("btsensor imu on", timeout=5)
        assert "OK" in out, f"btsensor imu on failed: {out!r}"

        p.waitUser(
            f"Hub is now advertising as 'SPIKE-BT-Sensor' at\n"
            f"  BD_ADDR = {bdaddr}\n"
            f"with IMU sampling on.\n\n"
            f"On a Linux host (one-time, persists across reboots):\n"
            f"  bluetoothctl\n"
            f"    [bluetooth]# power on\n"
            f"    [bluetooth]# scan on\n"
            f"    [bluetooth]# pair {bdaddr}\n"
            f"    [bluetooth]# trust {bdaddr}\n"
            f"    [bluetooth]# quit\n"
            f"(skip if already paired+trusted)\n\n"
            f"Press ENTER — pytest will open BTPROTO_RFCOMM directly,\n"
            f"no rfcomm bind/cat needed."
        )

        # Open AF_BLUETOOTH RFCOMM socket directly.  This activates
        # the channel from the BlueZ side, so the Hub flips to
        # bt: paired with cid != 0.
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
            sock.close()
            pytest.fail(
                f"BTPROTO_RFCOMM connect to {bdaddr} ch=1 failed: {exc}. "
                f"Make sure the Hub is paired + trusted in BlueZ and that "
                f"no other process is holding the channel."
            )

        # Snapshot Hub state baseline.  After connect() succeeds the
        # Hub should be paired with cid != 0 within ~1 s.
        try:
            time.sleep(0.5)
            bt_state = None
            cid = 0
            sent1 = 0
            dropped1 = 0
            deadline = time.time() + 5.0
            while time.time() < deadline:
                status = p.sendCommand("btsensor status", timeout=5)
                bt_state = re.search(r"bt:\s*(\S+)", status).group(1)
                cid = int(re.search(r"rfcomm cid:\s*(\d+)", status).group(1))
                m = re.search(
                    r"frames:\s*sent=(\d+)\s+dropped_oldest=(\d+)\s+dropped_full=(\d+)",
                    status,
                )
                sent1 = int(m.group(1))
                dropped1 = int(m.group(2))
                if bt_state == "paired" and cid != 0:
                    break
                time.sleep(0.5)
            assert bt_state == "paired", (
                f"BT state should be 'paired' after RFCOMM socket "
                f"connect, got '{bt_state}'"
            )
            assert cid != 0, (
                f"rfcomm cid should be non-zero, got {cid}"
            )

            # PC-side: drain bytes for 3 s and verify the wire format.
            buf = bytearray()
            t_end = time.time() + 3.0
            sock.settimeout(0.5)
            while time.time() < t_end:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk

            assert len(buf) > 0, "no RFCOMM bytes received in 3 s"
            # Issue #88 BUNDLE wire format: magic 0xB66B + type byte 0x02.
            # Little-endian on the wire so the marker pattern is 6b b6 02.
            assert b"\x6b\xb6\x02" in buf, (
                f"BUNDLE marker (6b b6 02) not in {len(buf)} received bytes"
            )

            # Hub-side: frames-sent counter must have advanced.  At
            # the 100 Hz BUNDLE tick we expect ~300 frames in 3 s;
            # require ≥100 to absorb stack and host jitter.
            status2 = p.sendCommand("btsensor status", timeout=5)
            m = re.search(
                r"frames:\s*sent=(\d+)\s+dropped_oldest=(\d+)\s+dropped_full=(\d+)",
                status2,
            )
            sent2 = int(m.group(1))
            dropped2 = int(m.group(2))
            delta_sent = sent2 - sent1
            delta_dropped = dropped2 - dropped1
            assert delta_sent >= 100, (
                f"Hub shipped only {delta_sent} frames in 3 s "
                f"(sent {sent1} -> {sent2})"
            )
            assert delta_dropped <= delta_sent // 4, (
                f"Hub dropped {delta_dropped} of {delta_sent} frames "
                f"(>25%) in 3 s — link is not healthy"
            )

            # PC-side magic count gives an upper bound on frames the
            # PC actually received.  Allow generous slack relative to
            # delta_sent because RFCOMM-level fragmentation may leave
            # a partial frame at the trailing edge.
            marker_count = buf.count(b"\x6b\xb6\x02")
            assert marker_count >= 50, (
                f"PC received only {marker_count} BUNDLE markers in "
                f"{len(buf)} bytes — stream is not flowing"
            )
        finally:
            sock.close()
    finally:
        p.sendCommand("btsensor imu off", timeout=5)
        p.sendCommand("btsensor bt off", timeout=5)
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)


@pytest.mark.interactive
def test_bt_button_short_press(p):
    """H-6 (manual): short press transitions BT off → advertising.

    The kernel-side btbutton driver decodes the LRB resistor ladder at
    50 ms cadence with 5-sample debounce (~250 ms), and the BT state
    machine runs on the btstack thread.  After ENTER we poll
    ``btsensor status`` for up to 3 s before declaring the press lost.
    """
    _btsensor_fresh_start(p)

    try:
        out = _wait_status_field(p, r"bt:\s+off")
        assert re.search(r"bt:\s+off", out), out

        # Drain dmesg so the post-press dmesg only shows new events.
        p.sendCommand("dmesg", timeout=10)

        p.waitUser(
            "Hub の BT ボタンを 1 回短押し (0.5 秒以下で離す) してから ENTER"
        )

        out = _wait_status_field(p, r"bt:\s+advertising", timeout=3.0)
        log = p.sendCommand("dmesg", timeout=10)
        assert "btsensor_button: short press" in log, log
        assert "btsensor: BT advertising (was off)" in log, log
    finally:
        p.sendCommand("btsensor bt off", timeout=5)
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)


@pytest.mark.interactive
def test_bt_button_long_press(p):
    """H-7 (manual): long press transitions BT advertising → off.

    Advertising is brought up via the NSH ``bt on`` path so the
    operator only has to perform one button press for this test.
    """
    _btsensor_fresh_start(p)

    try:
        out = p.sendCommand("btsensor bt on", timeout=5)
        assert "OK" in out, f"bt on failed: {out!r}"
        _wait_status_field(p, r"bt:\s+advertising")

        p.sendCommand("dmesg", timeout=10)

        p.waitUser(
            "Hub の BT ボタンを 2 秒以上長押ししてから離して ENTER"
        )

        _wait_status_field(p, r"bt:\s+off", timeout=3.0)
        log = p.sendCommand("dmesg", timeout=10)
        assert "btsensor_button: long press" in log, log
        assert "btsensor: BT off (was advertising)" in log, log
    finally:
        p.sendCommand("btsensor bt off", timeout=5)
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)


# ---------------------------------------------------------------------------
# H-8 helpers — RFCOMM ASCII command transport
# ---------------------------------------------------------------------------

def _rfcomm_send_command(sock, cmd, timeout=2.0):
    """Send ``cmd + '\\n'`` and return the matched reply line.

    The wire may carry binary IMU frames intermixed with ASCII replies
    (the btsensor TX arbiter prioritises responses but does not
    guarantee they arrive on a frame boundary), so we scan the
    incoming byte stream for the next ``OK\\n`` or ``ERR ...\\n`` line
    and return that.  Returns the reply (e.g. ``"OK"`` or
    ``"ERR busy"``) or ``None`` on timeout.
    """
    sock.sendall((cmd + "\n").encode())
    sock.settimeout(0.5)
    buf = bytearray()
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            chunk = sock.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            break
        buf += chunk
        if b"OK\n" in buf:
            return "OK"
        m = re.search(rb"ERR[^\n]*\n", buf)
        if m:
            return m.group(0).decode().strip()
    return None


@pytest.mark.interactive
def test_bt_rfcomm_command_suite(p):
    """H-8 (semi-manual): operator pairs once, then pytest exercises
    the full PC→Hub ASCII command surface and the IMU stream end to
    end.

    The Hub-side ``btsensor_cmd.c`` parser accepts ``IMU ON|OFF`` /
    ``SENSOR ON|OFF`` and ``SET ODR|ACCEL_FSR|GYRO_FSR <value>`` (Issue
    #88).  All mutators return ``-EBUSY`` while sampling is active, so
    the SET cases run with IMU off and the BUSY case runs after IMU on.
    ``SET BATCH`` is removed and ``SET ODR > 833`` returns
    ``ERR invalid_odr``.

    Requires CAP_NET_RAW (rerun pytest with sudo if the AF_BLUETOOTH
    socket cannot be created).  Skipped automatically in that case.
    """
    bdaddr = _btsensor_fresh_start(p)

    try:
        out = p.sendCommand("btsensor bt on", timeout=5)
        assert "OK" in out, f"bt on failed: {out!r}"

        p.waitUser(
            f"Hub is advertising as 'SPIKE-BT-Sensor' at {bdaddr}.\n"
            f"On the Linux host pair (and trust) once if not already:\n"
            f"  bluetoothctl\n"
            f"    [bluetooth]# pair {bdaddr}\n"
            f"    [bluetooth]# trust {bdaddr}\n"
            f"    [bluetooth]# quit\n"
            f"Press ENTER once pairing is in place."
        )

        try:
            sock = socket.socket(socket.AF_BLUETOOTH,
                                 socket.SOCK_STREAM,
                                 socket.BTPROTO_RFCOMM)
        except OSError as exc:
            pytest.skip(f"AF_BLUETOOTH unavailable: {exc} "
                        f"(needs Linux + CAP_NET_RAW; rerun with sudo)")
        sock.settimeout(5.0)
        try:
            sock.connect((bdaddr, 1))
        except OSError as exc:
            sock.close()
            pytest.fail(
                f"BTPROTO_RFCOMM connect to {bdaddr} ch=1 failed: {exc}"
            )

        try:
            _wait_status_field(p, r"bt:\s+paired", timeout=5.0)

            # Phase 1: IMU is off → SET commands all succeed.
            assert _rfcomm_send_command(sock, "SET ODR 416") == "OK"
            assert _rfcomm_send_command(sock, "SET ACCEL_FSR 4") == "OK"
            assert _rfcomm_send_command(sock, "SET GYRO_FSR 1000") == "OK"

            status = p.sendCommand("btsensor status", timeout=5)
            assert "odr=416Hz" in status, status
            assert "accel_fsr=4g" in status, status
            assert "gyro_fsr=1000dps" in status, status

            # Phase 2: invalid arguments produce ERR.
            r = _rfcomm_send_command(sock, "SET ODR 1660")  # > 833 cap
            assert r is not None and r.startswith("ERR"), r
            r = _rfcomm_send_command(sock, "SET FOO 1")
            assert r is not None and r.startswith("ERR"), r
            r = _rfcomm_send_command(sock, "BAD")
            assert r is not None and r.startswith("ERR"), r
            # SET BATCH is removed in Issue #88.
            r = _rfcomm_send_command(sock, "SET BATCH 16")
            assert r is not None and r.startswith("ERR"), r

            # Phase 3: SENSOR ON/OFF round-trip (LEGO TLV streaming).
            assert _rfcomm_send_command(sock, "SENSOR ON") == "OK"
            status = p.sendCommand("btsensor status", timeout=5)
            assert re.search(r"sensor:\s+on", status), status

            # Issue #91: write commands.  Without a physical LEGO sensor
            # we cannot verify the actuator effect, so just check that
            # the parser recognises each command (returns OK or ERR
            # rather than `ERR unknown ...`).  ERR replies are fine when
            # no device is bound — the firmware still went through the
            # cmd_sensor branch.
            for write_cmd in [
                "SENSOR MODE color 0",
                "SENSOR SEND color 3 64",
                "SENSOR PWM color 0 0 0",
                "SENSOR PWM force 1",        # force has no PWM → ERR not_supported
                "SENSOR MODE bogus 0",       # invalid class
                "SENSOR PWM color 0 0 0 0",  # too many channels
            ]:
                r = _rfcomm_send_command(sock, write_cmd)
                assert r is not None, write_cmd
                assert not r.startswith("ERR unknown"), (
                    f"{write_cmd!r} produced {r!r} — parser did not recognise it"
                )

            assert _rfcomm_send_command(sock, "SENSOR OFF") == "OK"

            # Phase 4: IMU ON, then verify BUNDLE frames flow on the
            # wire and carry the parameters we just set.
            assert _rfcomm_send_command(sock, "IMU ON") == "OK"
            status = p.sendCommand("btsensor status", timeout=5)
            assert re.search(r"imu:\s+on", status), status

            # Drain ~2 s of frames + verify magic + header fields.
            time.sleep(0.5)
            sock.settimeout(0.5)
            buf = bytearray()
            t_end = time.time() + 2.0
            while time.time() < t_end:
                try:
                    chunk = sock.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buf += chunk

            assert len(buf) > 0, "no RFCOMM bytes received with IMU on"
            assert b"\x6b\xb6\x02" in buf, (
                f"BUNDLE marker (6b b6 02) not in {len(buf)} bytes"
            )

            # Parse the first complete BUNDLE and verify the SET fields
            # are echoed back in the bundle header.  Issue #88 layout:
            #   [0:2]    magic  = 0xB66B
            #   [2]      type   = 0x02 (BUNDLE)
            #   [3:5]    frame_len
            #   bundle hdr at offset 5..20:
            #     +5  seq               (u16)
            #     +7  tick_ts_us        (u32)
            #     +11 imu_section_len   (u16)
            #     +13 imu_sample_count  (u8)
            #     +14 tlv_count         (u8 = 6)
            #     +15 imu_sample_rate_hz(u16) = 416
            #     +17 imu_accel_fsr_g   (u8)  = 4
            #     +18 imu_gyro_fsr_dps  (u16) = 1000
            #     +20 flags             (u8)
            idx = buf.find(b"\x6b\xb6\x02")
            assert idx >= 0 and len(buf) >= idx + 21, (
                f"truncated bundle at idx={idx}, total={len(buf)}"
            )
            envelope_and_hdr = buf[idx:idx + 21]
            assert envelope_and_hdr[2] == 0x02, (
                f"frame type {envelope_and_hdr[2]:#x} != 0x02"
            )
            tlv_count = envelope_and_hdr[5 + 9]
            assert tlv_count == 6, f"tlv_count {tlv_count} != 6"
            rate = int.from_bytes(envelope_and_hdr[5 + 10:5 + 12], "little")
            accel_fsr = envelope_and_hdr[5 + 12]
            gyro_fsr = int.from_bytes(envelope_and_hdr[5 + 13:5 + 15], "little")
            assert rate == 416, f"sample_rate_hz {rate} != 416"
            assert accel_fsr == 4, f"accel_fsr_g {accel_fsr} != 4"
            assert gyro_fsr == 1000, f"gyro_fsr_dps {gyro_fsr} != 1000"

            # Phase 5: SET while IMU is on returns ERR busy.
            r = _rfcomm_send_command(sock, "SET ODR 833")
            assert r is not None and "busy" in r.lower(), r

            # Phase 6: IMU OFF cleanly stops the stream.
            assert _rfcomm_send_command(sock, "IMU OFF") == "OK"
            status = p.sendCommand("btsensor status", timeout=5)
            assert re.search(r"imu:\s+off", status), status
        finally:
            sock.close()
    finally:
        p.sendCommand("btsensor imu off", timeout=5)
        p.sendCommand("btsensor sensor off", timeout=5)
        p.sendCommand("btsensor bt off", timeout=5)
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)
