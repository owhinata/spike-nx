"""Category H: btstack-based Classic BT SPP (Issue #52, #56).

These tests verify the Hub-side half of the IMU→SPP stream:
the CC2564C brings HCI to WORKING on /dev/ttyBT, the btsensor NSH
builtin spawns its daemon, and the daemon reaches HCI_STATE_WORKING.

After Issue #56 btsensor is a start/stop daemon (no longer `btsensor &`)
and its banners go through syslog → RAMLOG instead of stdout, so the
HCI_WORKING check polls `dmesg` rather than the live console.

H-5 is semi-automated: the operator pairs the Hub once via
``bluetoothctl`` (covered in docs/{ja,en}/development/pc-receive-spp.md)
and pytest then opens BTPROTO_RFCOMM directly to receive + validate
the stream — no ``rfcomm bind`` / ``cat /dev/rfcomm0`` step.
"""

import re
import socket
import time

import pytest


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
    p.sendCommand("dmesg", timeout=5)

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
            log = p.sendCommand("dmesg", timeout=5)
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
    # Reboot first so HCI bring-up is clean (mirrors H-4).
    p.reboot(timeout=15)

    out = p.sendCommand("btsensor start", timeout=5)
    assert "started (pid" in out, f"btsensor start failed: {out!r}"

    # Wait for HCI WORKING and capture BD_ADDR from the RAMLOG banner.
    bdaddr_pat = re.compile(
        r"btsensor: HCI working, BD_ADDR "
        r"((?:[0-9A-Fa-f]{2}:){5}[0-9A-Fa-f]{2})"
    )
    bdaddr = None
    deadline = time.time() + 6.0
    while time.time() < deadline:
        time.sleep(0.5)
        log = p.sendCommand("dmesg", timeout=5)
        m = bdaddr_pat.search(log)
        if m:
            bdaddr = m.group(1)
            break
    assert bdaddr, "btsensor: HCI working banner not seen within 6 s"

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
                    r"frames:\s*sent=(\d+)\s+dropped=(\d+)", status
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
            # Issue #56 Commit E moved the frame magic from 0xA55A
            # to 0xB66B (BTSENSOR_FRAME_MAGIC); little-endian on the
            # wire so the byte pattern is 6b b6.
            assert b"\x6b\xb6" in buf, (
                f"magic 0xB66B not found in {len(buf)} received bytes"
            )

            # Hub-side: frames-sent counter must have advanced.  At
            # ODR 833 Hz / batch 8 (~104 fps) we expect ~312 frames in
            # 3 s; require ≥100 to absorb stack and host jitter.
            status2 = p.sendCommand("btsensor status", timeout=5)
            m = re.search(
                r"frames:\s*sent=(\d+)\s+dropped=(\d+)", status2
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
            magic_count = buf.count(b"\x6b\xb6")
            assert magic_count >= 50, (
                f"PC received only {magic_count} magic markers in "
                f"{len(buf)} bytes — stream is not flowing"
            )
        finally:
            sock.close()
    finally:
        p.sendCommand("btsensor imu off", timeout=5)
        p.sendCommand("btsensor bt off", timeout=5)
        p.sendCommand("btsensor stop", timeout=5)
        time.sleep(1)
