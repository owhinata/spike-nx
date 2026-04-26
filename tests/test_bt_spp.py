"""Category H: btstack-based Classic BT SPP (Issue #52, #56).

These tests verify the Hub-side half of the IMU→SPP stream:
the CC2564C brings HCI to WORKING on /dev/ttyBT, the btsensor NSH
builtin spawns its daemon, and the daemon reaches HCI_STATE_WORKING.

After Issue #56 btsensor is a start/stop daemon (no longer `btsensor &`)
and its banners go through syslog → RAMLOG instead of stdout, so the
HCI_WORKING check polls `dmesg` rather than the live console.

PC-side receive verification (pair, RFCOMM bind, magic decode) lives
under docs/{ja,en}/development/pc-receive-spp.md and requires a Linux or
macOS host, so it is intentionally left as a manual procedure rather
than an automated test.
"""

import re
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
    """H-5 (manual): exercise the SPP pair + RFCOMM stream on a PC.

    This test only prompts the operator; it cannot be automated from
    the Hub side because it requires a paired Linux or macOS host.
    See docs/{ja,en}/development/pc-receive-spp.md for the exact
    commands.
    """
    p.waitUser(
        "From a Linux or macOS host:\n"
        "  - pair the Hub (blueutil / bluetoothctl)\n"
        "  - open /dev/rfcomm0 (Linux) or use the PyObjC helper (macOS)\n"
        "  - confirm the stream starts with magic 5a a5 ...\n"
        "Press ENTER here to close out the test."
    )
