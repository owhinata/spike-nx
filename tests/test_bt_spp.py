"""Category H: btstack-based Classic BT SPP (Issue #52).

These tests verify the Hub-side half of the IMU→SPP stream:
the CC2564C brings HCI to WORKING on /dev/ttyBT, the btsensor NSH
command is registered and powers on the SPP service.

PC-side receive verification (pair, RFCOMM bind, magic decode) lives
under docs/{ja,en}/development/pc-receive-spp.md and requires a Linux or
macOS host, so it is intentionally left as a manual procedure rather
than an automated test.
"""

import re

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
    """H-3: the btsensor NSH builtin is available."""
    output = p.sendCommand("help | grep btsensor")
    assert "btsensor" in output, (
        f"btsensor builtin not registered: {output!r}"
    )


def test_bt_btsensor_hci_working(p):
    """H-4: `btsensor &` reaches HCI_STATE_WORKING within a few seconds
    and prints a valid BD address.  The background launch leaves the
    task alive so subsequent tests exercise the same stack instance —
    those tests must tolerate an already-running daemon.
    """
    p.sendCommand("btsensor &", timeout=2)

    # The HCI_STATE_WORKING banner lands asynchronously; scan the
    # console output until it appears (the default sendCommand prompt
    # wait races with the async print, so we drive pexpect directly).
    p.proc.expect(r"btsensor: HCI working, BD_ADDR "
                  r"([0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:"
                  r"[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2})",
                  timeout=5)

    bdaddr = p.proc.match.group(1).decode()
    assert bdaddr != "00:00:00:00:00:00", f"BD address all zeros: {bdaddr}"
    assert bdaddr != "ff:ff:ff:ff:ff:ff", f"BD address all ones: {bdaddr}"

    # Consume the prompt that follows the banner so later tests start
    # from a clean state.
    p.proc.expect(r"nsh> ", timeout=2)


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
