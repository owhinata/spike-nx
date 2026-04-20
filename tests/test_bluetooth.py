"""Category H: CC2564C Bluetooth tests (Issue #47).

These tests verify that the SPIKE Prime Hub's CC2564C controller has
completed bring-up (firmware patch load, baud switch to 3 Mbps) and is
registered as a NuttX bnep0 netdev reachable by btsak.
"""

import re
import time

import pytest


def test_bt_bringup_dmesg(p):
    """H-1: dmesg reports "CC2564C ready at 3000000 bps" with no failure."""
    output = p.sendCommand("dmesg")
    assert "BT: CC2564C ready at 3000000 bps" in output, (
        f"Bring-up banner missing: {output!r}"
    )
    assert "btuart_register failed" not in output, (
        f"btuart_register failed during bring-up: {output!r}"
    )
    assert "stm32_bluetooth_initialize() failed" not in output, (
        f"stm32_bluetooth_initialize failed: {output!r}"
    )


def test_bt_netdev_visible(p):
    """H-2: ifconfig lists bnep0."""
    output = p.sendCommand("ifconfig")
    assert "bnep0" in output, f"bnep0 netdev not registered: {output!r}"


def test_bt_info_bdaddr(p):
    """H-3: btsak info returns a valid 48-bit BD address."""
    output = p.sendCommand("bt bnep0 info")
    m = re.search(
        r"BDAddr:\s+([0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:"
        r"[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2})",
        output,
    )
    assert m, f"BDAddr not found in bt info output: {output!r}"

    bdaddr = m.group(1)
    assert bdaddr != "00:00:00:00:00:00", f"BDAddr is all zeros: {bdaddr}"
    assert bdaddr != "ff:ff:ff:ff:ff:ff", f"BDAddr is all ones: {bdaddr}"


def test_bt_info_buffer_pool(p):
    """H-4: btsak info reports ACL buffer pool; rejects controllers that
    failed HCI bring-up (Free=0 or Max ACL=0)."""
    output = p.sendCommand("bt bnep0 info")

    free_match = re.search(r"Free:\s+(\d+)", output)
    assert free_match, f"Free slot line missing: {output!r}"
    free_slots = int(free_match.group(1))
    assert free_slots > 0, f"No free ACL slots — chip did not initialise: {output!r}"

    acl_max_match = re.search(r"Max:\s*\n\s*ACL:\s+(\d+)", output)
    assert acl_max_match, f"Max ACL slot count missing: {output!r}"
    assert int(acl_max_match.group(1)) > 0, (
        f"Max ACL slots is zero: {output!r}"
    )


@pytest.mark.interactive
def test_bt_scan(p):
    """H-5: btsak scan detects at least one nearby BT peer (requires a
    phone / BT device in discovery mode near the hub)."""
    p.sendCommand("bt bnep0 scan start", timeout=5)

    try:
        time.sleep(3)
        output = p.sendCommand("bt bnep0 scan get", timeout=5)
    finally:
        p.sendCommand("bt bnep0 scan stop", timeout=5)

    # btsak scan get prints one line per discovered peer (address + RSSI).
    peer_lines = re.findall(
        r"[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}:"
        r"[0-9a-f]{2}:[0-9a-f]{2}:[0-9a-f]{2}",
        output,
    )
    assert peer_lines, (
        "No BT peers discovered — ensure a phone / BT device is advertising "
        "nearby before running this test."
    )
    p.waitUser(
        f"Confirm: did btsak scan find the expected nearby device?\n{output}"
    )
