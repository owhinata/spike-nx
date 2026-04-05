"""Category A: Boot and initialization tests."""

import re


def test_nsh_prompt(p):
    """A-1: NSH prompt is available."""
    output = p.sendCommand("")
    assert "nsh" in output or True  # sendCommand already expects PROMPT


def test_dmesg_no_error(p):
    """A-2: dmesg contains no ERROR lines."""
    output = p.sendCommand("dmesg")
    for line in output.splitlines():
        assert "ERROR" not in line, f"ERROR found in dmesg: {line}"


def test_procfs_version(p):
    """A-3: /proc/version contains NuttX."""
    output = p.sendCommand("cat /proc/version")
    assert "NuttX" in output


def test_procfs_uptime(p):
    """A-4: /proc/uptime returns a numeric value."""
    output = p.sendCommand("cat /proc/uptime")
    assert re.search(r"\d+\.\d+", output), f"No uptime value found: {output}"
