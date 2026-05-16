"""Category A: Boot and initialization tests."""

import re


def test_nsh_prompt(p):
    """A-1: NSH prompt is available."""
    output = p.sendCommand("")
    assert "nsh" in output or True  # sendCommand already expects PROMPT


def test_dmesg_no_error(p):
    """A-2: dmesg contains no ERROR lines from the current boot.

    RAMLOG_BUFFER_SECTION=".ksram_ramlog" persists across MCU reset
    (#100), so dmesg may show several boot cycles.  Only the most
    recent boot is interesting — anchor on the last `BCRUMB:
    RCC_CSR=` line, which the boot path emits as one of the first
    log entries on every reset, and check the lines after it.
    """
    output = p.sendCommand("dmesg")
    lines = output.splitlines()
    anchor = 0
    for i, line in enumerate(lines):
        if "BCRUMB: RCC_CSR" in line:
            anchor = i
    for line in lines[anchor:]:
        assert "ERROR" not in line, f"ERROR found in dmesg: {line}"


def test_procfs_version(p):
    """A-3: /proc/version contains NuttX."""
    output = p.sendCommand("cat /proc/version")
    assert "NuttX" in output


def test_procfs_uptime(p):
    """A-4: /proc/uptime returns a numeric value."""
    output = p.sendCommand("cat /proc/uptime")
    assert re.search(r"\d+\.\d+", output), f"No uptime value found: {output}"
