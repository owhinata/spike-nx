"""LEGO Powered Up I/O port DCM tests (Issue #42).

These exercise /dev/legoport0..5 and the `port` CLI without
requiring a physical Powered Up device plugged in.  Hardware-coupled
checks (passive device detection, UART device latching) are marked
``interactive`` so they only run when the operator opts in.
"""

import re
import time

import pytest


@pytest.fixture
def legoport_idle(p):
    """Ensure all 6 ports report NONE before the test starts.

    Tests that depend on a clean idle baseline (e.g. cadence checks) use
    this fixture; the operator running them must keep the I/O ports
    unplugged.
    """
    output = p.sendCommand("port list")
    for port_letter in "ABCDEF":
        # Lines look like "  A   NONE             [] #0"
        line = next(
            (line for line in output.splitlines()
             if line.lstrip().startswith(port_letter + " ")),
            None,
        )
        assert line is not None, f"port {port_letter} missing from list:\n{output}"
        assert "NONE" in line, (
            f"port {port_letter} not idle (operator must unplug all ports):\n"
            f"  {line.strip()}"
        )
    return p


def test_legoport_devices_present(p):
    """All six device nodes are registered."""
    output = p.sendCommand("ls /dev")
    for n in range(6):
        node = f"legoport{n}"
        assert node in output, f"{node} not present:\n{output}"


def test_legoport_list_smoke(p):
    """`port list` prints a header plus six rows."""
    output = p.sendCommand("port list")
    assert "Port" in output and "Type" in output and "Flags" in output
    rows = [
        line for line in output.splitlines()
        if re.match(r"^\s+[A-F]\s+\S+", line)
    ]
    assert len(rows) == 6, f"expected 6 port rows, got {len(rows)}:\n{output}"


def test_legoport_no_device_returns_none(legoport_idle):
    """With no devices plugged in, every port reports NONE.

    Already enforced by the ``legoport_idle`` fixture; this test exists
    so the assertion appears as a named regression.
    """
    assert True


def test_legoport_step_time_under_1ms(legoport_idle):
    """Worker execution time stays under 1 ms after a soak.

    Catches "blocking call inadvertently added to the DCM worker"
    regressions.  Threshold is generous (1000 µs) — typical idle
    measurement should be well under 200 µs.
    """
    p = legoport_idle
    # Soak so transients drain and the LED app / battery / IMU work
    # queues all hit steady state.
    time.sleep(5)

    output = p.sendCommand("port stats")
    m = re.search(r"max step:\s+(\d+)\s+us", output)
    assert m, f"max step not reported:\n{output}"
    max_step_us = int(m.group(1))
    assert max_step_us < 1000, (
        f"max_step_us={max_step_us} exceeds 1000 us threshold; "
        "DCM worker may have a blocking call or excessive register churn."
    )


def test_legoport_interval_under_4ms(legoport_idle):
    """HPWORK rescheduling interval stays under 4 ms (target 2 ms + slack).

    Catches "TLC5955 / battery / DRDY worker is monopolising HPWORK"
    regressions.  4 ms = one full tick of slack on top of the 2 ms
    target cadence.
    """
    p = legoport_idle
    time.sleep(5)
    output = p.sendCommand("port stats")
    m = re.search(r"max interval:\s+(\d+)\s+us", output)
    assert m, f"max interval not reported:\n{output}"
    max_interval_us = int(m.group(1))
    assert max_interval_us < 4000, (
        f"max_interval_us={max_interval_us} exceeds 4000 us threshold; "
        "HPWORK contention with TLC5955 / battery / IMU workers."
    )


@pytest.mark.interactive
def test_legoport_wait_connect(p):
    """Operator-driven: plug a passive device into port A and confirm.

    Run with `pytest -m interactive`.  When prompted, plug a passive
    Powered Up device (Train motor, Tap point, Touch sensor, etc.)
    into port A within 30 seconds.  SPIKE motors (which are UART
    devices) latch as UNKNOWN_UART in #42 — that also satisfies this
    test.
    """
    print("\n>>> plug ANY Powered Up device into port A within 30 seconds <<<")
    output = p.sendCommand("port wait 0 30000", timeout=35)
    assert "connected:" in output, f"no connect detected:\n{output}"
    assert "NONE" not in output.splitlines()[-1]
