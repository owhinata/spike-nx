"""Category E: OS tests (slow).

Run only when kernel CONFIG changes are made.
Deselect with: -m "not slow"
"""

import pytest


@pytest.mark.slow
@pytest.mark.skip(reason="ostest hangs at signest_test (issue #26)")
def test_ostest(p):
    """E-1: NuttX OS test suite."""
    output = p.sendCommand("ostest", timeout=900)
    assert "Exiting with status 0" in output


def test_coremark(p):
    """E-2: CoreMark benchmark."""
    output = p.sendCommand("coremark", timeout=300)
    assert "CoreMark 1.0 :" in output, "coremark summary line missing"
    for line in output.splitlines():
        if "CoreMark 1.0" in line:
            print(f"\n--- CoreMark Result ---\n{line}")
            break
