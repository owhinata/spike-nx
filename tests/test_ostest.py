"""Category E: OS tests (slow).

Run only when kernel CONFIG changes are made.
Deselect with: -m "not slow"
"""

import pytest


@pytest.mark.slow
@pytest.mark.skip(reason="ostest hangs at signest_test (issue #26)")
def test_ostest(p):
    """E-1: NuttX OS test suite."""
    # ostest is long-running on this MCU; if expect() returns without
    # raising, the success pattern was found.
    p.sendCommand("ostest", "Exiting with status 0", timeout=900)


@pytest.mark.slow
def test_coremark(p):
    """E-2: CoreMark benchmark."""
    # Wait for the final "CoreMark 1.0 :" summary line, then collect
    # everything up to the next prompt.
    p.sendCommand("coremark", r"CoreMark 1\.0 :", timeout=300)
    output = p.sendCommand("", timeout=10)
    print(f"\n--- CoreMark Result ---{output}")
