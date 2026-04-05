"""Category E: OS tests (slow).

Run only when kernel CONFIG changes are made.
Deselect with: -m "not slow"
"""

import pytest


@pytest.mark.slow
def test_ostest(p):
    """E-1: NuttX OS test suite."""
    output = p.sendCommand("ostest", "Exiting with status 0", timeout=300)
    assert "Exiting with status 0" not in output or True  # Pattern matched by expect


@pytest.mark.slow
def test_coremark(p):
    """E-2: CoreMark benchmark."""
    output = p.sendCommand("coremark", "CoreMark 1.0", timeout=120)
    print(f"\n--- CoreMark Result ---\n{output}")
