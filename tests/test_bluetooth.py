"""Category H: CC2564C Bluetooth tests (Issue #47).

These tests verified bring-up on top of the NuttX standard Bluetooth host
stack (bnep0 + btsak) that has been removed in favour of btstack (Issue
#52).  They are kept as module-level skips so git history stays legible
until the new btstack-based test suite replaces them in Step F.
"""

import pytest

pytestmark = pytest.mark.skip(
    reason=(
        "NuttX standard BT stack removed in #52 Step A; replaced by btstack "
        "SPP tests in tests/test_bt_spp.py (Step F)."
    )
)


def test_bt_bringup_dmesg(p):
    """Superseded — see tests/test_bt_spp.py."""


def test_bt_netdev_visible(p):
    """Superseded — see tests/test_bt_spp.py."""


def test_bt_info_bdaddr(p):
    """Superseded — see tests/test_bt_spp.py."""


def test_bt_info_buffer_pool(p):
    """Superseded — see tests/test_bt_spp.py."""


def test_bt_scan(p):
    """Superseded — see tests/test_bt_spp.py."""
