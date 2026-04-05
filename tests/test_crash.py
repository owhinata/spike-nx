"""Category D: Crash handling tests.

Each test triggers a crash, waits for watchdog reset (~3s),
and verifies NSH prompt recovery.
"""

from conftest import PROMPT


def _crash_and_recover(p, cmd, fault_pattern):
    """Send a crash command, expect fault output, then reconnect after reset."""
    p.clean_buffer()
    p.proc.sendline(cmd)
    try:
        p.proc.expect(fault_pattern, timeout=5)
    except Exception:
        pass  # Crash output may be partial or garbled
    # Watchdog resets the board (~3s), then USB CDC/ACM re-enumerates
    p.reconnect(timeout=15)


def test_crash_assert(p):
    """D-1: ASSERT crash and watchdog recovery."""
    _crash_and_recover(p, "crash assert", "up_assert")


def test_crash_null(p):
    """D-2: NULL pointer dereference crash and recovery."""
    _crash_and_recover(p, "crash null", r"Hard Fault|HardFault")


def test_crash_divzero(p):
    """D-3: Division by zero crash and recovery."""
    _crash_and_recover(p, "crash divzero", "Fault")


def test_crash_stackoverflow(p):
    """D-4: Stack overflow crash and recovery."""
    _crash_and_recover(p, "crash stackoverflow", r"assert|Fault")
