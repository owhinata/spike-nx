"""Category C: System service tests."""

import pytest


def test_watchdog_device(p):
    """C-1: Watchdog device exists."""
    output = p.sendCommand("ls /dev/watchdog0")
    assert "watchdog0" in output


def test_cpuload(p):
    """C-2: CPU load is available via procfs."""
    output = p.sendCommand("cat /proc/cpuload")
    assert "%" in output


def test_help_builtins(p):
    """C-4: Built-in apps are registered."""
    output = p.sendCommand("help")
    assert "battery" in output
    assert "led" in output
    assert "imu" in output


@pytest.mark.interactive
def test_power_off(p):
    """C-5: Power off via center button long press and recovery."""
    p.waitUser(
        "Press and hold the center button for 2 seconds.\n"
        "    Confirm: LED turns blue, then power off / reset."
    )
    # After reset, USB CDC/ACM re-enumerates
    p.reconnect(timeout=15)


@pytest.mark.interactive
def test_usb_reconnect(p):
    """C-6: USB cable unplug and replug recovery."""
    p.waitUser("Unplug the USB cable.")
    p.waitUser("Replug the USB cable.")
    p.reconnect(timeout=15)
    # Verify normal operation after reconnect
    output = p.sendCommand("help")
    assert "battery" in output
