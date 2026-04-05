"""Category B: Peripheral driver tests."""

import re
import time

import pytest


def test_battery_gauge(p):
    """B-1: battery gauge reports voltage."""
    output = p.sendCommand("battery gauge")
    assert "Voltage:" in output
    assert "mV" in output


def test_battery_charger(p):
    """B-2: battery charger reports state and health."""
    output = p.sendCommand("battery charger")
    assert "State:" in output
    assert "Health:" in output


def test_battery_monitor(p):
    """B-3: battery monitor collects 3 samples without FAULT."""
    output = p.sendCommand("battery monitor 3", timeout=15)
    assert "FAULT" not in output
    # Data lines contain voltage values (e.g. "8555      40")
    data_lines = [l for l in output.splitlines() if re.search(r"^\w+\s+\d{4,}", l)]
    assert len(data_lines) >= 3, f"Expected 3+ data lines, got {len(data_lines)}"


def test_imu_accel(p):
    """B-4: accelerometer sensor test."""
    output = p.sendCommand("sensortest -n 3 accel0", timeout=15)
    assert "number:3/3" in output


def test_imu_gyro(p):
    """B-5: gyroscope sensor test."""
    output = p.sendCommand("sensortest -n 3 gyro0", timeout=15)
    assert "number:3/3" in output


def test_imu_fusion(p):
    """B-6: IMU fusion daemon start/status/stop with CPU load measurement."""
    # 1. Record CPU load before fusion
    cpu_before = p.sendCommand("cat /proc/cpuload")

    # 2. Start fusion daemon
    p.sendCommand("imu start")

    try:
        # 3. Wait for stabilization
        time.sleep(3)

        # 4. Check status
        status = p.sendCommand("imu status")
        assert re.search(r"running:\s+yes", status), f"Unexpected status: {status}"

        # 5. Check accelerometer (Z-axis ~9807 mm/s^2 when flat)
        accel = p.sendCommand("imu accel")
        assert "accel:" in accel
        z_match = re.search(r"z=\s*(-?\d+)", accel)
        assert z_match, f"No Z-axis value found: {accel}"
        z_val = abs(int(z_match.group(1)))
        assert 8000 < z_val < 12000, f"Z-axis out of range: {z_val} mm/s^2"

        # 6. Check gyroscope (all axes ~0 when stationary)
        gyro = p.sendCommand("imu gyro")
        assert "gyro:" in gyro

        # 7. Check orientation
        upside = p.sendCommand("imu upside")
        assert "up side:" in upside

        # 8. Record CPU load during fusion
        cpu_during = p.sendCommand("cat /proc/cpuload")
        print(f"\n--- CPU Load ---\nBefore fusion:\n{cpu_before}\nDuring fusion:\n{cpu_during}")
    finally:
        # 9. Stop fusion daemon (always clean up)
        p.sendCommand("imu stop")


def test_i2c_scan(p):
    """B-7: I2C bus 2 scan detects IMU at 0x6a."""
    output = p.sendCommand("i2c dev -b 2 0x03 0x77")
    if "command not found" in output:
        pytest.skip("i2c tool not enabled in defconfig")
    assert "6a" in output


@pytest.mark.interactive
def test_led_all(p):
    """B-8: LED all-test (requires visual confirmation)."""
    output = p.sendCommand("led all", timeout=30)
    assert "All tests done" in output
    p.waitUser("Confirm: Did the LEDs light up correctly?")
