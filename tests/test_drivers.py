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


def test_imu_uorb_topic(p):
    """B-4: combined IMU uORB topic is registered.

    After the Issue #56 rework the LSM6DSL driver publishes a single
    /dev/uorb/sensor_imu0 with raw int16 accel+gyro instead of separate
    sensor_accel0 / sensor_gyro0 topics, so sensortest cannot drive it
    (no `imu` entry in NuttX's sensor_info table).  Verifying the device
    node is the closest direct check that the driver bound at boot.
    """
    output = p.sendCommand("ls /dev/uorb/sensor_imu0")
    assert "sensor_imu0" in output, (
        f"sensor_imu0 topic not registered: {output!r}"
    )
    assert "No such file" not in output


def test_imu_raw_dump(p):
    """B-5: combined IMU topic publishes samples (driver runs at 833 Hz).

    Drives the imu daemon to activate the topic and uses `imu accel raw
    200` to dump int16 samples for 200 ms.  At ODR=833 Hz we expect ~167
    samples — assert at least 50 to give plenty of margin for jitter and
    poll wake-up delay.
    """
    p.sendCommand("imu start", timeout=5)
    try:
        time.sleep(1)  # let the daemon open the topic and start polling
        output = p.sendCommand("imu accel raw 200", timeout=5)
        m = re.search(r"#\s*(\d+)\s+sample\(s\)\s+over\s+200\s+ms", output)
        assert m, f"trailer line missing: {output!r}"
        n = int(m.group(1))
        assert n >= 50, f"too few samples in 200 ms: {n}"
    finally:
        p.sendCommand("imu stop", timeout=5)


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


def test_led_smoke(p):
    """B-8: LED smoke — each LED group lights briefly and NSH returns."""
    output = p.sendCommand("led smoke", timeout=5)
    for marker in ("status", "battery", "bluetooth", "matrix", "done"):
        assert f"smoke: {marker}" in output, f"missing '{marker}' line: {output}"


@pytest.mark.interactive
def test_led_all(p):
    """B-9: LED all-test (requires visual confirmation)."""
    output = p.sendCommand("led all", timeout=120)
    assert "All tests done" in output
    p.waitUser("Confirm: Did the LEDs light up correctly?")


@pytest.mark.interactive
def test_legosensor_motor_coast_brake_works(p):
    """B-10: `sensor motor_l coast/brake` reach the H-bridge driver
    (Issue #77 prep commit — LEGOSENSOR_MOTOR_*_{COAST,BRAKE} ioctl).

    Prerequisites: SPIKE Medium Motor on port B (or D / F — any odd
    port so it binds to /dev/uorb/sensor_motor_l).  The test only
    verifies that the kernel side accepts the command and that
    `legoport pwm <port> status` reflects the requested H-bridge state;
    the operator must visually confirm the motor reaction.
    """

    p.waitUser("Plug a SPIKE Medium Motor into port B and press Enter")

    # Wait for LUMP to SYNC the device, then make sure motor_l bound.
    time.sleep(2.0)
    info = p.sendCommand("sensor motor_l info", timeout=5)
    assert "type=48" in info or "type_id=48" in info or "TYPE 48" in info, (
        f"motor_l did not bind to a SPIKE Medium Motor: {info!r}"
    )

    # 1) BRAKE — drives both H-bridge low-side FETs ON.
    out = p.sendCommand("sensor motor_l brake")
    assert "BRAKE:" not in out and "errno" not in out.lower(), (
        f"sensor motor_l brake reported error: {out!r}"
    )
    status = p.sendCommand("legoport pwm B status")
    assert "BRAKE" in status, f"port B not in BRAKE state: {status!r}"

    # 2) COAST — opens the H-bridge.
    out = p.sendCommand("sensor motor_l coast")
    assert "COAST:" not in out and "errno" not in out.lower(), (
        f"sensor motor_l coast reported error: {out!r}"
    )
    status = p.sendCommand("legoport pwm B status")
    assert "COAST" in status, f"port B not in COAST state: {status!r}"

    # 3) Cross-class rejection — `sensor color coast` must report -ENOTTY.
    out = p.sendCommand("sensor color coast")
    assert "Inappropriate" in out or "ENOTTY" in out or "not supported" in out.lower(), (
        f"sensor color coast should reject with -ENOTTY, got: {out!r}"
    )
