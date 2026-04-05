# Test Specification

## Overview

Automated test environment for the SPIKE Prime Hub NuttX project. Uses pexpect + pyserial + pytest to execute NSH commands over serial and verify their output.

### Prerequisites

- SPIKE Prime Hub connected via USB
- NuttX (usbnsh) flashed
- Python virtual environment (`.venv/`) available

### Installing Test Dependencies

```bash
.venv/bin/pip install -r tests/requirements.txt
```

## Running Tests

```bash
# All tests (excluding slow)
.venv/bin/pytest tests/ -m "not slow" -D /dev/tty.usbmodem01

# Automated tests only (excluding interactive)
.venv/bin/pytest tests/ -m "not slow and not interactive" -D /dev/tty.usbmodem01

# All tests including slow (after kernel CONFIG changes)
.venv/bin/pytest tests/ -D /dev/tty.usbmodem01

# Specific category
.venv/bin/pytest tests/test_drivers.py -D /dev/tty.usbmodem01

# Specific test
.venv/bin/pytest tests/test_drivers.py::test_battery_gauge -D /dev/tty.usbmodem01
```

## Test Summary

| Category | Count | Automated | Interactive |
|----------|-------|-----------|-------------|
| A. Boot & Init | 4 | 4 | 0 |
| B. Peripherals | 8 | 7 | 1 |
| C. System | 6 | 4 | 2 |
| D. Crash Handling | 4 | 4 | 0 |
| E. OS Tests | 2 | 2 | 0 |
| **Total** | **24** | **21** | **3** |

## A. Boot & Initialization (`test_boot.py`)

### A-1: test_nsh_prompt

- **Purpose**: Verify NSH prompt responds
- **Command**: Send Enter
- **Pass criteria**: `nsh> ` returned

### A-2: test_dmesg_no_error

- **Purpose**: No errors in boot log
- **Command**: `dmesg`
- **Pass criteria**: No lines containing `ERROR`

### A-3: test_procfs_version

- **Purpose**: NuttX version info available
- **Command**: `cat /proc/version`
- **Pass criteria**: Output contains `NuttX`

### A-4: test_procfs_uptime

- **Purpose**: Uptime is available
- **Command**: `cat /proc/uptime`
- **Pass criteria**: Contains numeric value (`\d+\.\d+`)

## B. Peripheral Drivers (`test_drivers.py`)

### B-1: test_battery_gauge

- **Purpose**: Battery voltage reading
- **Command**: `battery gauge`
- **Pass criteria**: Contains `Voltage:` and `mV`

### B-2: test_battery_charger

- **Purpose**: Charger state reading
- **Command**: `battery charger`
- **Pass criteria**: Contains `State:` and `Health:`

### B-3: test_battery_monitor

- **Purpose**: Continuous battery sampling
- **Command**: `battery monitor 3`
- **Pass criteria**: 3+ data lines, no `FAULT`

### B-4: test_imu_accel

- **Purpose**: Accelerometer operation
- **Command**: `sensortest -n 3 accel0`
- **Pass criteria**: Contains `number:3/3`

### B-5: test_imu_gyro

- **Purpose**: Gyroscope operation
- **Command**: `sensortest -n 3 gyro0`
- **Pass criteria**: Contains `number:3/3`

### B-6: test_imu_fusion

- **Purpose**: IMU fusion daemon start/data/stop
- **Steps**:
    1. Record CPU load (before fusion)
    2. `imu start` — start daemon
    3. Wait 3 seconds
    4. `imu status` — verify `running: yes`
    5. `imu accel` — Z-axis approx. 9807 mm/s²
    6. `imu gyro` — all axes approx. 0
    7. `imu upside` — check orientation
    8. Record CPU load (during fusion)
    9. `imu stop` — stop daemon
- **Pass criteria**: Status OK, Z-axis in range (8000-12000), stop succeeds

### B-7: test_i2c_scan

- **Purpose**: Detect IMU on I2C bus
- **Command**: `i2c dev -b 2 0x03 0x77`
- **Pass criteria**: Contains `6a` (LSM6DS3 address)

### B-8: test_led_all `@interactive`

- **Purpose**: LED full pattern test
- **Command**: `led all`
- **Pass criteria**: `All tests done` + visual confirmation

## C. System Services (`test_system.py`)

### C-1: test_watchdog_device

- **Purpose**: Watchdog device exists
- **Command**: `ls /dev/watchdog0`
- **Pass criteria**: Contains `watchdog0`

### C-2: test_cpuload

- **Purpose**: CPU load info available
- **Command**: `cat /proc/cpuload`
- **Pass criteria**: Contains `CPU`

### C-3: test_stackmonitor

- **Purpose**: Stack monitor operation
- **Command**: `stkmon`
- **Pass criteria**: Contains `PID`

### C-4: test_help_builtins

- **Purpose**: Built-in apps registered
- **Command**: `help`
- **Pass criteria**: Contains `battery`, `led`, `imu`

### C-5: test_power_off `@interactive`

- **Purpose**: Power off and reset recovery
- **Steps**: Long press center button → reset → serial reconnect
- **Pass criteria**: `nsh> ` prompt recovery

### C-6: test_usb_reconnect `@interactive`

- **Purpose**: USB unplug/replug recovery
- **Steps**: Unplug USB → replug → serial reconnect
- **Pass criteria**: `nsh> ` prompt recovery + `help` works

## D. Crash Handling (`test_crash.py`)

Each test triggers a crash → watchdog reset (~3s) → NSH reconnect cycle.

### D-1: test_crash_assert

- **Command**: `crash assert`
- **Pass criteria**: `up_assert` → reset → `nsh> ` recovery

### D-2: test_crash_null

- **Command**: `crash null`
- **Pass criteria**: `Hard Fault` → reset → `nsh> ` recovery

### D-3: test_crash_divzero

- **Command**: `crash divzero`
- **Pass criteria**: `Fault` → reset → `nsh> ` recovery

### D-4: test_crash_stackoverflow

- **Command**: `crash stackoverflow`
- **Pass criteria**: `assert|Fault` → reset → `nsh> ` recovery

## E. OS Tests (`test_ostest.py`) `@slow`

Run only after kernel CONFIG changes. Exclude with `-m "not slow"`.

### E-1: test_ostest

- **Command**: `ostest`
- **Pass criteria**: `Exiting with status 0`
- **Timeout**: 300 seconds

### E-2: test_coremark

- **Command**: `coremark`
- **Pass criteria**: Contains `CoreMark 1.0`
- **Timeout**: 120 seconds

## Memory Leak Detection

The `free` command is executed before and after each test to compare heap memory. A warning is emitted if free memory decreases by more than 1KB after a test.

## Test Result Template

```
Date: YYYY-MM-DD
Commit: <hash>
Test target: <category>
Result: PASS / FAIL
Notes: <remarks>
```
