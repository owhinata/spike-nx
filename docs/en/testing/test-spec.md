# Test Specification

## Overview

Automated test environment for the SPIKE Prime Hub NuttX project. Uses pexpect + pyserial + pytest to execute NSH commands over serial and verify their output.

### Prerequisites

- SPIKE Prime Hub connected via USB
- NuttX (usbnsh) flashed
- Python virtual environment (`.venv/`) available

### Installing Test Dependencies

```bash
.venv/bin/pip install -r requirements.txt
```

## Running Tests

The serial device can be specified via the `-D` option or the `NUTTX_DEVICE` environment variable.
Default is `/dev/tty.usbmodem01`.

```bash
# Set via environment variable (recommended: add to .zshrc)
export NUTTX_DEVICE=/dev/tty.usbmodem01

# Automated tests only (recommended for regular use)
.venv/bin/pytest tests/ -m "not slow and not interactive"

# Automated + interactive (visual/manual operation)
.venv/bin/pytest tests/ -m "not slow"

# All tests including slow (after kernel CONFIG changes)
.venv/bin/pytest tests/

# Specific category
.venv/bin/pytest tests/test_drivers.py

# Specific test
.venv/bin/pytest tests/test_drivers.py::test_battery_gauge
```

## Test Summary

| Category | Count | Automated | Interactive | Skipped |
|----------|-------|-----------|-------------|---------|
| A. Boot & Init | 4 | 4 | 0 | 0 |
| B. Peripherals | 9 | 8 | 1 | 0 |
| C. System | 6 | 4 | 2 | 0 |
| D. Crash Handling | 4 | 4 | 0 | 0 |
| E. OS Tests | 2 | 1 | 0 | 1 ([#26](https://github.com/owhinata/spike-nx/issues/26)) |
| F. Sound | 13 | 9 | 4 | 0 |
| **Total** | **38** | **31** | **7** | **1** |

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
    4. `imu status` — verify `running:` + `yes`
    5. `imu accel` — Z-axis absolute value 8000-12000 mm/s²
    6. `imu gyro` — contains `gyro:`
    7. `imu upside` — contains `up side:`
    8. Record CPU load (during fusion, ~15%)
    9. `imu stop` — stop daemon (try/finally ensures cleanup)
- **Pass criteria**: Status OK, Z-axis in range, stop succeeds

### B-7: test_i2c_scan

- **Purpose**: Detect IMU on I2C bus
- **Command**: `i2c dev -b 2 0x03 0x77`
- **Pass criteria**: Contains `6a` (LSM6DS3 address)
- **Note**: Auto-skipped if `i2c` command not available

### B-8: test_led_smoke

- **Purpose**: Smoke test that briefly lights each LED group (status / battery / bluetooth / matrix) to verify the LED driver path and NSH responsiveness
- **Command**: `led smoke`
- **Pass criteria**: Output contains `smoke: status`, `smoke: battery`, `smoke: bluetooth`, `smoke: matrix`, `smoke: done`
- **Duration**: ~0.5 s (each LED lit for 100 ms)

### B-9: test_led_all `@interactive`

- **Purpose**: LED full pattern test (RGB cycle / rainbow / breathe / matrix)
- **Command**: `led all`
- **Pass criteria**: `All tests done` + visual confirmation
- **Timeout**: 120 seconds (includes Matrix LED sequences)

## C. System Services (`test_system.py`)

### C-1: test_watchdog_device

- **Purpose**: Watchdog device exists
- **Command**: `ls /dev/watchdog0`
- **Pass criteria**: Contains `watchdog0`

### C-2: test_cpuload

- **Purpose**: CPU load info available
- **Command**: `cat /proc/cpuload`
- **Pass criteria**: Contains `%`

### C-3: test_stackmonitor

- **Purpose**: Stack monitor daemon starts and stops
- **Command**: `stackmonitor_start` / `stackmonitor_stop`
- **Pass criteria**: Command found, `/proc/0` contains `stack` entry

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

Each test triggers a crash → watchdog reset (~3 s) → NSH reconnect cycle. The `_reboot_before_crash` autouse fixture also reboots the board *before* each test so every scenario starts from a clean heap regardless of what the previous test left behind. Heap comparison is disabled with `@pytest.mark.no_memcheck`.

### D-1: test_crash_assert

- **Command**: `crash assert`
- **Pass criteria**: `up_assert` → reset → `nsh> ` recovery

### D-2: test_crash_null

- **Command**: `crash null`
- **Pass criteria**: `Hard Fault` → IWDG reset → `nsh> ` recovery
- **Note**: `CONFIG_WATCHDOG_AUTOMONITOR=y` ([#31](https://github.com/owhinata/spike-nx/issues/31)) keeps IWDG running and software-kicked, so after a hard fault disables interrupts the IWDG resets the board after ~3 s and the test recovers.

### D-3: test_crash_divzero

- **Command**: `crash divzero`
- **Pass criteria**: `Fault` → IWDG reset → `nsh> ` recovery

### D-4: test_crash_stackoverflow

- **Command**: `crash stackoverflow`
- **Pass criteria**: `assert|Fault` → IWDG reset → `nsh> ` recovery

## E. OS Tests (`test_ostest.py`)

`test_ostest` is marked `@slow` and is only run after kernel CONFIG changes (deselect with `-m "not slow"`). `test_coremark` finishes in about 12 seconds and runs in the default suite.

### E-1: test_ostest `@slow` `@skip`

- **Command**: `ostest`
- **Pass criteria**: `Exiting with status 0`
- **Timeout**: 900 seconds
- **Skip reason**: Hangs at signest_test (nested signal handler test) ([#26](https://github.com/owhinata/spike-nx/issues/26))

### E-2: test_coremark

- **Command**: `coremark`
- **Pass criteria**: Contains `CoreMark 1.0 :`
- **Timeout**: 300 seconds
- **Measured result**: 170.82 iterations/sec (STM32F413, Cortex-M4)

## F. Sound Driver (`test_sound.py`)

Smoke tests for `/dev/tone0`, `/dev/pcm0`, and the `apps/sound` NSH builtin. The automated tests keep audible output to a minimum (one short tone + one short beep); full audible verification lives in the interactive tests.

### F-1: test_sound_devices_present

- **Purpose**: `/dev/tone0` and `/dev/pcm0` are registered at boot
- **Command**: `ls /dev`
- **Pass criteria**: Contains `tone0` and `pcm0`

### F-2: test_sound_dmesg_banner

- **Purpose**: bringup syslog contains three sound init lines
- **Command**: `dmesg`
- **Pass criteria**: Contains `sound: initialized`, `tone: /dev/tone0 registered`, `pcm: /dev/pcm0 registered`

### F-3: test_sound_usage

- **Purpose**: `sound` with no args prints usage
- **Command**: `sound`
- **Pass criteria**: Contains `Usage`, `beep`, `notes`

### F-4: test_tone_single_note

- **Purpose**: Write one quarter note to `/dev/tone0` and verify NSH returns within the expected window (tone-path smoke)
- **Command**: `echo "C4/4" > /dev/tone0`
- **Pass criteria**: Elapsed 400-900 ms (120 BPM quarter = 500 ms + release gap)

### F-5: test_sound_beep_default

- **Purpose**: Default `sound beep` (500 Hz / 200 ms) returns within the expected window (PCM-path smoke)
- **Command**: `sound beep`
- **Pass criteria**: Elapsed 150-800 ms

### F-6: test_sound_volume_roundtrip

- **Purpose**: Volume SET/GET round-trips through `TONEIOC_VOLUME_*` (silent)
- **Command**: `sound volume` / `sound volume 30` / `sound volume 75` / restore
- **Pass criteria**: Each SET value is reflected by the next GET

### F-7: test_sound_off

- **Purpose**: `sound off` returns cleanly even with nothing playing
- **Command**: `sound off`
- **Pass criteria**: Output does not contain `failed`

### F-8: test_pcm_short_write_rejected

- **Purpose**: Writing fewer bytes than the PCM header returns `-EINVAL` without panicking (survivor check)
- **Command**: `echo "abcd" > /dev/pcm0` → `echo alive`
- **Pass criteria**: Follow-up command returns `alive`

### F-9: test_pcm_bad_magic_rejected

- **Purpose**: A 20-byte garbage header is rejected without panic
- **Command**: `echo 'XXXXXXXXXXXXXXXXXXXX' > /dev/pcm0` → `echo alive`
- **Pass criteria**: Follow-up command returns `alive`

### F-I1: test_sound_audible_tone_dev `@interactive`

- **Purpose**: Audible verification for `/dev/tone0`
- **Command**: `echo "C4/4 E4/4 G4/4 C5/2" > /dev/tone0`
- **Pass criteria**: Operator hears an ascending C/E/G/C arpeggio

### F-I2: test_sound_audible_pcm_dev `@interactive`

- **Purpose**: Audible verification for `/dev/pcm0` via `sound beep`
- **Command**: `sound beep 440 400` → `sound beep 880 400`
- **Pass criteria**: Operator hears 440 Hz then 880 Hz

### F-I3: test_sound_audible_notes_app `@interactive`

- **Purpose**: Audible verification for the `apps/sound` `sound notes` path
- **Command**: `sound notes "T240 C4/4 E4/4 G4/4 C5/4 G4/4 E4/4 C4/2"`
- **Pass criteria**: Operator hears an up-and-down C arpeggio

### F-I4: test_sound_audible_volume `@interactive`

- **Purpose**: Volume changes actually affect loudness
- **Command**: volume 100 → beep → volume 20 → beep → restore
- **Pass criteria**: The second beep is noticeably quieter than the first

## Test Synchronization (sendCommand)

`NuttxSerial.sendCommand()` in `tests/conftest.py` synchronizes serial output using unique per-call sentinels.

1. Phase 1 — PRE marker: send `echo MKPRE<nonce>` and expect `<pre>\r\nnsh> ` to establish a clean baseline. This absorbs any stale buffer contents.
2. Phase 2 — real command: after `sendline(cmd)`, expect the line-anchored prompt `\r\nnsh> `. Anchoring on `\r\n` (not a bare `nsh> ` substring) prevents false matches inside command output.

This approach resolves the intermittent echo-sync failures of the previous "first word echo" strategy (issue [#33](https://github.com/owhinata/spike-nx/issues/33)). Only one `sendline` is issued per phase, so NSH never has to process queued input behind a slow/heavy command.

## Memory Leak Detection

The `free` command is executed before and after each test to compare heap memory. A warning is emitted if free memory decreases by more than 1KB after a test.

Individual tests can opt out of the free measurement with the `@pytest.mark.no_memcheck` marker (the `check_memory_leak` fixture in `conftest.py` skips the measurement when this marker is present).

!!! note
    Tests involving resets (e.g., `test_power_off`) may trigger memory warnings due to differences in initial heap state before and after the reset. This is not a memory leak.
