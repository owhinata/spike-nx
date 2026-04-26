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
| G. Flash / LittleFS | 7 | 4 | 3 | 0 |
| H. Bluetooth (CC2564C) | 5 | 4 | 1 | 0 |
| **Total** | **50** | **39** | **11** | **1** |

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

### B-4: test_imu_uorb_topic

- **Purpose**: Combined IMU uORB topic is registered (the LSM6DSL
  driver was reworked under Issue #56 to publish a single
  `sensor_imu0` topic with raw int16 accel + gyro instead of separate
  `sensor_accel0` / `sensor_gyro0` topics)
- **Command**: `ls /dev/uorb/sensor_imu0`
- **Pass criteria**: Output contains `sensor_imu0`, no `No such file`
- **Notes**: NuttX `sensortest` has no `imu` entry in its sensor table
  (`apps/system/sensortest/sensortest.c`), so the device-node check is
  the closest direct verification that the driver bound at boot

### B-5: test_imu_raw_dump

- **Purpose**: Combined IMU topic publishes samples at the configured
  ODR (833 Hz default after Issue #56)
- **Command**: `imu start` → `imu accel raw 200` → `imu stop`
- **Pass criteria**: Trailer line `# N sample(s) over 200 ms` with
  N ≥ 50 (≈167 samples expected at ODR 833 Hz, with margin for poll
  jitter)

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

## G. Flash / LittleFS (`test_flash.py`)

W25Q256 + LittleFS regression tests plus interactive Zmodem transfer tests via picocom + lrzsz.  See [W25Q256 driver](../drivers/w25q256.md) and [Zmodem file transfer](../development/file-transfer.md).

### G-1: test_flash_mount

- **Purpose**: `/mnt/flash` is mounted as LittleFS at boot
- **Command**: `mount`
- **Pass**: Output contains `/mnt/flash` and `littlefs`

### G-2: test_flash_mtdblock_visible

- **Purpose**: Only `/dev/mtdblock0` is exposed; full-chip raw `/dev/mtd0` is hidden (LEGO area protection)
- **Command**: `ls /dev/mtdblock0` / `ls /dev/mtd0`
- **Pass**: First returns the name, second returns ENOENT

### G-3: test_flash_write_read

- **Purpose**: File write / read round-trip
- **Command**: `echo "regression-<ts>" > /mnt/flash/regress.txt` → `cat /mnt/flash/regress.txt`
- **Pass**: `cat` returns the written payload
- **Timeout**: 120 s (LittleFS first metadata update can take several seconds)

### G-4: test_flash_persist_across_reboot

- **Purpose**: Files survive `reboot` (LittleFS durability)
- **Command**: `echo "persist-<ts>" > /mnt/flash/persist.txt` → `reboot` → `cat /mnt/flash/persist.txt`
- **Pass**: Contents match after reboot

### G-I1: test_flash_zmodem_send `@interactive`

- **Purpose**: PC → Hub Zmodem upload + on-device md5 verification
- **Steps**:
    1. pytest generates a fresh random 1 MB file at `/tmp/spike_zmtest_1mb.bin` (regenerated every run)
    2. Releases the serial port and prompts the operator to run `picocom --send-cmd 'sz -vv -L 256' /dev/tty.usbmodem01`
    3. Operator runs `nsh> rz` in picocom, presses `C-a C-s`, enters the file path
    4. Operator exits picocom and presses Enter; pytest reconnects the serial port
    5. pytest verifies the file size (1048576 bytes) and that Hub-side `md5 -f /mnt/flash/spike_zmtest_1mb.bin` matches the PC-side md5
- **Requires**: lrzsz (`brew install lrzsz`)

### G-I2: test_flash_zmodem_recv `@interactive`

- **Purpose**: Hub → PC Zmodem download + PC-side md5 round-trip verification
- **Depends on**: `test_flash_zmodem_send` having run in the same session (target file must exist on Hub)
- **Steps**:
    1. pytest gets the ground-truth md5 from the Hub via `md5 -f`
    2. Removes any stale local file with the same name and releases the serial port
    3. Prompts the operator to run `cd <pytest cwd> && picocom --receive-cmd 'rz -vv -y' /dev/tty.usbmodem01`
    4. Operator runs `nsh> sz /mnt/flash/spike_zmtest_1mb.bin` in picocom, presses `C-a C-r`, then Enter with no arguments
    5. Operator exits picocom and presses Enter; pytest reconnects the serial port
    6. pytest md5sums the locally-saved file and asserts it matches the Hub-side md5
    7. On success, removes both the local copy and the Hub copy
- **Requires**: lrzsz (`brew install lrzsz`)

### G-S1: test_flash_stress_concurrent `@interactive`

- **Purpose**: Run Sound DMA1_S5 + IMU I2C2 daemon + Flash SPI2/DMA1_S3/S4 + USB CDC concurrently to confirm the W25Q256 driver does not disrupt other peripherals (Codex-required gate).
- **Steps**:
    1. `imu start` to bring up the IMU daemon (continuous I2C2 reads + EXTI4 + HPWORK load).
    2. Capture `dmesg` as a baseline.
    3. `sound volume 15` to a quiet level, then `sound beep 440 4000 &` to play a 4-second 440 Hz tone in the background.
    4. `dd if=/dev/zero of=/mnt/flash/stress.bin bs=1024 count=64` writes 64 KB (16 × 4 KB sector erases + 256 page programs ≈ 3-5 s, fully overlapping the tone).
    5. `dd if=/mnt/flash/stress.bin of=/dev/null bs=1024 count=64` reads it back.
    6. Sleep 2 s so any tail-end PCM DMA completes.
    7. `ls -l /mnt/flash/stress.bin` confirms the size is 65536 bytes.
    8. `imu status` confirms the IMU daemon is still alive (`running: yes`).
    9. Assert the new dmesg lines do not contain `ERROR / FAULT / underrun / overflow / ASSERT / panic`.
    10. Operator confirms the tone had no clicks, dropouts, or pitch wobble (the DAC driver does not log underruns, so this is the only audible check).
    11. Cleanup: `imu stop`, `rm /mnt/flash/stress.bin`, restore the original volume.

### Example Commands

```bash
# Regression tests only (excluding interactive)
.venv/bin/pytest tests/test_flash.py -m "not interactive" -D /dev/tty.usbmodem01

# Send only
.venv/bin/pytest tests/test_flash.py::test_flash_zmodem_send -D /dev/tty.usbmodem01

# Stress test only
.venv/bin/pytest tests/test_flash.py::test_flash_stress_concurrent -D /dev/tty.usbmodem01

# All tests (including interactive)
.venv/bin/pytest tests/test_flash.py -D /dev/tty.usbmodem01
```

## H. Bluetooth (`test_bt_spp.py`)

Issue #52 replaced the NuttX stock BT host stack with btstack + Classic BT SPP, so the test suite was rewritten accordingly.  The automated tests cover the Hub-side readiness (chardev, bring-up, btsensor builtin, HCI_STATE_WORKING); the PC-side pair + RFCOMM open step is kept manual under `docs/development/pc-receive-spp.md` because it needs a Linux or macOS host.

### H-1: test_bt_chardev_exists

- **Goal**: kernel-side chardev is registered
- **Command**: `ls /dev/ttyBT`
- **Check**: output contains `/dev/ttyBT` and not `No such file`

### H-2: test_bt_powered_banner

- **Goal**: CC2564C is powered and the slow clock is running
- **Command**: `dmesg`
- **Check**: output contains `BT: CC2564C powered, /dev/ttyBT ready`

### H-3: test_bt_btsensor_builtin

- **Goal**: the btsensor NSH builtin is registered
- **Command**: `help` (substring match — `grep` is not enabled in the
  usbnsh defconfig)
- **Check**: output lists `btsensor`

### H-4: test_bt_btsensor_hci_working

- **Goal**: `btsensor start` drives btstack into HCI_STATE_WORKING and
  logs a plausible BD address
- **Procedure**:
    1. `reboot` — empirically the first `btsensor start` after a fresh
       boot consistently reaches WORKING in ~1 s; restarts after the
       rest of the test session can fall into a stuck state where the
       action queue returns `timed out`
    2. drain RAMLOG once with `dmesg`
    3. `btsensor start` → expect `started (pid N)`
    4. poll `dmesg` (0.5 s interval, 6 s deadline) for
       `btsensor: HCI working, BD_ADDR <XX:XX:XX:XX:XX:XX>`
- **Pass criteria**: banner is found and BD address is neither
  all-zeros nor all-ones (uppercase hex per `bd_addr_to_str()`)
- **Cleanup**: `btsensor stop` so the daemon does not leak into
  subsequent tests
- **Notes**: banners go through `syslog()` → RAMLOG
  (`CONFIG_RAMLOG_SYSLOG=y` without `SYSLOG_CONSOLE`), so they are
  read via `dmesg`, not from the live serial console

### H-5: test_bt_pc_pair_and_stream `@interactive`

Semi-manual: the operator handles the parts that cannot be driven from
the Hub (pair the device, open RFCOMM); pytest then verifies the
stream automatically by reading `btsensor status` from the Hub.

- **Goal**: end-to-end SPP pair + RFCOMM stream is alive after the PC
  finishes pairing
- **Prerequisite**: H-4 passes; a Linux or macOS host with
  blueutil / bluetoothctl is available
- **Hub-side setup** (driven by the test): `reboot` →
  `btsensor start` → `btsensor bt on` (advertising) → `btsensor imu on`
  (samples flowing).  Issue #56 made BT/IMU both off-by-default, so
  the test must explicitly enable advertising and IMU sampling before
  prompting the operator.
- **Operator step**: pair (and trust) the Hub once via
  `bluetoothctl` so BlueZ has a link key — see
  `docs/development/pc-receive-spp.md`.  No `rfcomm bind` /
  `cat /dev/rfcomm0` step: pytest opens `BTPROTO_RFCOMM` directly.
  Press ENTER once pairing is in place (skip pairing if it has
  already been done — link keys persist).
- **Pytest connect**: `socket.socket(AF_BLUETOOTH, SOCK_STREAM,
  BTPROTO_RFCOMM)` → `connect((bdaddr, 1))`.  Skipped automatically
  if the socket cannot be created (Linux + CAP_NET_RAW required;
  rerun pytest with sudo).
- **Pytest checks** (after socket connect):
    1. Poll `btsensor status` for up to 5 s for `bt: paired` and
       `rfcomm cid != 0` (absorbs the gap between socket connect and
       the channel-open event reaching the Hub)
    2. PC-side: receive ≥ 1 byte over 3 s and find frame magic
       `0xB66B` (Issue #56 Commit E; on the wire it is little-endian
       `6b b6`); at least 50 magic markers must appear
    3. Hub-side: `frames: sent` advances by ≥ 100 over the same
       window (~312 expected at ODR 833 Hz / batch 8 ≈ 104 fps)
    4. `frames: dropped` does not exceed 25 % of the sent delta
- **Cleanup**: the test issues `btsensor imu off`, `btsensor bt off`,
  `btsensor stop` once the checks complete

### H-6: test_bt_button_short_press `@interactive`

- **Goal**: a physical short press flips the BT state machine from
  off to advertising
- **Setup** (driven by the test): `reboot` → `btsensor start`,
  status confirmed at `bt: off`
- **Operator step**: short press the BT button (< 1 s), press ENTER
- **Checks**: `btsensor status` shows `bt: advertising` within 3 s and
  `dmesg` contains `btsensor_button: short press` followed by
  `btsensor: BT advertising (was off)`

### H-7: test_bt_button_long_press `@interactive`

- **Goal**: a physical long press flips the BT state machine from
  advertising to off
- **Setup** (driven by the test): `reboot` → `btsensor start` →
  `btsensor bt on` (advertising via NSH so the operator only does
  one press)
- **Operator step**: long press the BT button (≥ 1 s), press ENTER
- **Checks**: `btsensor status` shows `bt: off` within 3 s and
  `dmesg` contains `btsensor_button: long press` followed by
  `btsensor: BT off (was advertising)`

### H-8: test_bt_rfcomm_command_suite `@interactive`

Semi-manual: operator pairs once, then pytest exercises every
PC→Hub ASCII command (Issue #56 Commit D) and verifies that the
parameter changes propagate into the live IMU stream header.

- **Goal**: each ASCII command is parsed correctly, replies travel
  back over RFCOMM, and the next IMU frame carries the new ODR /
  batch / FSR fields
- **Prerequisite**: H-5 passes; a Linux host with bluetoothctl and
  CAP_NET_RAW for the test process (rerun with sudo if required;
  the test skips automatically if AF_BLUETOOTH is unavailable)
- **Hub-side setup** (driven by the test): `reboot` →
  `btsensor start` → `btsensor bt on`
- **Operator step**: pair (+ trust) the Hub once via `bluetoothctl`,
  press ENTER (skip pairing if a link key is already in place)
- **Pytest connect**: `BTPROTO_RFCOMM` socket to `(bdaddr, 1)`,
  poll `btsensor status` until `bt: paired`
- **Phase 1 — SET while IMU off (commands succeed)**:
  `SET ODR 416`, `SET BATCH 16`, `SET ACCEL_FSR 4`, `SET GYRO_FSR 1000`
  each return `OK`; `btsensor status` reflects `odr=416Hz batch=16
  accel_fsr=4g gyro_fsr=1000dps`
- **Phase 2 — invalid input (ERR replies)**: `SET BATCH 200` (out of
  1..80), `SET FOO 1`, `BAD` each return a line starting with `ERR`
- **Phase 3 — IMU ON + frame header validation**: `IMU ON` returns
  `OK`; pytest drains ~2 s of frames, finds magic `0xB66B`, and
  parses the first 18-byte header: `type=0x01`, `sample_count=16`,
  `sample_rate_hz=416`, `accel_fsr_g=4`, `gyro_fsr_dps=1000`
- **Phase 4 — SET while IMU on (busy)**: `SET ODR 833` returns a
  reply containing `busy`
- **Phase 5 — IMU OFF**: returns `OK`; status reflects `imu: off`
- **Cleanup**: `btsensor imu off`, `btsensor bt off`, `btsensor stop`

### Example invocations

```bash
# Regression only (exclude interactive)
.venv/bin/pytest tests/test_bt_spp.py -m "not interactive" -D /dev/tty.usbmodem01

# Manual button + RFCOMM tests (sudo for AF_BLUETOOTH socket on H-8)
sudo -E .venv/bin/pytest tests/test_bt_spp.py::test_bt_button_short_press -D /dev/tty.usbmodem01
sudo -E .venv/bin/pytest tests/test_bt_spp.py::test_bt_rfcomm_command_suite -D /dev/tty.usbmodem01
```

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
