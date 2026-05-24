# IMU Driver Design

## 1. Overview

The onboard LSM6DS3TR-C (6-axis IMU) on the SPIKE Prime Hub is driven using the NuttX LSM6DSL uORB sensor driver. The LSM6DS3TR-C has full register compatibility with the LSM6DSL, so the driver is used as-is.

The IMU processing library (`apps/imu/`) is sensor-agnostic and consumes data via uORB topics.

## 2. Device Specifications

The LSM6DS3TR-C is a single-die 6-axis IMU integrating a 3-axis accelerometer and 3-axis gyroscope.

| Item | Value |
|---|---|
| I2C Address | 0x6A (SDO/SA0 = GND) |
| WHO_AM_I | 0x6A |
| Startup ODR | 833 Hz (runtime-tunable via ioctl) |
| Startup accel FSR | ±2 g (runtime-tunable to 2/4/8/16 g via ioctl) — Phase 2.5 (#145) narrowed from ±8 g for 4× finer tilt resolution |
| Startup gyro FSR | ±1000 dps (runtime-tunable to 125/250/500/1000/2000 dps via ioctl) — Phase 2.5 (#145) narrowed from ±2000 dps for 2× finer angular resolution while keeping 1.8× margin over the drivebase worst-case 565 dps |

## 3. Board Wiring

| Pin | Function | Description |
|---|---|---|
| PB10 | I2C2_SCL | I2C clock (AF4) |
| PB3 | I2C2_SDA | I2C data (AF9, F413-specific) |
| PB4 | INT1 | Gyro DRDY interrupt (EXTI4) |

## 4. Register Configuration

| Register | Setting | Purpose |
|---|---|---|
| CTRL1_XL (0x10) | ODR=833Hz, FS=±2g | Accel: ODR and full scale (Phase 2.5 default) |
| CTRL2_G (0x11) | ODR=833Hz, FS=±1000dps | Gyro: ODR and full scale (Phase 2.5 default) |
| CTRL3_C (0x12) | BDU=1, IF_INC=1 | Block data update, address auto-increment |
| CTRL5_C (0x14) | ROUNDING=011 | Rounding for burst reads |
| DRDY_PULSE_CFG (0x0B) | DRDY_PULSED=1 | Pulsed mode DRDY |
| INT1_CTRL (0x0D) | INT1_DRDY_G=1 | Route gyro DRDY to INT1 |

## 5. Data Acquisition

A single INT1 (gyro DRDY) interrupt triggers a 12-byte burst read:

- Bytes 0-5: Gyro X/Y/Z (OUTX_L_G 0x22 through OUTZ_H_G 0x27)
- Bytes 6-11: Accel X/Y/Z (OUTX_L_A 0x28 through OUTZ_H_A 0x2D)

Both sensors operate at the same ODR (833 Hz), so accel data is also updated when the gyro DRDY fires.

### Data Flow

```
INT1 (gyro DRDY) fires
  -> ISR: capture timestamp (CLOCK_BOOTTIME us, low 32 bits)
  -> HPWORK: 12-byte burst read from 0x22
  -> Every 16 samples: also read OUT_TEMP_L/H (0x20); reuse the cached
     value in between
  -> struct sensor_imu (timestamp + raw int16 accel/gyro + temperature_raw)
  -> push_event delivers to /dev/uorb/sensor_imu0
```

The driver does not perform physical-unit conversion; consumers convert
the raw LSB values using the configured FSR.

## 6. NuttX Driver Architecture

### Board Layer

```
boards/spike-prime-hub/src/lsm6dsl_uorb.c        - I2C control, burst read, uORB registration
boards/spike-prime-hub/src/lsm6dsl_uorb.h        - config struct, registration API
boards/spike-prime-hub/src/stm32_lsm6dsl.c       - I2C2 init, INT1 interrupt setup, driver registration
boards/spike-prime-hub/include/board_lsm6dsl.h   - Board-local ioctl numbers (FSR set)
```

### uORB Topics

```
/dev/uorb/sensor_imu0   (struct sensor_imu)
```

Registered via `sensor_custom_register()` as `SENSOR_TYPE_CUSTOM`.  A
single `read()` returns one combined accel + gyro + temperature_raw +
ISR-captured timestamp record.

### struct sensor_imu

Defined in `boards/spike-prime-hub/include/board_lsm6dsl.h` (24 B fixed,
enforced by `_Static_assert` on size and offsets):

| Offset | Field | Type | Description |
|---|---|---|---|
| +0 | timestamp | uint32_t | Low 32 bits of CLOCK_BOOTTIME us (~71m35s wraparound).  ARMv7-M 4-byte aligned word load/store is single-copy atomic, so ISR -> worker handoff is tearing-free. |
| +4 | ax / ay / az | int16_t | Accel raw LSB, Hub body frame (chip frame Y/Z negated by the driver) |
| +10 | gx / gy / gz | int16_t | Gyro raw LSB, Hub body frame (chip frame Y/Z negated by the driver) |
| +16 | temperature_raw | int16_t | OUT_TEMP raw, refreshed every 16 samples (stale in between) |
| +18 | odr_idx | uint8_t | `enum lsm6dsl_odr_e` value (0..0xA) — embeds the HW ODR active when this sample was captured |
| +19 | fsr_xl_idx | uint8_t | `enum lsm6dsl_fsr_xl_e` value (sparse: 0=2g, 1=16g, 2=4g, 3=8g) |
| +20 | fsr_gy_idx | uint8_t | `enum lsm6dsl_fsr_gy_e` value (sparse: 0=250, 1=125, 2=500, 4=1000, 6=2000 dps) |
| +21..+23 | reserved[3] | uint8_t × 3 | Padding (zeroed by `push_data()` via `memset`) |

Issue #139 added the per-sample `odr_idx` / `fsr_xl_idx` / `fsr_gy_idx`
fields so consumers (drivebase / imu daemon / btsensor BUNDLE) can
compute physical units correctly even when ODR or FSR changes mid-stream.

### ioctl

| ioctl | Argument | Behavior |
|---|---|---|
| `SNIOC_SET_INTERVAL` | uint32 (period_us) | Pick the closest ODR whose period is <= `period_us` |
| `SNIOC_SETSAMPLERATE` | uint32 (Hz: 13/26/52/104/208/416/833/1660/3330/6660) | Set ODR by frequency |
| `LSM6DSL_IOC_SETACCELFSR` | uint32 (g: 2/4/8/16) | Set accel full-scale range |
| `LSM6DSL_IOC_SETGYROFSR` | uint32 (dps: 125/250/500/1000/2000) | Set gyro full-scale range |
| `SNIOC_GETSAMPLERATE` | uint32* (out) | Read current ODR as the `enum lsm6dsl_odr_e` index |
| `LSM6DSL_IOC_GETACCELFSR` | uint32* (out) | Read current accel FSR as the `enum lsm6dsl_fsr_xl_e` index |
| `LSM6DSL_IOC_GETGYROFSR` | uint32* (out) | Read current gyro FSR as the `enum lsm6dsl_fsr_gy_e` index |

SET ioctls take physical values (Hz / g / dps) and convert them to the
driver-internal enum index.  GET ioctls return the **enum index directly**
— consumers carry their own `idx → physical value` lookup table, matching
the per-sample idx fields embedded in `struct sensor_imu`.

Issue #139 removed the historical `-EBUSY` guard that rejected SET while
the sensor was active.  The SET handlers run under the same `devlock`
that serialises `push_data()`, so the register R-M-W cannot race the
publish path.  A transient ~1-sample window exists where the chip's
internal pipeline returns a sample latched at the old register setting
but tagged with the new idx; consumers (drivebase, imu daemon) absorb
that via their on-idx-change recalibration paths.

### defconfig

```
CONFIG_STM32_I2C2=y
CONFIG_I2C=y
CONFIG_I2C_RESET=y
CONFIG_SCHED_HPWORK=y
CONFIG_SENSORS=y
CONFIG_APP_IMU=y
```

### Boot-time I2C bus recovery

After a WDOG / assert / crash soft-reset of the MCU, the LSM6DS3TR-C may be left mid-transaction with SDA stuck low, wedging the I2C2 bus. The SPIKE Prime Hub has no IMU-specific power-control GPIO (LSM6DSL VDD/VDDIO is always powered), so a hardware power-cycle is not possible. Instead, `stm32_lsm6dsl.c` performs an I2C2 bus recovery on every boot.

Sequence (one attempt):

1. Acquire the I2C2 handle via `stm32_i2cbus_initialize(2)`.
2. Call `I2C_RESET(i2c)` (NuttX `stm32_i2c_reset`) — temporarily reconfigures SCL/SDA as open-drain GPIO, toggles SCL up to 10 cycles until SDA goes high, generates START + STOP to reset the slave state machine, then re-initializes the I2C peripheral.
   - On failure (SDA never released, or clock stretch never relaxes) the bus is left deinitialized with the pins still in GPIO mode. Restore it with `stm32_i2cbus_uninitialize` followed by `stm32_i2cbus_initialize` so the next attempt starts from a clean peripheral state.
3. Call `lsm6dsl_register_uorb()`.
   - `lsm6dsl_hw_init()` does a pre-flight WHO_AM_I read (register `0x0F`, expected `0x6A`) and returns `-ENODEV` early on mismatch so the retry kicks in quickly instead of timing out inside the SW_RESET poll.
   - The SW_RESET completion poll is capped at 50 iterations (~50 ms), followed by a defensive 10 ms settle delay before the next register write.
4. On failure, sleep 10 ms and loop back to step 1 (max 3 attempts).

A successful registration logs `IMU: LSM6DS3TR-C registered on I2C2 addr=0x6a (attempt N)` (zero-origin, normally `N=0`). Retry-loop failures emit `snwarn`; giving up after 3 attempts emits `snerr` and `/dev/uorb/sensor_imu0` is not registered. In practice, if the chip cannot be revived within 3 attempts, the chip's internal state is also corrupted and a full battery cut-off is required.

For reference, pybricks performs the same bus recovery on every I2C2 init in `HAL_I2C_MspInit()` (10 SCL clock toggles). NuttX's standard `stm32_i2c_reset()` is strictly more thorough — it adds open-drain pin reconfig and a START/STOP transition.

## 7. Sensor Fusion

Uses a quaternion-based complementary filter ported from pybricks.

### Initialization

Generates a quaternion from the first gravity vector measurement.

### Per-Sample Processing

1. **Apply calibration**: Accelerometer offset+scale correction, gyroscope bias+scale correction
2. **Gravity estimation**: Extract estimated gravity vector from the 3rd row of the rotation matrix
3. **Error signal**: Compute cross product of estimated gravity and measured gravity
4. **Stationary degree**: Compute blending coefficient (0..1)
5. **Fused angular velocity**: `omega_fused = omega_gyro + correction * fusion_gain`
6. **Quaternion integration**: Forward Euler method + normalization

### Heading

- **1D heading**: Per-axis rotation integration
- **3D heading**: atan2 projection of the x-axis onto the horizontal plane

## 8. Calibration

Phase 2.5 (Issues #145 / #146) introduced a two-stage calibration:

- **Offline (Tedaldi)** — `imu_tk` (Pretto 2014) jointly solves bias / scale /
  misalignment from a ~5 min capture session and stores the result as
  `/mnt/flash/imu_cal.txt` (properties format).
- **Online (EMA)** — `drivebase_imu` tracks post-boot temperature drift with
  an α=0.1 exponential moving average, seeded by the offline bias.

The two stages cooperate: offline removes per-unit constant bias / scale /
axis non-orthogonality, online catches temperature-dependent residual.

### Calibration file format

`/mnt/flash/imu_cal.txt` (schema_version=1, properties format):

```
schema_version = 1
nominal_gyro_radps_per_lsb = 6.108652e-04
nominal_accel_ms2_per_lsb  = 5.985504e-04
fsr_gy_dps = 1000
fsr_xl_g = 2
odr_hz = 107
ambient_temp_c = 28.1

gyro_bias_lsb_x1000  = 22406 65567 11412
accel_bias_lsb_x1000 = -390870 12782 80597
gyro_M_x1000  = 997 -1 18 5 988 -3 -1 -10 1002
accel_M_x1000 = 1008 -2 9 0 1002 3 0 0 1002
```

- `*_bias_lsb_x1000`: 3-axis bias (raw LSB × 1000)
- `*_M_x1000`: 3×3 matrix combining misalignment × scale (row-major, x1000)
- Apply (LSB domain): `corrected[i] = sum_j(M[i][j] × (raw[j] − bias[j])) / 1000`
- Dividing M by the nominal sensitivity yields M_runtime with diagonal ≈ 1000
  (= within 1% of nominal when the unit matches spec)

### Host pipeline (cal generation)

See `tools/imu_cal/README.md` for full details. Three steps:

1. **Capture** — Use the ImuViewer "IMU Capture (Tedaldi)" expander to record
   ~5 min of static-pose + rotation as 27 B/sample frames (frame_type 0x03,
   104 Hz, FSR ±2g / ±1000dps). Tedaldi uses **the first 10 seconds** as its
   noise-floor reference window, so leave the robot untouched right after
   Start. Aim for ≥12 distinct static poses of ≥5 s each interleaved with
   2–3 s hand rotations.
2. **Run Tedaldi** — `tools/imu_cal/run_imu_tk.sh <session_dir>` invokes the
   `ghcr.io/owhinata/ubuntu-imu_tk` Docker image and emits two `.calib`
   files (T, K, B per triad).
3. **Generate cfg** — `tools/imu_cal/imu_tk_output_to_cfg.py --session-dir
   <session_dir>` folds the `.calib` + meta into `imu_cal.txt`. A warning
   prints if the M diagonal deviates more than 5% from 1000.

### Deploy to the Hub

```text
nsh> rz                                  # zmodem receive
(host) sx -k imu_cal.txt > /dev/ttyACM0
nsh> mv received_file /mnt/flash/imu_cal.txt
nsh> reboot
```

After reboot, `dmesg | grep imu_cal` confirms the load:

```text
drivebase: imu_cal: loaded /mnt/flash/imu_cal.txt (FSR=±1000 dps, ODR=104 Hz, T=23°C)
```

### Hub-side application

`drivebase_imu` applies matmul + bias subtraction in `integrate()` per
sample (LSB domain). Math is x1000 fixed-point integer only. The EMA seeds
on the cal-loaded bias and updates during stationary windows. `drivebase
_imu show` exposes both the cal values and the runtime state.

### ImuViewer-side application (Issue #146)

The same cal can be applied to the host telemetry stream (frame_type 0x02):

1. Open the ImuViewer **Telemetry** expander
2. Tick **Apply offline calibration**
3. Pick `imu_cal.txt` with **Browse...**
4. Status line reads `loaded · FSR ±1000dps/±2g · ODR 107Hz · T 28.1°C`

`SensorAggregator.OnBundle()` applies the matmul before FSR scaling — same
math as the Hub. The Madgwick filter then sees cal-corrected input. If the
BUNDLE header's FSR disagrees with the cal file's FSR, the aggregator logs
a warning and skips cal application until the FSRs match again.

### Verification (acceptance)

| verb | Purpose | Acceptance |
|---|---|---|
| `drivebase _imu show` | Current cal + runtime bias / temperature | `cal.loaded=1`, M diagonal ≈ 1000 |
| `drivebase _imu drift <sec>` | Static-state drift rate | `drift_mdegpm` < 500 (real Tedaldi run achieved 3 mdeg/min) |
| `drivebase _imu verify <deg>` | Manual rotation vs gyro integral | Phase 2.5 raw-Z integral was `target × cos(tilt)`; Phase 3a (Section 13) recovers world-vertical yaw via Madgwick so `actual_mdeg ≈ target_deg × 1000 ± 4°` regardless of mounting tilt |

## 9. Stationary Detection

### Algorithm

1. Compute a slow moving average of 125 samples as the baseline
2. Compare each sample against gyro and accel thresholds
3. When determined to be stationary for approximately 1 second continuously:
   - Call the bias estimation callback
   - Measure the actual sample time

## 10. Axis Sign Correction

Axis signs are corrected to match the Hub PCB mounting orientation:

| Axis | Sign |
|---|---|
| X | -1 |
| Y | +1 |
| Z | -1 |

!!! note
    Axis sign correction is not yet applied. Raw data is output as-is.

## 11. IMU Processing Library (apps/imu/)

A sensor-agnostic processing layer that consumes uORB data.

| File | Description |
|---|---|
| `imu_main.c` | NSH command `imu` + daemon |
| `imu_fusion.c/h` | Sensor fusion (complementary filter) |
| `imu_stationary.c/h` | Stationary detection |
| `imu_calibration.c/h` | Calibration persistence |
| `imu_geometry.c/h` | Quaternion, vector, matrix operations |
| `imu_types.h` | Type definitions |

### NSH Commands

```
nsh> imu start    # Start daemon
nsh> imu accel    # Show acceleration
nsh> imu gyro     # Show angular velocity
nsh> imu status   # Show status
nsh> imu stop     # Stop daemon
```

## 12. pybricks Feature Mapping

| pybricks Feature | NuttX Implementation | Notes |
|---|---|---|
| `imu.up()` | `imu_fusion` gravity vector classification | 6-face detection |
| `imu.tilt()` | `imu_fusion` derived from gravity vector | Pitch/Roll |
| `imu.acceleration()` | uORB `sensor_imu0` ax/ay/az read, converted to mm/s² in apps/imu using current FSR | Conversion in apps/imu |
| `imu.angular_velocity()` | uORB `sensor_imu0` gx/gy/gz read, converted to deg/s in apps/imu using current FSR | Conversion in apps/imu |
| `imu.heading()` | `imu_fusion` 3D heading | Z-axis rotation |
| `imu.rotation()` | `imu_fusion` 1D heading | Per-axis |
| `imu.orientation()` | `imu_fusion` quaternion -> Euler angles | Yaw/Pitch/Roll |
| `imu.settings()` | `imu_calibration` + NSH command | Threshold settings |
| `imu.stationary()` | `imu_stationary` state query | bool |
| `imu.reset_heading()` | `imu_fusion` heading reset | Zero integration values |

## 13. Drivebase IMU Madgwick fusion (Phase 3a, Issue #147)

`apps/drivebase/drivebase_imu` is the user-space integration layer the daemon
consumes for its optional gyro-locked heading source. Phase 3a replaces the
former raw `∫gz_corr dt` heading with a Madgwick 6-DOF IMU-only fusion stage
so a tilted Hub mounting no longer biases the heading by `cos(tilt)`.

### Why Madgwick instead of raw integration

The Phase 2.5 verify on a ~51° tilted bench measured 228.7° on a physical
360° turn (= 360 × cos(51°) ≈ 226.5°). Raw Z-gyro integration measures
rotation about the IMU's physical Z, not the robot's actual rotation axis;
with the LSM6DSL mounted off-vertical the projection error is structural.
Madgwick fuses accel (gravity) into the quaternion estimate, then we extract
world-vertical yaw — which is the robot heading the daemon's PID actually
wants.

### Per-sample pipeline (LSM6DSL ODR 833 Hz)

`integrate()` in `apps/drivebase/drivebase_imu.c`:

1. **FSR transition guard** (Issue #139) — rescale cached gyro bias and idle
   threshold to preserve their physical meaning when the driver switches
   FSR live.
2. **Gyro Tedaldi matmul** — `g_corr_x1000[i] = Σⱼ M[i][j] × (g_raw[j] × 1000 −
   bias[j]) / 1000` (Phase 2.5, integer x1000).
3. **Idle-EMA bias update** on `gz_corr` to follow temperature drift.
4. **Accel FSR match guard** — compare `fsr_xl_idx_to_g(batch[i].fsr_xl_idx)`
   against `cal.fsr_xl_g`; on mismatch fall back to identity correction and
   raise `accel_fsr_match=0` (visible in `_imu show`). Madgwick still gets a
   roughly-correct gravity direction (just wrong scale), and the filter
   re-normalises so the bound on yaw drift stays predictable.
5. **Accel Tedaldi matmul** (matched branch) or **identity** (mismatch).
6. **Float conversion**:
   - `ω_f[rad/s] = corr_x1000 × (gyro_mdps_num/1000) × π/180 / 1e6`
   - `a_f[g] = corr_x1000 × (fsr_xl_g / 32768) / 1000`
7. **Stationary-gated β** (pybricks `pbio/src/imu.c:327` form, converted to
   rad/s+g):
   ```
   stationary = min(1, ACCL_MIN_G/max(|‖a‖-1|, ACCL_MIN_G))
              × min(1, GYRO_MIN_RADPS/max(‖ω‖, GYRO_MIN_RADPS))
   β_eff = 0.05 × stationary
   ```
   At idle, β=0.05 (full accel correction); under wheel impact / fast
   rotation the accel error climbs and β drops toward zero so vibration
   cannot pull yaw via the accel term.
8. **Quaternion bootstrap** — first sample seeds the quaternion via
   shortest-arc rotation from (0,0,1) onto the normalised accel, so tilt
   estimate converges immediately rather than after seconds of β-blend.
9. **Madgwick update** — direct C port of
   `host/ImuViewer/src/ImuViewer.Core/Filters/MadgwickFilter.cs` (β=0.05 for
   Hub/host numerical parity).

### Per-drain yaw extraction

After the drain batch is processed:

```
ψ_curr = atan2f(2(q0 q3 + q1 q2), 1 - 2(q2² + q3²))
dψ     = wrap_pi(ψ_curr - ψ_prev)
heading_mdeg += dψ × 180000/π
ψ_prev = ψ_curr
```

`atan2f` runs once per drain (~500 Hz), not per sample. At 2000 dps the
worst-case angular displacement across a 2 ms RT tick is ~4 deg, far from
±π, so unwrap accuracy holds.

### FPU and CPU envelope

- `CONFIG_ARCH_FPU=y` + `CONFIG_LIBM_NEWLIB=y` (compile-time `#error`
  guards in `drivebase_imu.c`).
- NuttX lazy FPU is OFF (`arch/arm/src/armv7-m/arm_fpuconfig.c:63`); context
  switching already saves S16–S31 so Phase 3a does not add a new switch
  cost.
- Estimated load at 833 Hz: Madgwick ~0.2 % CPU, `atan2f` per drain ~0.05 %.
- Hard limit on `integrate()` call site: task context only (RT tick
  work-queue or `_imu` CLI). Not safe from an ISR.

### Timestamp wrap fix

`sensor_imu.timestamp` is the low 32 bits of `CLOCK_BOOTTIME` µs (≈71 min
wrap). Phase 3a replaces the legacy `last_sample_ts_us != 0 && ts >
last_sample_ts_us` guard with a `last_sample_valid` flag plus a `(uint32_t)
(ts - last)` modular subtract, so a wrap event never produces negative dt.

### Diagnostics

`drivebase _imu show` prints:

```
madgwick.q_x1000=<w> <x> <y> <z>    # quaternion × 1000 (≈ 1000 0 0 0 at boot)
madgwick.tilt_mdeg=<value>           # angle from world vertical (mdeg)
madgwick.beta_x1000=50               # base fusion gain × 1000 (default)
madgwick.initialized=1               # quaternion seeded
madgwick.accel_fsr_match=1           # 1 = accel Tedaldi cal applied this
                                     #     sample (cal loaded AND live
                                     #     FSR matches cal.fsr_xl_g).
                                     # 0 = identity fallback (cal not
                                     #     loaded OR live FSR drifted
                                     #     away from cal.fsr_xl_g).
```

`drivebase _imu verify <deg>` (Section 8 acceptance row) now treats the
return value as the unwrapped world-vertical yaw, so a physical 360° turn
on a 51° tilted bench reads ≈ 360 000 mdeg.

### Stale detection API

`bool db_imu_is_stale(im, now_us, threshold_us)` returns true when no drain
has produced ≥ 1 sample within `threshold_us` (default
`DB_IMU_DEFAULT_STALE_THRESHOLD_US = 50 ms`, 25 RT ticks).  Phase 3b uses
this as a guard in the heading PID injection path so a brief I²C recovery
(Issue #102) falls back to encoder heading instead of integrating stale
samples.

## 14. Drivebase heading PID injection (Phase 3b, Issue [#148](https://github.com/owhinata/spike-nx/issues/148))

After Phase 3a, `db_imu_get_heading_mdeg()` returns world-vertical robot
heading and `drivebase _imu show` tracks manual rotation to ±0.003°.
Phase 3b wires that into the drivebase aggregate heading PID's **state
input** — encoder `(sR-sL)/2` alone cannot close the loop on wheel slip,
gear backlash, or motor-to-motor mismatch.

### Structure

| Concept | Source | Where |
|---|---|---|
| `use_gyro_requested` | user-requested (`set-gyro` ioctl / cfg `use_gyro_plus1`) | `db_drivebase_s.use_gyro_requested` |
| `use_gyro_latched` | captured at command-start when capturable | `db_drivebase_s.use_gyro_latched` |
| `gyro_origin_mdeg` | snapshot of `raw - angle_mdeg` at `set-gyro` / `set_origin` / `reset` / latch | `db_drivebase_s.gyro_origin_mdeg` |
| `gyro_origin_valid` | whether the baseline is trusted | `db_drivebase_s.gyro_origin_valid` |

`db_drivebase_capture_start_heading(db, state, now_us, do_latch)` (in
`drivebase_drivebase.c`) is called from four entry points
(`setup_position_move` / `drive_curve` / `drive_forever` / `db_drivebase_
stop`) and from the per-tick `db_drivebase_update`.  With do_latch=true it
latches use_gyro and snapshots the origin in place if none is valid; with
do_latch=false it just overwrites `state->heading_x_mdeg` with the
origin-relative IMU value, gated on every runtime check passing — any
transient (stale sample, lost calibration) lets the encoder value pass
through for that tick.

### Race protection by construction

- The ioctl path only writes `requested`; it never touches `latched`.  Mid-
  motion `set-gyro` returns `-EBUSY`.
- A mid-motion IMU stall just bounces the per-tick injection to encoder
  fallback without changing `latched`.  When the IMU recovers, gyro
  injection resumes seamlessly on the next tick.
- In-flight retargeting (`drive_forever` re-arm) re-uses the existing
  origin instead of recapturing, so the publish baseline never jumps under
  the user while a motion is running.

### RT tick order

```
chardev → db_imu_drain_and_update → db_drivebase_update → publish overwrite
```

Pre-Phase-3b the IMU drain came after `db_drivebase_update`, so the PID
read a one-tick-old IMU sample.  Phase 3b moves the drain ahead so the
sample from this tick reaches the PID.

### Publish overwrite

`drivebase_daemon.c` overwrites `st.angle_mdeg` with `(int32_t)CLAMP(raw -
gyro_origin_mdeg, INT32_MIN, INT32_MAX)` whenever `latched == 1D` (mid-
motion) or `requested == 1D && origin_valid` (idle after `set-gyro`).  The
PID and the user-visible publish share a single origin, so they stay in
SSOT lock-step.

### Out of scope (deferred Issues)

- X/Y bias EMA temperature tracking — Phase 3a Codex CONCERN 2 territory
- 3D mode (`set-gyro 3d`) — returns `-ENOSYS` until Phase 7
- Evaluation of the I-term spike on IMU-stale → encoder fallback — bench
  acceptance assumes no stalls; this is a follow-up Issue
