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
| Startup accel FSR | ±8 g (runtime-tunable to 2/4/8/16 g via ioctl) |
| Startup gyro FSR | 2000 dps (runtime-tunable to 125/250/500/1000/2000 dps via ioctl) |

## 3. Board Wiring

| Pin | Function | Description |
|---|---|---|
| PB10 | I2C2_SCL | I2C clock (AF4) |
| PB3 | I2C2_SDA | I2C data (AF9, F413-specific) |
| PB4 | INT1 | Gyro DRDY interrupt (EXTI4) |

## 4. Register Configuration

| Register | Setting | Purpose |
|---|---|---|
| CTRL1_XL (0x10) | ODR=833Hz, FS=±8g | Accel: ODR and full scale |
| CTRL2_G (0x11) | ODR=833Hz, FS=2000dps | Gyro: ODR and full scale |
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

Defined in `nuttx/include/nuttx/uorb.h`:

| Field | Type | Description |
|---|---|---|
| timestamp | uint32_t | Low 32 bits of CLOCK_BOOTTIME us (~71m35s wraparound).  ARMv7-M 4-byte aligned word load/store is single-copy atomic, so ISR -> worker handoff is tearing-free. |
| ax / ay / az | int16_t | Accel raw LSB, Hub body frame (chip frame Y/Z negated by the driver) |
| gx / gy / gz | int16_t | Gyro raw LSB, Hub body frame (chip frame Y/Z negated by the driver) |
| temperature_raw | int16_t | OUT_TEMP raw, refreshed every 16 samples (stale in between) |
| reserved | int16_t | Alignment padding |

### ioctl

| ioctl | Argument | Behavior |
|---|---|---|
| `SNIOC_SET_INTERVAL` | uint32 (period_us) | Pick the closest ODR whose period is <= `period_us` |
| `SNIOC_SETSAMPLERATE` | uint32 (Hz: 13/26/52/104/208/416/833/1660/3330/6660) | Set ODR by frequency |
| `LSM6DSL_IOC_SETACCELFSR` | uint32 (g: 2/4/8/16) | Set accel full-scale range |
| `LSM6DSL_IOC_SETGYROFSR` | uint32 (dps: 125/250/500/1000/2000) | Set gyro full-scale range |

Calling any of the configuration ioctls while the sensor is active
(after `SNIOC_ACTIVATE`) returns `-EBUSY`.  Deactivate the sensor before
reconfiguring.

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

### Gyroscope Bias

Estimated using exponential smoothing during stationary state.

### Accelerometer Calibration

Offset and scale computed from 6-posture gravity measurements (positive and negative directions of each axis).

### Gyroscope Scale

Per-axis correction factor. Adjusted so that one rotation is approximately 360 degrees.

### Storage Format

Settings are persisted as a binary file at `/data/imu_cal.bin`.

### Default Values

| Parameter | Default Value |
|---|---|
| Gravity | ±9806.65 mm/s² |
| Scale | 360 |
| Gyro threshold | 2 deg/s |
| Accel threshold | 2500 mm/s² |

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
