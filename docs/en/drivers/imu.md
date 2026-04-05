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
| ODR | 833 Hz (pybricks-aligned) |
| Accel FSR | ±8 g (0.244 mg/LSB) |
| Gyro FSR | 2000 dps (70.0 mdps/LSB) |

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
  -> HPWORK: 12-byte burst read from 0x22
  -> raw int16 x 6 (gyro XYZ + accel XYZ)
  -> float scale multiplication
  -> push_event delivers to sensor_gyro0 and sensor_accel0
```

## 6. NuttX Driver Architecture

### Board Layer

```
boards/spike-prime-hub/src/lsm6dsl_uorb.c   - I2C control, burst read, uORB registration
boards/spike-prime-hub/src/lsm6dsl_uorb.h   - config struct, registration API
boards/spike-prime-hub/src/stm32_lsm6dsl.c  - I2C2 init, INT1 interrupt setup, driver registration
```

### uORB Topics

```
/dev/uorb/sensor_accel0   (m/s^2)
/dev/uorb/sensor_gyro0    (rad/s)
```

### defconfig

```
CONFIG_STM32_I2C2=y
CONFIG_I2C=y
CONFIG_SCHED_HPWORK=y
CONFIG_SENSORS=y
CONFIG_APP_IMU=y
```

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
| `imu.acceleration()` | uORB `sensor_accel0` direct read | mm/s² units |
| `imu.angular_velocity()` | uORB `sensor_gyro0` direct read | deg/s units |
| `imu.heading()` | `imu_fusion` 3D heading | Z-axis rotation |
| `imu.rotation()` | `imu_fusion` 1D heading | Per-axis |
| `imu.orientation()` | `imu_fusion` quaternion -> Euler angles | Yaw/Pitch/Roll |
| `imu.settings()` | `imu_calibration` + NSH command | Threshold settings |
| `imu.stationary()` | `imu_stationary` state query | bool |
| `imu.reset_heading()` | `imu_fusion` heading reset | Zero integration values |
