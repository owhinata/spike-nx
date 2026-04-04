# IMU Driver Design

## 1. Overview

The LSM6DSL on the B-L4S5I-IOT01A Discovery board is adopted as the development target. The ultimate goal is to achieve functionality equivalent to the pybricks LSM6DS3TR-C driver on NuttX.

The LSM6DSL and LSM6DS3TR-C (mounted on the SPIKE Prime Hub) are from the same ST IMU family and have compatible register layouts. The IMU processing library (apps/imu/) is sensor-agnostic, so no changes are required when migrating between sensors.

## 2. Device Specifications

The LSM6DSL/LSM6DS3TR-C is a single-die 6-axis IMU integrating a 3-axis accelerometer and 3-axis gyroscope.

| Sensor | I2C Address (SDO/SA0) | WHO_AM_I |
|---|---|---|
| Accelerometer + Gyroscope | 0x6A (LOW) / 0x6B (HIGH) | 0x6A |

### Performance Specifications

| Parameter | Gyroscope | Accelerometer |
|---|---|---|
| Max ODR | 6.66 kHz | 6.66 kHz |
| Full Scale | 125 / 250 / 500 / 1000 / 2000 dps | +-2 / 4 / 8 / 16 g |

### Default Scales

| Sensor | Default | Rationale |
|---|---|---|
| Accelerometer | +-2 g | Chip reset default, sufficient for orientation detection |
| Gyroscope | 250 dps | Chip reset default |

## 3. Board Wiring

### B-L4S5I-IOT01A (Development)

| Pin | Function | Description |
|---|---|---|
| PB10 | I2C2_SCL | I2C clock |
| PB11 | I2C2_SDA | I2C data |
| PD11 | INT1 | Gyro DRDY interrupt (EXTI11) |

### SPIKE Prime Hub (Final Target)

| Pin | Function | Description |
|---|---|---|
| PB10 | I2C2_SCL | I2C clock |
| PB3 | I2C2_SDA | I2C data (AF9) |
| PB4 | INT1 | Gyro DRDY interrupt (EXTI4) |

## 4. Key Register Settings

| Register | Setting | Purpose |
|---|---|---|
| CTRL3_C (0x12) | BDU=1, IF_INC=1 | Block data update, address auto-increment |
| CTRL5_C (0x14) | ROUNDING=011 | Rounding for burst reads |
| DRDY_PULSE_CFG (0x0B) | DRDY_PULSED=1 | Pulsed mode DRDY (more reliable) |
| INT1_CTRL (0x0D) | INT1_DRDY_G=1 | Route gyro DRDY to INT1 |

## 5. Data Acquisition

A single INT1 (gyro DRDY) interrupt triggers a 12-byte bulk read:

- Bytes 0-5: Gyro X/Y/Z (OUTX_L_G 0x22 through OUTZ_H_G 0x27)
- Bytes 6-11: Accel X/Y/Z (OUTX_L_A 0x28 through OUTZ_H_A 0x2D)

Since both sensors operate at the same ODR, accel data is also updated when the gyro DRDY fires.

### Data Flow

```
INT1 (gyro DRDY) fires
  -> HPWORK: 12-byte bulk read from 0x22
  -> raw int16 x 6 (gyro XYZ + accel XYZ)
  -> float scale multiplication
  -> push_event delivers to sensor_gyro0 and sensor_accel0
```

Accelerometer and gyroscope events share the same timestamp, guaranteeing synchronized data.

### Data Acquisition Modes

| Mode | Mechanism | Usage Condition |
|---|---|---|
| Interrupt | INT1 DRDY -> HPWORK -> burst read | Board provides `attach` callback |
| Polling | kthread sleep -> burst read | `attach = NULL` |

## 6. NuttX Driver Architecture

The driver is implemented as a uORB sensor driver in the nuttx fork.

```
[Kernel Space]
  lsm6dsl_uorb.h    - config struct, registration API
  lsm6dsl_uorb.c    - I2C helpers, burst read, uORB registration

[uORB Topics]
  /dev/uorb/sensor_accel0   (m/s^2)
  /dev/uorb/sensor_gyro0    (rad/s)
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
| Gravity | +-9806.65 mm/s^2 |
| Scale | 360 |
| Gyro threshold | 2 deg/s |
| Accel threshold | 2500 mm/s^2 |

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

## 11. Hub Migration Guide

When migrating to the SPIKE Prime Hub, replace the LSM6DSL driver with the LSM6DS3TR-C driver.

### Key Differences

| Item | LSM6DSL (B-L4S5I) | LSM6DS3TR-C (Hub) |
|---|---|---|
| I2C Address | 0x6A | 0x6A |
| I2C Bus | I2C2 (PB10/PB11) | I2C2 (PB10/PB3) |
| WHO_AM_I | 0x6A | 0x6A |
| ODR | Max 6.66 kHz | Max 6.66 kHz |

The sensor fusion and calibration code in the processing library (`apps/imu/`) requires no changes. Since the uORB topic interface is identical, only the driver needs to be replaced.

## 12. IMU Processing Library (apps/imu/)

A sensor-agnostic processing layer that consumes uORB data.

| File | Description |
|---|---|
| `imu_types.h` | Type definitions |
| `imu_geometry.c/h` | Quaternion, vector, matrix operations |
| `imu_stationary.c/h` | Stationary detection |
| `imu_fusion.c/h` | Sensor fusion (complementary filter) |
| `imu_calibration.c/h` | Calibration persistence |
| `imu_main.c` | NSH command `imu` + daemon |

## 13. pybricks Feature Mapping

| pybricks Feature | NuttX Implementation | Notes |
|---|---|---|
| `imu.up()` | `imu_fusion` gravity vector classification | 6-face detection |
| `imu.tilt()` | `imu_fusion` derived from gravity vector | Pitch/Roll |
| `imu.acceleration()` | uORB `sensor_accel0` direct read | mm/s^2 units |
| `imu.angular_velocity()` | uORB `sensor_gyro0` direct read | deg/s units |
| `imu.heading()` | `imu_fusion` 3D heading | Z-axis rotation |
| `imu.rotation()` | `imu_fusion` 1D heading | Per-axis |
| `imu.orientation()` | `imu_fusion` quaternion -> Euler angles | Yaw/Pitch/Roll |
| `imu.settings()` | `imu_calibration` + NSH command | Threshold settings |
| `imu.stationary()` | `imu_stationary` state query | bool |
| `imu.reset_heading()` | `imu_fusion` heading reset | Zero integration values |
