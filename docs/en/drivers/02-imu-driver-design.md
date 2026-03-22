# IMU Driver Design

## 1. Overview

The LSM6DSL (6-axis: accelerometer + gyroscope) on the B-L4S5I-IOT01A Discovery board is used as the development target. The goal is to achieve feature parity with the pybricks LSM6DS3TR-C driver on NuttX.

The LSM6DSL and LSM6DS3TR-C (on SPIKE Prime Hub) are in the same ST IMU family with compatible register layouts. The IMU processing library (apps/imu/) is sensor-agnostic and requires no changes when migrating between sensors.

---

## 2. Device: LSM6DSL

### Architecture

The LSM6DSL is a single-die 6-axis IMU combining a 3-axis accelerometer and 3-axis gyroscope.

| Sensor | I2C Address (SDO/SA0) | WHO_AM_I |
|--------|------------------------|----------|
| Accel + Gyro | 0x6A (LOW) / 0x6B (HIGH) | 0x6A |

### Performance Specifications

| Parameter | Gyroscope | Accelerometer |
|-----------|-----------|---------------|
| Max ODR | 6.66 kHz | 6.66 kHz |
| Full-scale | 125 / 250 / 500 / 1000 / 2000 dps | ±2 / 4 / 8 / 16 g |

### Data Acquisition (pybricks pattern)

A single DRDY interrupt on INT1 (gyro data ready) triggers a 12-byte burst read:
- Bytes 0-5: Gyro X/Y/Z (OUTX_L_G 0x22 through OUTZ_H_G 0x27)
- Bytes 6-11: Accel X/Y/Z (OUTX_L_A 0x28 through OUTZ_H_A 0x2D)

Both sensors share the same ODR so accel data is always fresh when gyro DRDY fires.

### Key Register Configuration

| Register | Setting | Purpose |
|----------|---------|---------|
| CTRL3_C (0x12) | BDU=1, IF_INC=1 | Block Data Update, auto-increment |
| CTRL5_C (0x14) | ROUNDING=011 | Enable rounding for burst reads |
| DRDY_PULSE_CFG (0x0B) | DRDY_PULSED=1 | Pulsed DRDY mode (more reliable) |
| INT1_CTRL (0x0D) | INT1_DRDY_G=1 | Route gyro DRDY to INT1 |

---

## 3. NuttX Driver Architecture

The driver is implemented as a uORB sensor driver in the nuttx fork.

```
[Kernel Space]
  lsm6dsl_uorb.h    - Config struct, registration API
  lsm6dsl_uorb.c    - I2C helpers, burst read, uORB registration

[uORB Topics]
  /dev/uorb/sensor_accel0   (m/s²)
  /dev/uorb/sensor_gyro0    (rad/s)
```

### Data Acquisition Modes

| Mode | Mechanism | When used |
|------|-----------|-----------|
| Interrupt | INT1 DRDY → HPWORK → burst read | Board provides `attach` callback |
| Polling | kthread sleep → burst read | `attach = NULL` |

### Default Scale Settings

| Sensor | Default | Rationale |
|--------|---------|-----------|
| Accelerometer | ±2 g | Chip reset default, sufficient for orientation |
| Gyroscope | 250 dps | Chip reset default |

---

## 4. Data Flow

```
INT1 (gyro DRDY) fires
  → HPWORK: burst read 12 bytes from 0x22
  → raw int16 × 6 (gyro XYZ + accel XYZ)
  → float scale multiplication
  → push_event to sensor_gyro0 and sensor_accel0
```

Both accel and gyro events share the same timestamp, ensuring synchronized data.

---

## 5. IMU Processing Library (apps/imu/)

A sensor-agnostic processing layer that consumes uORB data. The same code works with both the LSM6DSL and LSM6DS3TR-C.

### File Structure

| File | Description |
|------|-------------|
| `imu_types.h` | Type definitions |
| `imu_geometry.c/h` | Quaternion, vector, and matrix math |
| `imu_stationary.c/h` | Stationarity detection |
| `imu_fusion.c/h` | Sensor fusion (complementary filter) |
| `imu_calibration.c/h` | Calibration persistence |
| `imu_main.c` | NSH command `imu` + daemon |

---

## 6. Sensor Fusion

Uses a quaternion-based complementary filter ported from pybricks.

### Initialization

A quaternion is generated from the first gravity vector measurement.

### Per-sample Processing

1. **Apply calibration**: Accelerometer offset+scale correction, gyroscope bias+scale correction
2. **Gravity estimate**: Extract estimated gravity vector from 3rd row of rotation matrix
3. **Error signal**: Compute cross product of estimated vs. measured gravity
4. **Stationary measure**: Compute blending factor (0..1)
5. **Fused angular velocity**: `omega_fused = omega_gyro + correction * fusion_gain`
6. **Quaternion integration**: Forward Euler method + normalization

### Heading

- **1D heading**: Per-axis rotation integration
- **3D heading**: atan2 projection of x-axis onto horizontal plane

---

## 7. Calibration

### Gyro Bias

Estimated using exponential smoothing during stationary periods.

### Accelerometer Calibration

Uses 6-position gravity measurement (positive and negative direction for each axis) to compute offsets and scales.

### Gyro Scale

Per-axis correction factor, adjusted so that one full rotation measures approximately 360 degrees.

### Storage Format

Settings are persisted as a binary file at `/data/imu_cal.bin`.

### Default Values

| Parameter | Default Value |
|-----------|---------------|
| Gravity | ±9806.65 mm/s² |
| Scale | 360 |
| Gyro threshold | 2 deg/s |
| Accel threshold | 2500 mm/s² |

---

## 8. Stationarity Detection

### Algorithm

1. Compute a 125-sample slow moving average as a baseline
2. Compare each sample against gyro and accel thresholds
3. After approximately 1 second of consecutive stationary samples:
   - Trigger bias estimation callback
   - Measure actual sample time

---

## 9. pybricks Feature Parity Table

| pybricks Feature | NuttX Implementation | Notes |
|------------------|---------------------|-------|
| `imu.up()` | `imu_fusion` gravity vector classification | 6-face detection |
| `imu.tilt()` | Derived from `imu_fusion` gravity vector | pitch/roll |
| `imu.acceleration()` | Direct uORB `sensor_accel0` read | mm/s² units |
| `imu.angular_velocity()` | Direct uORB `sensor_gyro0` read | deg/s units |
| `imu.heading()` | `imu_fusion` 3D heading | z-axis rotation |
| `imu.rotation()` | `imu_fusion` 1D heading | per-axis |
| `imu.orientation()` | `imu_fusion` quaternion to Euler angles | yaw/pitch/roll |
| `imu.settings()` | `imu_calibration` + NSH command | threshold config |
| `imu.stationary()` | `imu_stationary` state query | bool |
| `imu.reset_heading()` | `imu_fusion` heading reset | zero integration values |

---

## 10. Hub Migration Guide

When migrating to the SPIKE Prime Hub, replace the LSM6DSL driver with an LSM6DS3TR-C driver.

### Key Differences

| Aspect | LSM6DSL (B-L4S5I) | LSM6DS3TR-C (Hub) |
|--------|---------------------|-------------------|
| I2C address | 0x6A | 0x6A |
| I2C bus | I2C2 (PB10/PB11) | I2C2 (PB10/PB3) |
| WHO_AM_I | 0x6A | 0x6A |
| ODR | Up to 6.66 kHz | Up to 6.66 kHz |

### Axis Sign Correction

Axis signs must be corrected to account for the Hub PCB mounting orientation.

| Axis | Sign |
|------|------|
| X | -1 |
| Y | +1 |
| Z | -1 |

### Processing Library

The sensor fusion and calibration code in `apps/imu/` requires no changes. Since the uORB topic interface is identical, only the driver needs to be swapped.

---

## 11. Board Wiring (B-L4S5I-IOT01A)

### I2C2 Connection (onboard)

| Pin | Function | Description |
|-----|----------|-------------|
| PB10 | I2C2_SCL | I2C clock |
| PB11 | I2C2_SDA | I2C data |
| PD11 | INT1 | Gyro DRDY interrupt (EXTI11) |
