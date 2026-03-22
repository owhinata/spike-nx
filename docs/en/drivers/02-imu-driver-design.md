# IMU Driver Design

## 1. Overview

The LSM9DS0 (9-axis: accelerometer + gyroscope + magnetometer) is used as the development target, paired with an STM32F4 Discovery board for early development. The goal is to achieve feature parity with the pybricks LSM6DS3TR-C driver on NuttX.

After development is complete, the LSM9DS0 driver will be replaced with an LSM6DS3TR-C driver when migrating to the SPIKE Prime Hub. The IMU processing library (apps/imu/) is sensor-agnostic and requires no changes.

---

## 2. Device: LSM9DS0

### Architecture

The LSM9DS0 uses a two-die architecture.

| Die | Sensors | I2C Address (SA0=H/L) | WHO_AM_I |
|-----|---------|------------------------|----------|
| G die | Gyroscope | 0x6B / 0x6A | 0xD4 |
| XM die | Accelerometer + Magnetometer | 0x1D / 0x1E | 0x49 |

The SA0 pin selects between two addresses for each die.

### Performance Specifications

| Parameter | Gyro (G) | Accelerometer (XM) | Magnetometer (XM) |
|-----------|----------|--------------------|--------------------|
| Max ODR | 760 Hz | 1600 Hz | 100 Hz |
| Full-scale | 245 / 500 / 2000 dps | ±2 / 4 / 6 / 8 / 16 g | ±2 / 4 / 8 / 12 gauss |

### I2C Multi-byte Reads

Setting bit 7 (0x80) of the register address enables auto-increment mode. This is required for 6-byte continuous reads (2 bytes each for X, Y, Z).

---

## 3. NuttX Driver Architecture

The driver follows the existing NuttX LSM9DS1 driver pattern with the following file structure.

```
[Kernel Space]
  lsm9ds0_base.h    - Register definitions, device struct, ops
  lsm9ds0_base.c    - I2C helpers, config/start/stop for each sensor
  lsm9ds0_uorb.c    - uORB registration, poll thread, data scaling
  lsm9ds0.h         - Public API (config struct, register function)

[uORB Topics]
  /dev/uorb/sensor_accel0   (m/s², from XM die)
  /dev/uorb/sensor_gyro0    (rad/s, from G die)
  /dev/uorb/sensor_mag0     (gauss, from XM die)
```

### Key Difference from LSM9DS1

The LSM9DS0 has **separate I2C addresses** for the G die and XM die. Unlike the LSM9DS1, where accelerometer and gyroscope share a single address, the LSM9DS0 gyroscope has its own independent address. The device struct must hold two I2C addresses.

### Default Scale Settings

| Sensor | Default | Rationale |
|--------|---------|-----------|
| Accelerometer | ±8 g | Covers typical robot motion range |
| Gyroscope | 2000 dps | Handles fast rotations |
| Magnetometer | 4 gauss | Sufficient for typical ambient fields |

### Poll Thread

A dedicated thread periodically reads data from all 3 sensors and pushes uORB events.

---

## 4. Data Flow

```
I2C read (6 bytes per sensor)
  → raw int16 × 3 (X, Y, Z)
  → two's complement conversion
  → float scale multiplication
  → uORB push_event (sensor_accel / sensor_gyro / sensor_mag struct)
```

Raw values are stored as signed 16-bit integers (little-endian) for each sensor. After scale multiplication to convert to physical units, the data is published to the corresponding uORB topic.

---

## 5. IMU Processing Library (apps/imu/)

A sensor-agnostic processing layer that consumes uORB data. The same code works with both the LSM9DS0 and LSM6DS3TR-C.

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

When migrating to the SPIKE Prime Hub, replace the LSM9DS0 driver with an LSM6DS3TR-C driver.

### Key Differences

| Aspect | LSM9DS0 (Discovery) | LSM6DS3TR-C (Hub) |
|--------|---------------------|-------------------|
| I2C address | 2 addresses (G + XM) | 1 address (accel+gyro combined) |
| Magnetometer | Yes | No |
| I2C bus | I2C1 | I2C2 |
| ODR | 760 Hz (gyro) | 833 Hz |

### Hub I2C2 Pin Assignment

| Pin | Function | AF |
|-----|----------|-----|
| PB10 | SCL | AF4 |
| PB3 | SDA | AF9 (STM32F413-specific) |

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

## 11. Board Wiring (Discovery + LSM9DS0)

### I2C1 Connection

| Discovery Pin | LSM9DS0 Pin | Description |
|--------------|-------------|-------------|
| PB6 | SCL | I2C1 clock |
| PB7 | SDA | I2C1 data |
| 3.3V | VCC | Power supply |
| GND | GND | Ground |

### SA0 Configuration

The SA0 pin selects the I2C address. Default is HIGH (pull-up).

| SA0 | G Address | XM Address |
|-----|-----------|------------|
| HIGH | 0x6B | 0x1D |
| LOW | 0x6A | 0x1E |
