# Device Driver Implementation Plan

## 1. Overview

Implementation plan for NuttX device drivers to control SPIKE Prime Hub hardware. Based on bringup research (bringup/), drivers are implemented in stages.

---

## 2. Driver List and Priority

| # | Driver | Device Path | Priority | Dependencies |
|---|---|---|---|---|
| 1 | I/O Port Detection (DCM) | `/dev/legoport[0-5]` | **P0** | GPIO |
| 2 | LUMP UART Protocol | (internal) | **P0** | UART4/5/7/8/9/10 |
| 3 | H-Bridge Motor Control | `/dev/legomotor[N]` | **P0** | TIM1/3/4 PWM |
| 4 | Sensor Data Readout | `/dev/legosensor[N]` | **P1** | LUMP |
| 5 | TLC5955 LED Driver | `/dev/leds` | **P1** | SPI1 |
| 6 | IMU (LSM6DS3TR-C) | `/dev/imu0` | **P1** | I2C2 |
| 7 | USB CDC/ACM Console | `/dev/ttyACM0` | **Done** | OTG FS |
| 8 | W25Q256 SPI Flash | `/dev/mtdblock0` | **P2** | SPI2 |
| 9 | Power Management | (board init) | **P0** | PA13/PA14 GPIO |
| 10 | Bluetooth (TBD) | — | **P3** | USART2 |

### Priority Definitions

- **P0**: Essential for minimum motor/sensor operation
- **P1**: Required for basic robot operation
- **P2**: Useful for data logging and firmware updates
- **P3**: Wireless communication (future)

---

## 3. Implementation Phases

### Phase 1: Port Foundation (P0)

**Goal**: I/O port device detection and basic motor control

#### 1a. Power Management (Done)

PA13 (BAT_PWR_EN) and PA14 (PORT_3V3_EN) initialization implemented in `stm32_boot.c`.

#### 1b. I/O Port Detection (DCM)

pybricks reference: `legodev_pup.c`

- Device Connection Manager for 6 ports
- 2ms GPIO polling for passive device detection
- Stable detection (20 consecutive matches ≈ 400ms) confirms device type
- Implemented via NuttX HPWORK queue

**Port GPIO Pin Assignments:**

| Port | UART TX | UART RX | AF | GPIO1 | GPIO2 | UART BUF |
|---|---|---|---|---|---|---|
| A | PE8 | PE7 | AF8 (UART7) | PA5 | PA3 | PB2 |
| B | PD1 | PD0 | AF11 (UART4) | PA4 | PA6 | PD3 |
| C | PE1 | PE0 | AF8 (UART8) | PB0 | PB14 | PD4 |
| D | PC12 | PD2 | AF8 (UART5) | PB4 | PB15 | PD7 |
| E | PE3 | PE2 | AF11 (UART10) | PC13 | PE12 | PB5 |
| F | PD15 | PD14 | AF11 (UART9) | PC14 | PE6 | PB10 |

**DCM Detection Algorithm:**
```
1. Toggle GPIO1 between OUTPUT HIGH/LOW while reading GPIO2
2. Determine device category from GPIO2 response pattern:
   - Resistive → Passive device (light, external motor, etc.)
   - Pull-up → UART device (smart sensor/motor)
   - No response → Not connected
3. For UART devices:
   - Switch pins from GPIO → UART AF (stm32_configgpio)
   - Set UART BUF pin HIGH (enable RS485 transceiver)
   - Start LUMP handshake
```

#### 1c. LUMP UART Protocol

pybricks reference: `legodev_pup_uart.c` (1265 lines)

- LEGO UART Messaging Protocol implementation
- Sync phase: 2400 baud → device mode info → switch to 115200 baud
- Data phase: periodic data reception + keepalive (200ms)
- Driven by dedicated kernel thread with state machine

**State Machine:**
```
RESET → SYNC_TYPE → SYNC_MODES → SYNC_DATA → ACK → DATA
  ↑                                                  |
  └──────── timeout / error ←────────────────────────┘
```

**UART Ownership**: Port manager controls UART init/release. NuttX standard serial driver (`/dev/ttySx`) is not used for I/O port UARTs. A thin wrapper around STM32 UART registers is implemented instead.

#### 1d. H-Bridge Motor Control

pybricks reference: `motor_driver_hbridge_pwm.c`

- PWM duty setting + GPIO/AF mode switching for direction control
- 4 states: Coast / Brake / Forward / Reverse
- Duty range: -1000 to +1000

**Timer Assignments:**

| Timer | Ports | M1 Pins | M2 Pins | PWM Freq |
|---|---|---|---|---|
| TIM1 | A, B | PE9/CH1, PE13/CH3 | PE11/CH2, PE14/CH4 | 12 kHz |
| TIM3 | E, F | PC6/CH1, PC8/CH3 | PC7/CH2, PB1/CH4 | 12 kHz |
| TIM4 | C, D | PB6/CH1, PB8/CH3 | PB7/CH2, PB9/CH4 | 12 kHz |

### Phase 2: Sensors & LED (P1)

#### 2a. Sensor Data Readout

- Expose sensor data received via LUMP data phase through `/dev/legosensor[N]`
- ioctl for mode switching, read() for data retrieval
- Supported sensors:
  - Color Sensor (Type 61): COLOR, REFLT, AMBI, RGB_I, HSV
  - Ultrasonic Sensor (Type 62): DISTL, DISTS
  - Force Sensor (Type 63): FRAW

#### 2b. TLC5955 LED Driver

pybricks reference: `led_dual_pwm.c`, bringup research: `10-tlc5955-led-driver.md`

- Control 48ch 16-bit PWM LED driver via SPI1
- 5×5 LED matrix (RGB × 25 = 75 ch, 48ch used)
- LATCH pin: GPIO controlled
- NuttX `/dev/leds` or `/dev/userleds` interface

#### 2c. IMU (LSM6DS3TR-C)

pybricks reference: `imu_lsm6ds3tr_c_stm32.c`

- ST 6-axis IMU (3-axis accelerometer + 3-axis gyroscope)
- Connected via I2C2
  - SCL: PB10 (AF4)
  - SDA: PB3 (AF9)
  - INT1: PB4 (EXTI4, data-ready interrupt)
- Axis sign correction: X=-1, Y=+1, Z=-1 (Hub PCB mounting orientation)
- Can leverage NuttX sensor driver framework (`CONFIG_SENSORS_LSM6DSL` etc.)
- Exposed as `/dev/imu0` or `/dev/accel0` + `/dev/gyro0`

### Phase 3: Storage (P2)

#### 3a. W25Q256 SPI NOR Flash

bringup research: `11-w25q256-flash-driver.md`

- Access 32MB SPI NOR Flash via SPI2
- 4-byte address mode
- Use NuttX MTD driver (`CONFIG_MTD_W25`)
- Format with LittleFS or SmartFS
- Storage for user programs and data logs

### Phase 4: Wireless (P3)

#### 4a. Bluetooth

- BLE module via USART2 (PD5/PD6)
- Details TBD

---

## 4. Directory Structure

```
boards/spike-prime-hub/src/
  stm32_boot.c          # Power init (PA13/PA14) ← Done
  stm32_bringup.c       # Device driver registration
  stm32_usbdev.c        # USB CDC/ACM ← Done
  stm32_legoport.c      # I/O port manager (DCM)
  stm32_legomotor.c     # H-Bridge motor control
  stm32_tlc5955.c       # TLC5955 LED driver
  stm32_lsm6ds3.c       # IMU (LSM6DS3TR-C) init

drivers/lego/           # NuttX generic drivers (apps/ or in-board)
  lump_uart.c           # LUMP UART protocol engine
  lump_uart.h
  legodev.h             # Device type definitions
  legomotor.c           # Motor upper driver
  legosensor.c          # Sensor upper driver
```

---

## 5. ioctl Interface

### Port Manager (`/dev/legoport[N]`)

```c
#define LEGOPORT_GET_DEVICE_TYPE    _LEGOPORTIOC(0)
#define LEGOPORT_GET_DEVICE_INFO    _LEGOPORTIOC(1)
#define LEGOPORT_WAIT_CONNECT       _LEGOPORTIOC(2)
#define LEGOPORT_WAIT_DISCONNECT    _LEGOPORTIOC(3)
```

### Motor (`/dev/legomotor[N]`)

```c
#define LEGOMOTOR_SET_DUTY          _LEGOMOTORIOC(0)  // int16: -1000 to +1000
#define LEGOMOTOR_COAST             _LEGOMOTORIOC(1)
#define LEGOMOTOR_BRAKE             _LEGOMOTORIOC(2)
#define LEGOMOTOR_GET_POSITION      _LEGOMOTORIOC(3)  // int32: degrees
#define LEGOMOTOR_GET_SPEED         _LEGOMOTORIOC(4)  // int16: deg/s
#define LEGOMOTOR_GET_ABS_POS       _LEGOMOTORIOC(5)  // int16: absolute position
#define LEGOMOTOR_RESET_POS         _LEGOMOTORIOC(6)
```

### Sensor (`/dev/legosensor[N]`)

```c
#define LEGOSENSOR_SET_MODE         _LEGOSENSORIOC(0)
#define LEGOSENSOR_GET_MODE         _LEGOSENSORIOC(1)
#define LEGOSENSOR_GET_MODE_INFO    _LEGOSENSORIOC(2)
#define LEGOSENSOR_GET_DATA         _LEGOSENSORIOC(3)
```

---

## 6. Implementation Order

```
Phase 1a: Power Management     ← Done
Phase 1b: DCM Port Detection   ← Next task
Phase 1c: LUMP UART            ← Can parallel with 1b
Phase 1d: H-Bridge Motor       ← After 1c
    ↓
Phase 2a: Sensor Readout       ← After 1c
Phase 2b: TLC5955 LED          ← Independent
Phase 2c: IMU                  ← Independent (I2C2)
    ↓
Phase 3a: W25Q256 Flash        ← Independent
    ↓
Phase 4a: Bluetooth            ← Future
```

---

## 7. Verification

### Phase 1 Verification

```bash
# Verify on Discovery Kit (requires jig to connect PU devices)

# Port detection
nsh> cat /dev/legoport0
Type: 65 (SPIKE Large Motor)

# Motor control
nsh> echo 500 > /dev/legomotor0   # 50% forward
nsh> echo 0 > /dev/legomotor0     # stop (brake)
nsh> echo -500 > /dev/legomotor0  # 50% reverse
```

### Phase 2 Verification

```bash
# Sensor readout
nsh> cat /dev/legosensor0
Color: 3 (Blue)

# LED control
nsh> echo "1,0,0" > /dev/leds     # red display
```
