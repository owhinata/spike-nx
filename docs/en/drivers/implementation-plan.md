# Device Driver Implementation Plan

## 1. Overview

Implementation plan for device drivers to control hardware on the SPIKE Prime Hub from NuttX. Based on bringup investigation results, implementation proceeds in stages.

## 2. Driver List and Priority

| # | Driver | Device Path | Priority | Dependency |
|---|---|---|---|---|
| 1 | I/O Port Detection (DCM) | `/dev/legoport[0-5]` | **P0** | GPIO |
| 2 | LUMP UART Protocol | (internal) | **P0** | UART4/5/7/8/9/10 |
| 3 | H-Bridge Motor Control | `/dev/legomotor[N]` | **P0** | TIM1/3/4 PWM |
| 4 | Sensor Data Reading | `/dev/legosensor[N]` | **P1** | LUMP |
| 5 | TLC5955 LED Driver | `/dev/leds` | **Done** | SPI1 |
| 6 | IMU (LSM6DS3TR-C) | `/dev/imu0` | **Done** | I2C2 |
| 7 | DAC Audio | `/dev/dac0` | **P1** | DAC1 + TIM6 |
| 8 | ADC Battery Monitoring | `/dev/adc0` | **P1** | ADC1 (6ch) + DMA2 |
| 9 | USB CDC/ACM Console | `/dev/ttyACM0` | **Done** | OTG FS |
| 10 | W25Q256 SPI Flash | `/dev/mtdblock0` | **P2** | SPI2 |
| 11 | Power Management | (board init) | **Done** | PA13/PA14 GPIO |
| 12 | MP2639A Charge Control | (board internal) | **P2** | TIM5 PWM + ADC |
| 13 | Bluetooth (CC256x) | -- | **P3** | USART2 + DMA |

### Priority Definitions

- **P0**: Essential for minimum motor/sensor operation
- **P1**: Needed for basic robot operation
- **P2**: Useful for data logging and firmware updates
- **P3**: Wireless communication (future)

## 3. Implementation Phases

### Phase 1: Port Foundation (P0)

**Goal**: Device detection on I/O ports and basic motor control

#### 1a. Power Management (Done)

PA13 (BAT_PWR_EN) and PA14 (PORT_3V3_EN) initialization is already implemented in `stm32_boot.c`.

#### 1b. I/O Port Detection (DCM)

- Device Connection Manager for 6 ports
- 2ms periodic GPIO polling for passive device detection
- Stable detection (20 consecutive matches = approx. 400ms) to confirm device type
- Polling implemented via NuttX HPWORK queue

#### 1c. LUMP UART Protocol

- Implementation of LEGO UART Messaging Protocol
- Sync phase: 2400 baud -> device mode info acquisition -> switch to 115200 baud
- Data phase: periodic data reception + keep-alive (200ms)
- State machine driven by dedicated kernel thread

#### 1d. H-Bridge Motor Control

- PWM duty setting + direction control via GPIO/AF mode switching
- 4 states: Coast / Brake / Forward / Reverse
- Duty range: -1000 to +1000

### Phase 2: Sensors and LEDs (P1)

#### 2a. Sensor Data Reading

- Expose sensor data received during LUMP data phase via `/dev/legosensor[N]`
- Mode switching via ioctl, data retrieval via read()
- Supported: Color Sensor (Type 61), Ultrasonic Sensor (Type 62), Force Sensor (Type 63)

#### 2b. TLC5955 LED Driver

- Control 48ch 16-bit PWM LED driver via SPI1
- 5x5 LED matrix (48ch used out of RGB x 25 = 75ch)

#### 2c. IMU (LSM6DS3TR-C)

- Via I2C2, address 0x6A, INT1 DRDY interrupt
- Axis sign correction: X=-1, Y=+1, Z=-1

#### 2d. DAC Audio

- Analog audio signal output via DAC1 CH1 (PA4)
- Speaker enable: PC10 (GPIO HIGH)
- Sample rate control via TIM6 TRGO, buffer transfer via DMA1_Stream5

#### 2e. ADC Battery Monitoring and Button Input

- ADC1 6 channels + DMA2_Stream0 for periodic sampling
- TIM2 trigger to start ADC conversion

| Channel | Pin | Purpose |
|---|---|---|
| CH10 | PC0 | Battery current (max 7300mA) |
| CH11 | PC1 | Battery voltage (max 9900mV) |
| CH8 | PB0 | Battery temperature (NTC thermistor) |
| CH3 | PA3 | USB charge input current |
| CH14 | PC4 | Charge status (resistor ladder) |
| CH1 | PA1 | Button pad (multiple buttons detected via resistor ladder) |

**Note**: Hub button input uses an ADC resistor ladder, not GPIO. The pressed button is determined from the voltage level on PA1.

### Phase 3: Storage and Charging (P2)

#### 3a. W25Q256 SPI NOR Flash

- Access 32MB SPI NOR Flash via SPI2
- 4-byte address mode
- Uses NuttX MTD driver (`CONFIG_MTD_W25`)
- Formatted with LittleFS or SmartFS

#### 3b. MP2639A Charge Control

- Charge current setting: PWM control via TIM5 CH1 (PA0) (96kHz)
- Charge status monitoring: 8-level detection via ADC resistor ladder (CH14/PC4)

### Phase 4: Wireless Communication (P3)

#### 4a. Bluetooth (CC256x)

- USART2 (PD5/PD6) + flow control (PD3/PD4)
- TX/RX via DMA1_Stream6/7
- Details TBD

## 4. Port GPIO Pin Assignment

| Port | UART TX | UART RX | AF | GPIO1 | GPIO2 | UART BUF |
|---|---|---|---|---|---|---|
| A | PE8 | PE7 | AF8 (UART7) | PA5 | PA3 | PB2 |
| B | PD1 | PD0 | AF11 (UART4) | PA4 | PA6 | PD3 |
| C | PE1 | PE0 | AF8 (UART8) | PB0 | PB14 | PD4 |
| D | PC12 | PD2 | AF8 (UART5) | PB4 | PB15 | PD7 |
| E | PE3 | PE2 | AF11 (UART10) | PC13 | PE12 | PB5 |
| F | PD15 | PD14 | AF11 (UART9) | PC14 | PE6 | PB10 |

## 5. DCM Detection Algorithm

```
1. Read GPIO2 while switching GPIO1 between OUTPUT HIGH / LOW
2. Determine device category from GPIO2 response pattern:
   - With resistor -> Passive device (lights, external motors, etc.)
   - Pull-up -> UART device (smart sensors/motors)
   - No response -> Not connected
3. For UART devices:
   - Switch pins from GPIO to UART AF
   - Set UART BUF pin HIGH (enable RS485 transceiver)
   - Begin LUMP handshake
```

## 6. ioctl Interface

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

## 7. Directory Structure

```
boards/spike-prime-hub/src/
  stm32_boot.c          # Power initialization (PA13/PA14)
  stm32_bringup.c       # Device driver registration
  stm32_usbdev.c        # USB CDC/ACM
  stm32_legoport.c      # I/O Port Manager (DCM)
  stm32_legomotor.c     # H-Bridge motor control
  stm32_tlc5955.c       # TLC5955 LED driver
  stm32_lsm6ds3.c       # IMU (LSM6DS3TR-C) initialization
  stm32_audio.c         # DAC audio + speaker control
  stm32_battery.c       # ADC battery monitoring + button input
  stm32_charger.c       # MP2639A charge control

drivers/lego/           # NuttX generic drivers
  lump_uart.c           # LUMP UART protocol engine
  lump_uart.h
  legodev.h             # Device type definitions
  legomotor.c           # Motor upper driver
  legosensor.c          # Sensor upper driver
```

## 8. Implementation Order

```
Phase 1a: Power Management          (Done)
Phase 1b: DCM Port Detection        (Next task)
Phase 1c: LUMP UART                 (Can be parallel with 1b)
Phase 1d: H-Bridge Motor            (After 1c completion)
    |
Phase 2a: Sensor Reading            (After 1c completion)
Phase 2b: TLC5955 LED               (Done)
Phase 2c: IMU                       (Done)
Phase 2d: DAC Audio                 (Can be implemented independently, DAC1 + TIM6)
Phase 2e: ADC Battery               (Can be implemented independently, ADC1 + DMA2)
    |
Phase 3a: W25Q256 Flash             (Can be implemented independently)
Phase 3b: MP2639A Charging           (After 2e ADC completion)
    |
Phase 4a: Bluetooth                 (Future)
```
