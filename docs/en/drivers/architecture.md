# Device Driver Architecture

## 1. Layered Architecture

```
                    User Space
  +---------------------------------------------+
  |  /dev/legoport[0-5]    (Port Manager)        |
  |  /dev/legosensor[N]    (Sensor Data)         |
  |  /dev/legomotor[N]     (Motor Control)       |
  +--------------+---------------+---------------+
                 |               |
             Kernel Space        |
  +--------------+---------------+---------------+
  |  LEGO Port Manager (Upper Layer)             |
  |  - Per-port state machine thread             |
  |  - DCM passive detection (2ms polling)       |
  |  - Dynamic UART init/release                 |
  |  - Device type notification                  |
  +---------------------------------------------+
  |  LUMP UART Protocol Engine (Middle Layer)    |
  |  - Synchronization handshake                 |
  |  - Mode information parsing                  |
  |  - Data send/receive                         |
  |  - Keep-alive management                     |
  +---------------------------------------------+
  |  STM32 UART + GPIO (Lower Layer / HAL)      |
  |  - stm32_configgpio() pin mode switching     |
  |  - stm32_serial UART communication           |
  |  - HPWORK queue (2ms DCM polling)            |
  +---------------------------------------------+
```

## 2. Device Node Design

### Persistent Devices (Registered at Boot)

| Device | Path | Purpose |
|---|---|---|
| Port Manager | `/dev/legoport0` through `5` | Port state management, device detection notification |

### Dynamic Devices (Registered on Device Connection)

| Device | Path | Registration Condition |
|---|---|---|
| Sensor | `/dev/legosensor0` and up | Non-motor device detected via LUMP |
| Motor | `/dev/legomotor0` and up | Motor device detected via LUMP |

Dynamically removed with `unregister_driver()` on disconnection.

## 3. Hot-Plug Lifecycle

```
Boot:
  Register /dev/legoport[N]
  Start DCM polling (HPWORK, 2ms)

Connection Detected:
  DCM detects stable type (20 consecutive matches, ~400ms)
  +- UART device:
  |   Switch pins from GPIO to UART AF (stm32_configgpio)
  |   Dynamically initialize UART driver
  |   Execute LUMP synchronization handshake
  |   On success -> register /dev/legosensor[N] or /dev/legomotor[N]
  |   Start data receive loop
  +- Passive device:
      Register device node immediately

Disconnection Detected:
  UART: 600ms data timeout
  Remove device node with unregister_driver()
  Reset UART, release serial driver
  Restore pins to GPIO mode
  Resume DCM polling
```

## 4. UART Ownership Strategy

### Challenge

UART pins are used as GPIO during device detection and as UART AF during communication. This can conflict with the NuttX serial driver.

### Policy

1. **Do not initialize serial drivers for I/O port UARTs at boot**
2. Port manager owns the pins. During DCM, operate as GPIO via `stm32_configgpio()`
3. After UART device detection, switch to AF mode via `stm32_configgpio()` and initialize a lightweight UART driver
4. The LUMP engine requires low-level UART access (byte-level TX/RX, baud rate switching), so it uses a thin wrapper around STM32 UART registers

### GPIO <-> UART Dynamic Switching

```c
// GPIO input mode (during device detection)
stm32_configgpio(GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN8);

// UART AF mode (during UART device communication)
stm32_configgpio(GPIO_AF8 | GPIO_PORTE | GPIO_PIN8);  // UART7_TX
```

## 5. Thread Model

| Component | Method | Rationale |
|---|---|---|
| DCM Polling | HPWORK (2ms) | Simple GPIO operations, no dedicated thread needed |
| LUMP Protocol | Dedicated kernel thread | Complex state machine + timing requirements (100ms keepalive, 250ms IO timeout) |

## 6. Data Format

Data format is self-described during the LUMP protocol synchronization phase:

| Field | Description |
|---|---|
| `num_values` | Number of data elements (int8: 1-32, int16: 1-16, int32/float: 1-8) |
| `data_type` | int8 / int16 / int32 / float (little-endian) |
| `raw_min/max` | Raw data range |
| `pct_min/max` | Percentage range |
| `si_min/max` | SI unit range |
| `units` | Unit string |

Maximum payload: 32 bytes/mode.

The NuttX driver exposes this metadata to user space via `ioctl(LEGOSENSOR_GET_MODE_INFO)`, allowing applications to correctly interpret raw data.

## 7. ioctl Interface

### Port Manager (`/dev/legoport[N]`)

```c
#define LEGOPORT_GET_DEVICE_TYPE    _LEGOPORTIOC(0)   // Get connected device type
#define LEGOPORT_GET_DEVICE_INFO    _LEGOPORTIOC(1)   // Number of modes, flags
#define LEGOPORT_WAIT_CONNECT       _LEGOPORTIOC(2)   // Wait for device connection
#define LEGOPORT_WAIT_DISCONNECT    _LEGOPORTIOC(3)   // Wait for device disconnection
```

### Sensor (`/dev/legosensor[N]`)

```c
#define LEGOSENSOR_SET_MODE         _LEGOSENSORIOC(0) // Switch mode
#define LEGOSENSOR_GET_MODE         _LEGOSENSORIOC(1) // Get current mode
#define LEGOSENSOR_GET_MODE_INFO    _LEGOSENSORIOC(2) // Mode info (value count, type, name)
#define LEGOSENSOR_GET_DATA         _LEGOSENSORIOC(3) // Get raw data
```

Data for the current mode can also be obtained via `read()` (up to 32 bytes).

### Motor (`/dev/legomotor[N]`)

```c
#define LEGOMOTOR_SET_DUTY          _LEGOMOTORIOC(0)  // int16: -1000 to +1000
#define LEGOMOTOR_COAST             _LEGOMOTORIOC(1)  // Coast
#define LEGOMOTOR_BRAKE             _LEGOMOTORIOC(2)  // Brake
#define LEGOMOTOR_GET_POSITION      _LEGOMOTORIOC(3)  // int32: degrees
#define LEGOMOTOR_GET_SPEED         _LEGOMOTORIOC(4)  // int16: deg/s
#define LEGOMOTOR_GET_ABS_POS       _LEGOMOTORIOC(5)  // int16: absolute position
#define LEGOMOTOR_RESET_POS         _LEGOMOTORIOC(6)  // Reset position
```

## 8. SPIKE Sensor Details

### Color Sensor (Type 61)

| Mode | Name | Data | Direction |
|---|---|---|---|
| 0 | COLOR | 1x int8 | Read: detected color index |
| 1 | REFLT | 1x int8 | Read: reflected light intensity |
| 2 | AMBI | 1x int8 | Read: ambient light intensity |
| 3 | LIGHT | 3x int8 | **Write**: LED brightness setting |
| 5 | RGB_I | 4x int16 | Read: RGBI values |
| 6 | HSV | 3x int16 | Read: HSV values |

### Ultrasonic Sensor (Type 62)

| Mode | Name | Data | Direction |
|---|---|---|---|
| 0 | DISTL | 1x int16 | Read: distance (long range), mm |
| 1 | DISTS | 1x int16 | Read: distance (short range), mm |
| 3 | LISTN | 1x int8 | Read: ultrasonic listening |
| 5 | LIGHT | 4x int8 | **Write**: LED brightness setting |

### Force Sensor (Type 63)

| Mode | Name | Data | Direction |
|---|---|---|---|
| 4 | FRAW | 1x int16 | Read: raw force sensor value |

## 9. Reference Files

- `pybricks/lib/pbio/drv/legodev/legodev_pup.c` -- Port-level driver
- `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c` -- LUMP UART protocol (1265 lines)
- `pybricks/lib/pbio/drv/legodev/legodev_spec.c` -- Device-specific knowledge
- `pybricks/lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c` -- H-bridge driver
- `pybricks/lib/pbio/include/pbdrv/legodev.h` -- Device type definitions, API
- `pybricks/lib/lego/lego_uart.h` -- LUMP protocol definitions
