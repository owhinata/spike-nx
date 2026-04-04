# Device Driver Architecture

## 1. Overall Architecture

```
                    User Space
  ┌─────────────────────────────────────────────┐
  │  /dev/legoport[0-5]    (port manager)       │
  │  /dev/legosensor[N]    (sensor data)        │
  │  /dev/legomotor[N]     (motor control)      │
  └────────────┬───────────────┬────────────────┘
               │               │
           Kernel Space        │
  ┌────────────┴───────────────┴────────────────┐
  │  LEGO Port Manager (upper layer)            │
  │  - Per-port state machine thread            │
  │  - DCM passive detection (2ms polling)      │
  │  - Dynamic UART init/deinit                 │
  │  - Device type notification                 │
  ├─────────────────────────────────────────────┤
  │  LUMP UART Protocol Engine (middle layer)   │
  │  - Sync handshake                           │
  │  - Mode info parsing                        │
  │  - Data send/receive                        │
  │  - Keepalive management                     │
  ├─────────────────────────────────────────────┤
  │  STM32 UART + GPIO (lower layer / HAL)      │
  │  - stm32_configgpio() pin mode switching    │
  │  - stm32_serial UART communication          │
  │  - HPWORK queue (2ms DCM polling)           │
  └─────────────────────────────────────────────┘
```

---

## 2. Device Node Design

### Persistent Devices (Registered at Boot)

| Device | Path | Purpose |
|---|---|---|
| Port manager | `/dev/legoport0` through `5` | Port state management, device detection notification |

### Dynamic Devices (Registered on Device Connect)

| Device | Path | Registration Condition |
|---|---|---|
| Sensor | `/dev/legosensor0` onward | Non-motor LUMP device detected |
| Motor | `/dev/legomotor0` onward | Motor LUMP device detected |

Dynamically removed via `unregister_driver()` on disconnect.

---

## 3. SPIKE Sensor Details

### Color Sensor (Type 61)

| Mode | Name | Data | Direction |
|---|---|---|---|
| 0 | COLOR | 1×int8 | Read: detected color index |
| 1 | REFLT | 1×int8 | Read: reflected light intensity |
| 2 | AMBI | 1×int8 | Read: ambient light intensity |
| 3 | LIGHT | 3×int8 | **Write**: set LED brightness |
| 5 | RGB_I | 4×int16 | Read: RGBI values |
| 6 | HSV | 3×int16 | Read: HSV values |

Stale data delay: 30ms. Requires power supply (Pin1).

### Ultrasonic Sensor (Type 62)

| Mode | Name | Data | Direction |
|---|---|---|---|
| 0 | DISTL | 1×int16 | Read: distance (long range), mm |
| 1 | DISTS | 1×int16 | Read: distance (short range), mm |
| 3 | LISTN | 1×int8 | Read: ultrasonic listening |
| 5 | LIGHT | 4×int8 | **Write**: set LED brightness |

Stale data delay: 50ms. Requires power supply (Pin1).

### Force Sensor (Type 63)

| Mode | Name | Data | Direction |
|---|---|---|---|
| 4 | FRAW | 1×int16 | Read: raw force value |

No special requirements.

---

## 4. ioctl Interface

### Port Manager

```c
#define LEGOPORT_GET_DEVICE_TYPE    _LEGOPORTIOC(0)   // Get connected device type
#define LEGOPORT_GET_DEVICE_INFO    _LEGOPORTIOC(1)   // Mode count, flags
#define LEGOPORT_WAIT_CONNECT       _LEGOPORTIOC(2)   // Block until device connects
#define LEGOPORT_WAIT_DISCONNECT    _LEGOPORTIOC(3)   // Block until device disconnects
```

### Sensor

```c
#define LEGOSENSOR_SET_MODE         _LEGOSENSORIOC(0) // Switch mode
#define LEGOSENSOR_GET_MODE         _LEGOSENSORIOC(1) // Get current mode
#define LEGOSENSOR_GET_MODE_INFO    _LEGOSENSORIOC(2) // Mode info (num_values, type, name)
#define LEGOSENSOR_GET_DATA         _LEGOSENSORIOC(3) // Get raw data
```

`read()` also returns current mode data (max 32 bytes).

### Motor

See [14-motor-control-driver.md](14-motor-control-driver.md).

---

## 5. Hot-Plug Lifecycle

```
Boot:
  Register /dev/legoport[N]
  Start DCM polling (HPWORK, 2ms)

Connect Detection:
  DCM detects stable type (20 consecutive matches, ~400ms)
  ├─ UART device:
  │   Switch pins GPIO → UART AF (stm32_configgpio)
  │   Dynamically init UART driver
  │   Run LUMP sync handshake
  │   Success → register /dev/legosensor[N] or /dev/legomotor[N]
  │   Begin data receive loop
  └─ Passive device:
      Register device node immediately

Disconnect Detection:
  UART: 600ms data timeout
  unregister_driver() to remove device node
  Reset UART, deinit serial driver
  Switch pins back to GPIO mode
  Resume DCM polling
```

---

## 6. UART Ownership Strategy

### Challenge

UART pins are used as GPIO during detection and as UART AF during communication. Potential conflict with NuttX serial driver.

### Recommended Approach

1. **Do NOT init I/O port UART serial drivers at boot**
2. Port manager owns the pins. During DCM, uses `stm32_configgpio()` for GPIO
3. After UART device detection, switch to AF mode and init lightweight UART driver
4. LUMP engine needs low-level UART access (byte-level TX/RX, baud rate switch) → thin wrapper around STM32 UART registers

### Thread vs Work Queue

| Component | Approach | Reason |
|---|---|---|
| DCM polling | HPWORK (2ms) | Simple GPIO ops, no dedicated thread needed |
| LUMP protocol | Dedicated kernel thread | Complex state machine + timing (100ms keepalive, 250ms IO timeout) |

---

## 7. Data Format Handling

LUMP sync phase self-describes data formats:

| Field | Description |
|---|---|
| `num_values` | Element count (int8: 1-32, int16: 1-16, int32/float: 1-8) |
| `data_type` | int8 / int16 / int32 / float (little-endian) |
| `raw_min/max` | Raw data range |
| `pct_min/max` | Percentage range |
| `si_min/max` | SI unit range |
| `units` | Unit string |

Max payload: 32 bytes per mode.

NuttX driver exposes metadata via `ioctl(LEGOSENSOR_GET_MODE_INFO)` for correct data interpretation.

---

## 8. Reference Files

- `pybricks/lib/pbio/drv/legodev/legodev_pup.c` — Port-level driver
- `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c` — LUMP UART protocol (1265 lines)
- `pybricks/lib/pbio/drv/legodev/legodev_spec.c` — Device-specific knowledge
- `pybricks/lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c` — H-bridge driver
- `pybricks/lib/pbio/include/pbdrv/legodev.h` — Device type definitions, API
- `pybricks/lib/lego/lego_uart.h` — LUMP protocol definitions
