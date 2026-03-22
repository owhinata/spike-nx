# I/O Port Device Detection Mechanism

## 1. Overview

The SPIKE Prime Hub's 6 I/O ports **auto-detect** connected devices (motors/sensors) using a two-phase approach:

1. **Passive detection** (resistor-based, ~400ms): Identify non-UART devices by GPIO voltage patterns
2. **UART detection** (LUMP protocol): Identify smart devices via UART communication when passive detection fails

---

## 2. Hardware Configuration

### Port Pin Structure

Each port has 5 signal lines:

| Signal | Connector Pin | Function |
|---|---|---|
| gpio1 | Pin 5 (input path) | Read ID1 |
| gpio2 | Pin 6 (input/output) | Read/drive ID2 |
| uart_tx | Pin 5 (output path) | Drive ID1 / UART TX |
| uart_rx | Pin 6 (input path) | Read ID2 / UART RX |
| uart_buf | — | Buffer enable (Active Low) |

`uart_tx` and `gpio1` connect to the same physical pin (Pin 5) via a buffer. Same for `uart_rx` and `gpio2` (Pin 6).

### Per-Port Pin Mapping

| Port | UART | uart_tx | uart_rx | gpio1 | gpio2 | uart_buf | AF |
|---|---|---|---|---|---|---|---|
| A | UART7 | PE8 | PE7 | PD7 | PD8 | PA10 | AF8 |
| B | UART4 | PD1 | PD0 | PD9 | PD10 | PA8 | AF11 |
| C | UART8 | PE1 | PE0 | PD11 | PE4 | PE5 | AF8 |
| D | UART5 | PC12 | PD2 | PC15 | PC14 | PB2 | AF8 |
| E | UART10 | PE3 | PE2 | PC13 | PE12 | PB5 | AF11 |
| F | UART9 | PD15 | PD14 | PC11 | PE6 | PC5 | AF11 |

**Note**: Ports E and F use UART10/UART9 (STM32F413-specific).

### Power Control

- **Port VCC (3.3V)**: Controlled via PA14 for all ports (`pbdrv_ioport_enable_vcc()`)
- VCC is OFF at boot → enabled when device detection starts
- All GPIO pull-ups/pull-downs disabled for accurate resistor detection

---

## 3. Passive Detection State Machine (DCM)

### ID Group Classification

GPIO voltage patterns classified into 4 groups:

| Group | Value | State |
|---|---|---|
| GND | 0 | Shorted to ground |
| VCC | 1 | Connected to VCC (3.3V) |
| PULL_DOWN | 2 | Via pull-down resistor |
| OPEN | 3 | Floating (not connected) |

### Detection Flow (2ms polling)

```
Step 1: Drive ID1 HIGH (uart_tx=HIGH, uart_buf=LOW), set ID2 as input
  ↓ YIELD (2ms)
Step 2: Read ID2 → prev_value. Drive ID1 LOW
  ↓ YIELD
Step 3: Read ID2 → cur_value
  ├─ (1→0): Touch sensor (TOUCH)
  ├─ (0→1): Train point (TPOINT)
  └─ No change → Step 4
  ↓
Step 4-5: Determine ID1 group (VCC/GND/PULL_DOWN/OPEN)
  ↓ YIELD
Step 6-8: Test ID2 side similarly
  ├─ ID1=OPEN and (1→0): 3_PART
  ├─ (0→1): EXPLOD
  └─ No change → Step 9
  ↓
Step 9-10: Final determination
  ├─ ID1_group < OPEN → lookup[ID1_group][ID2_group]
  └─ ID1_group = OPEN → UNKNOWN_UART (proceed to UART detection)
```

### Resistor Matrix Lookup Table

| | ID2=GND | ID2=VCC | ID2=PULL_DOWN |
|---|---|---|---|
| **ID1=GND** | POWER (4) | TURN (3) | LIGHT2 (10) |
| **ID1=VCC** | TRAIN (2) | LMOTOR (6) | LIGHT1 (9) |
| **ID1=PULL_DOWN** | MMOTOR (1) | XMOTOR (7) | LIGHT (8) |

- **ID1=OPEN**: UART device signature → enter UART detection phase

### Debounce

Same type must be detected **20 consecutive times** (`AFFIRMATIVE_MATCH_COUNT = 20`). At 2ms polling × ~10 yields/cycle × 20 = **~400ms** to confirm.

---

## 4. UART Detection (LUMP Protocol)

### GPIO → UART Mode Switch

When `UNKNOWN_UART` is confirmed, pins switch from GPIO to UART AF mode:

```c
pbdrv_gpio_alt(&pins->uart_rx, pins->uart_alt);  // Pin 6 → UART RX AF
pbdrv_gpio_alt(&pins->uart_tx, pins->uart_alt);  // Pin 5 → UART TX AF
pbdrv_gpio_out_low(&pins->uart_buf);              // Enable buffer
```

In NuttX, `stm32_configgpio()` can be called at runtime for the same effect.

### LUMP Message Format

Header byte (1 byte):
```
Bit [7:6] = Message Type (SYS=0, CMD=1, INFO=2, DATA=3)
Bit [5:3] = Payload Size (0=1B, 1=2B, 2=4B, 3=8B, 4=16B, 5=32B)
Bit [2:0] = Command/Mode number (0-7)
```

- **SYS**: Header only (SYNC=0x00, NACK=0x02, ACK=0x04)
- **CMD/DATA**: Header + payload + checksum (XOR)
- **INFO**: Header + info_type + payload + checksum

### Handshake Sequence

```
Phase 1: Speed Negotiation
  Hub → Device: CMD SPEED 115200 (at 115200 baud)
  Device → Hub: ACK (success) or timeout (→ 2400 baud EV3 mode)

Phase 2: Synchronization
  Device → Hub: CMD TYPE <type_id> (device identification)
  Up to 10 retries

Phase 3: Mode Information
  Device → Hub:
    CMD MODES <num_modes>
    CMD SPEED <baud_rate>
    CMD VERSION <fw_ver> <hw_ver>
    Per mode:
      INFO NAME <name>
      INFO RAW/PCT/SI <min> <max>
      INFO FORMAT <num_values> <type> <digits> <decimals>
    SYS ACK (end of info)

Phase 4: Acknowledge + Baud Switch
  Hub → Device: SYS ACK
  Wait 10ms
  Hub: Switch to new baud rate

Phase 5: Normal Operation
  Hub → Device: SYS NACK (keep-alive every 100ms)
  Hub → Device: CMD SELECT <mode> (mode switch)
  Device → Hub: DATA <payload> (sensor data)
  Disconnect detected after 600ms without data
```

---

## 5. Device Type List

### Passive Devices (Resistor Matrix)

| ID | Name | Description |
|---|---|---|
| 0 | NONE | No device |
| 1 | LPF2_MMOTOR | WeDo 2.0 Medium Motor |
| 2 | LPF2_TRAIN | Train Motor |
| 5 | LPF2_TOUCH | Touch Sensor |
| 8 | LPF2_LIGHT | Powered Up Lights |

### UART Devices (LUMP Protocol)

| ID | Name |
|---|---|
| 29 | EV3 Color Sensor |
| 30 | EV3 Ultrasonic Sensor |
| 32 | EV3 Gyro Sensor |
| 33 | EV3 IR Sensor |
| 34 | WeDo 2.0 Tilt Sensor |
| 35 | WeDo 2.0 Motion Sensor |
| 37 | BOOST Color and Distance Sensor |
| 38 | BOOST Interactive Motor |
| 46 | Technic Large Motor |
| 47 | Technic XL Motor |
| 48 | SPIKE Medium Motor |
| 49 | SPIKE Large Motor |
| 61 | SPIKE Color Sensor |
| 62 | SPIKE Ultrasonic Sensor |
| 63 | SPIKE Force Sensor |
| 64 | Technic Color Light Matrix |
| 65 | SPIKE Small Motor |
| 75 | Technic Medium Angular Motor |
| 76 | Technic Large Angular Motor |

---

## 6. NuttX Driver Design

### Dynamic GPIO ↔ UART Switching

`stm32_configgpio()` can be called at runtime:

```c
// GPIO input mode (device detection)
stm32_configgpio(GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN8);

// UART AF mode (UART device communication)
stm32_configgpio(GPIO_AF8 | GPIO_PORTE | GPIO_PIN8);  // UART7_TX
```

### Recommended Architecture: Character Device

```
/dev/legoport0 through /dev/legoport5
```

- `ioctl()` for: get device type, set mode, read/write sensor data
- Kernel thread or HP work queue for DCM polling (2ms)
- On UART device detection: switch pins to AF mode, dynamically init NuttX serial driver
- On disconnect: revert to GPIO mode, restart DCM polling

### Implementation Challenges

1. **UART ownership**: UART pins used as GPIO during detection. Don't init UART driver at boot; init dynamically after detection
2. **2ms timing**: `work_queue(HPWORK, ...)` provides sufficient resolution
3. **Baud rate switching**: LUMP requires dynamic 115200 → 460800 switching
4. **Hot-plug**: Detect → UART session → disconnect → re-detect loop as persistent thread

### References

- [pybricks/technical-info UART Protocol](https://github.com/pybricks/technical-info/blob/master/uart-protocol.md)
- [ev3dev UART Sensor Protocol](http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html)
- [Biased Logic - LEGO Powered Up Connector](https://www.biasedlogic.com/index.php/lego-powered-up-connector/)
