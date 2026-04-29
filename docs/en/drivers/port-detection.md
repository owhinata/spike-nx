# I/O Port Device Detection

## 1. Overview

The 6 I/O ports on the SPIKE Prime Hub automatically detect connected devices (motors/sensors). Detection is performed in two stages:

1. **Passive detection** (resistance-based, ~400ms): Identifies non-UART devices by voltage patterns on GPIO pins
2. **UART detection** (LUMP protocol): Identifies smart devices via UART communication when passive detection is inconclusive

## 2. Hardware Configuration

### Port Pin Configuration

Each port has 5 signal lines:

| Signal | Connector Pin | Function |
|---|---|---|
| gpio1 | Pin 5 (input path) | ID1 read |
| gpio2 | Pin 6 (input/output) | ID2 read/drive |
| uart_tx | Pin 5 (output path) | ID1 drive / UART TX |
| uart_rx | Pin 6 (input path) | ID2 read / UART RX |
| uart_buf | -- | Buffer enable (Active Low) |

`uart_tx` and `gpio1` are connected to the same physical pin (Pin 5) via a buffer. `uart_rx` and `gpio2` are likewise connected (Pin 6).

### Per-Port Pin Mapping

| Port | UART | uart_tx | uart_rx | gpio1 | gpio2 | uart_buf | AF |
|---|---|---|---|---|---|---|---|
| A | UART7 | PE8 | PE7 | PD7 | PD8 | PA10 | AF8 |
| B | UART4 | PD1 | PD0 | PD9 | PD10 | PA8 | AF11 |
| C | UART8 | PE1 | PE0 | PD11 | PE4 | PE5 | AF8 |
| D | UART5 | PC12 | PD2 | PC15 | PC14 | PB2 | AF8 |
| E | UART10 | PE3 | PE2 | PC13 | PE12 | PB5 | AF11 |
| F | UART9 | PD15 | PD14 | PC11 | PE6 | PC5 | AF11 |

**Note**: Ports E and F use UART10/UART9 (STM32F413 specific).

### Power Control

- **Port VCC (3.3V)**: Controlled for all ports via PA14
- VCC is OFF at boot -> turned ON when device detection starts
- All GPIO pull-up/pull-down resistors are disabled (for accurate resistance detection)

## 3. Passive Detection State Machine (DCM)

### ID Group Classification

GPIO pin voltage patterns are classified into 4 groups:

| Group | Value | State |
|---|---|---|
| GND | 0 | Shorted to GND |
| VCC | 1 | Connected to VCC (3.3V) |
| PULL_DOWN | 2 | Via pull-down resistor |
| OPEN | 3 | Floating (not connected) |

### Detection Flow (2ms Polling)

```
Step 1: Drive ID1 HIGH (uart_tx=HIGH, uart_buf=LOW), set ID2 as input
  | YIELD (2ms)
Step 2: Read ID2 -> prev_value. Drive ID1 LOW
  | YIELD
Step 3: Read ID2 -> cur_value
  |- (1->0): Touch sensor detected (TOUCH)
  |- (0->1): Train point detected (TPOINT)
  +- No change -> proceed to Step 4
  |
Step 4-5: Determine ID1 group (VCC/GND/PULL_DOWN/OPEN)
  | YIELD
Step 6-8: Test ID2 side similarly
  |- ID1=OPEN and (1->0): 3_PART
  |- (0->1): EXPLOD
  +- No change -> proceed to Step 9
  |
Step 9-10: Final determination
  |- ID1_group < OPEN -> lookup[ID1_group][ID2_group]
  +- ID1_group = OPEN -> UNKNOWN_UART (proceed to UART detection)
```

### Resistance Matrix Lookup Table

| | ID2=GND | ID2=VCC | ID2=PULL_DOWN |
|---|---|---|---|
| **ID1=GND** | POWER (4) | TURN (3) | LIGHT2 (10) |
| **ID1=VCC** | TRAIN (2) | LMOTOR (6) | LIGHT1 (9) |
| **ID1=PULL_DOWN** | MMOTOR (1) | XMOTOR (7) | LIGHT (8) |

- **ID1=OPEN**: UART device signature -> proceed to UART detection phase

### Debounce

The type is confirmed only after **20 consecutive** detections of the same type (`AFFIRMATIVE_MATCH_COUNT = 20`). Polling 2ms x approx. 10 YIELDs/cycle x 20 iterations = **approx. 400ms** to confirm.

## 4. UART Detection (LUMP Protocol)

### GPIO to UART Mode Switch

When `UNKNOWN_UART` is confirmed, pins are switched from GPIO to UART AF mode:

```c
// Switching in NuttX
stm32_configgpio(GPIO_AF8 | GPIO_PORTE | GPIO_PIN7);   // Pin 6 -> UART RX AF
stm32_configgpio(GPIO_AF8 | GPIO_PORTE | GPIO_PIN8);   // Pin 5 -> UART TX AF
// Enable buffer (Active Low)
stm32_gpiowrite(uart_buf_pin, false);
```

### LUMP Message Format

Header byte (1 byte):
```
Bit [7:6] = Message type (SYS=0, CMD=1, INFO=2, DATA=3)
Bit [5:3] = Payload size (0=1B, 1=2B, 2=4B, 3=8B, 4=16B, 5=32B)
Bit [2:0] = Command/mode number (0-7)
```

- **SYS**: Header only (SYNC=0x00, NACK=0x02, ACK=0x04)
- **CMD/DATA**: Header + payload + checksum (XOR)
- **INFO**: Header + info_type + payload + checksum

### Handshake Sequence

```
Phase 1: Speed Negotiation
  Hub -> Device: CMD SPEED 115200 (115200 baud)
  Device -> Hub: ACK (success) or timeout (-> 2400 baud EV3 mode)

Phase 2: Synchronization
  Device -> Hub: CMD TYPE <type_id> (device type identification)
  Up to 10 retries

Phase 3: Mode Information Acquisition
  Device -> Hub:
    CMD MODES <num_modes>
    CMD SPEED <baud_rate>
    CMD VERSION <fw_ver> <hw_ver>
    For each mode:
      INFO NAME <name>
      INFO RAW/PCT/SI <min> <max>
      INFO FORMAT <num_values> <type> <digits> <decimals>
    SYS ACK (information transmission complete)

Phase 4: Acknowledgment + Baud Rate Switch
  Hub -> Device: SYS ACK
  Wait 10ms
  Hub: Switch to new baud rate (typically 460800 baud)

Phase 5: Normal Operation
  Hub -> Device: SYS NACK (keep-alive every 100ms)
  Hub -> Device: CMD SELECT <mode> (mode switch)
  Device -> Hub: DATA <payload> (sensor data)
  Disconnection detected after 600ms of no response
```

### LUMP State Machine

```
RESET -> SYNC_TYPE -> SYNC_MODES -> SYNC_DATA -> ACK -> DATA
  ^                                                  |
  +--------- Timeout / Error ------------------------+
```

## 5. Device Type List

### Passive Devices (Resistance Matrix Detection)

| ID | Name | Description |
|---|---|---|
| 0 | NONE | Not connected |
| 1 | LPF2_MMOTOR | WeDo 2.0 Medium Motor |
| 2 | LPF2_TRAIN | Train Motor |
| 5 | LPF2_TOUCH | Touch Sensor |
| 8 | LPF2_LIGHT | Powered Up Lights |

### UART Devices (LUMP Protocol Detection)

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

## 6. NuttX Implementation Considerations

1. **UART Ownership**: UART pins are used as GPIO during detection. UART drivers are not initialized at boot; they are dynamically initialized after device detection
2. **2ms Timing**: Achievable with `work_queue(HPWORK, ...)`
3. **Baud Rate Switching**: LUMP protocol requires dynamic switching such as 115200 -> 460800
4. **Hot-Plug**: Detection -> UART communication -> disconnection detection -> re-detection loop runs in a persistent thread

## 7. NuttX Implementation Status (Issue #42)

DCM (passive detection + UART handoff hook) is implemented (`feat/42-legoport-dcm` branch). The LUMP protocol itself is Issue #43.

### Implementation Files

- `boards/spike-prime-hub/include/board_legoport.h` — public ABI (ioctl numbers, type enum, structs)
- `boards/spike-prime-hub/src/stm32_legoport.c` — DCM state machine + handoff registry
- `boards/spike-prime-hub/src/stm32_legoport_chardev.c` — `/dev/legoport[0-5]` chardev shim
- `apps/legoport/legoport_main.c` — `legoport list / info / wait / stats` CLI

### Execution Model

- HPWORK runs at 2 ms cadence; one yield boundary = one state advance (faithful port of pybricks `PT_YIELD(pt)`)
- Non-yield state transitions fall through within a single invocation (no tick consumed)
- Path lengths: TPOINT=3 yields, TOUCH=4, OPEN/PULL × VCC/PULL_DOWN=10 (worst); debounce 20 consecutive ≈ 400 ms worst case.

### Type IDs

Public ABI preserves pybricks `PBDRV_LEGODEV_TYPE_ID_LPF2_*` values:

- 1=MMOTOR, 2=TRAIN, 3=TURN, 4=POWER, 5=TOUCH, 6=LMOTOR, 7=XMOTOR
- 8=LIGHT, 9=LIGHT1, 10=LIGHT2, 11=TPOINT, 12=EXPLOD, 13=3_PART
- **14=UNKNOWN_UART** (UART device confirmed; latched until LUMP identifies it)

### Handoff API (#43 integration)

```c
int stm32_legoport_register_uart_handoff(int port, legoport_uart_handoff_cb_t cb, void *priv);
int stm32_legoport_release_uart(int port);
```

- After `register_uart_handoff`, when DCM confirms `UNKNOWN_UART` it invokes the CB.  The CB owns: switching `uart_tx`/`uart_rx` to UART AF, asserting `uart_buf` low, starting the LUMP engine.
- Protection: `handoff_generation` guards against stale results; `handoff_inflight` + drain wait pins `priv` lifetime.
- **Owner contract**: once the CB returns OK, #43 **MUST** call `release_uart(port)` on every exit path.  There is no DCM-side watchdog; a leaked owner latches the port until reboot.
- **CB non-reentrancy**: the CB MUST NOT call `register/release` for the same port — that deadlocks on the inflight drain.

### `WAIT_CONNECT/DISCONNECT` Semantics

`event_counter` (uint32_t monotonic) plus a per-fd snapshot lets WAIT detect edges without missing notifications even with single-open semantics.

### UNOWNED Rescan

When no CB is registered or a CB fails, DCM enters `LATCHED_UART_UNOWNED_IDLE` and runs a full rescan every 100 ms.  K=3 consecutive NONE confirmations release the latch (disconnect-detection latency ≈ 350 ms).

## 8. Reference Resources

- [pybricks/technical-info UART Protocol](https://github.com/pybricks/technical-info/blob/master/uart-protocol.md)
- [ev3dev UART Sensor Protocol](http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-stretch/sensors.html)
- `pybricks/lib/pbio/drv/legodev/legodev_pup.c` -- Port-level driver (DCM source)
- `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c` -- LUMP UART protocol (1265 lines, ported in #43)
