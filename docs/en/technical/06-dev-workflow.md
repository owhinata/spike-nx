# Development Workflow

## 1. DFU Flash Procedure

### Enter DFU Mode

1. Unplug USB cable
2. Hold the center Bluetooth button for 5 seconds
3. While holding, plug in USB cable
4. Status LED blinks → DFU mode active
5. Release button

### Verify Device

```bash
dfu-util -l
# Found DFU: [0694:0008] ... name="Internal Flash"
```

### Flash Firmware

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

| Option | Meaning |
|---|---|
| `-d 0694:0008` | LEGO VID/PID |
| `-a 0` | Alternate interface 0 (internal flash) |
| `-s 0x08008000:leave` | Write address + exit DFU after flash |
| `-D nuttx.bin` | Binary to download |

The `:leave` suffix auto-boots the firmware after writing.

### One-Command Flash

```bash
make -f scripts/nuttx.mk flash
```

---

## 2. Debug Strategy

### SWD

SWD pins (PA13/PA14) are repurposed for power control, and extracting them externally is impractical. **SWD is not used.**

| Pin | SWD Function | Hub Usage |
|---|---|---|
| PA13 | SWDIO | BAT_PWR_EN (battery power hold) |
| PA14 | SWCLK | PORT_3V3_EN (I/O port 3.3V power) |

### Debug Methods

| Method | Use Case | Details |
|---|---|---|
| **USB CDC/ACM NSH** | Primary day-to-day | NSH console + syslog + `dmesg` |
| **RAMLOG + dmesg** | Log retention on USB disconnect | RAM ring buffer. Check after reconnect |
| **coredump** | Post-crash analysis | Persist to backup SRAM → analyze with GDB |
| **NSH commands** | Runtime monitoring | `ps`, `free`, `top`, `/proc` |

See [13-debugging-strategy.md](13-debugging-strategy.md) for details.

---

## 3. Console

**Fixed to USB CDC/ACM.** I/O ports are not reserved for debugging (all ports available for Powered Up devices).

```
CONFIG_STM32_OTGFS=y
CONFIG_USBDEV=y
CONFIG_CDCACM=y
CONFIG_CDCACM_CONSOLE=y
CONFIG_NSH_USBCONSOLE=y
CONFIG_NSH_USBCONDEV="/dev/ttyACM0"
```

Recognized as `/dev/ttyACM0` (Linux) or `/dev/cu.usbmodemXXXX` (macOS) on host PC.

### Initial Bring-up Note

No serial output until USB CDC/ACM driver is working. Workarounds if USB isn't functional early on:

1. **RAMLOG**: Boot messages accumulate in RAM. Check via `dmesg` once USB works
2. **LED**: After TLC5955 driver, show boot progress via LEDs
3. **Last resort**: Temporarily use I/O port UART (e.g., UART7) for debug output

---

## 4. Minimal Bring-up Sequence

### Step 1: Clock Config + Power Hold

**Goal**: Hub stays powered on without hard faulting

- PLL setup: 16 MHz HSE → 96 MHz SYSCLK
- Drive BAT_PWR_EN (PA13) HIGH
- Success: Hub maintains power, doesn't shut down

### Step 2: USB CDC/ACM

**Goal**: Establish console connection

- Enable USB OTG FS driver
- Configure CDC/ACM device class
- Success: Host PC recognizes `/dev/ttyACM0`

### Step 3: NSH Shell

**Goal**: NuttX OS running with command shell

- NuttX kernel boot → NSH shell
- Command input/output via USB CDC/ACM console
- Success: `nsh>` prompt displayed, `help` command works

### Step 4: TLC5955 LED

**Goal**: Visual feedback

- Implement SPI1 + TLC5955 driver
- Control status LEDs / 5×5 matrix
- Success: LED on/off control works
- Note: All LEDs are TLC5955-driven. No directly GPIO-controllable LEDs exist

### Step 5: Additional Peripherals

Enable incrementally:
1. SPI2 (W25Q256 flash → filesystem)
2. I2C2 (LSM6DS3TR-C IMU)
3. ADC1 (battery monitoring)
4. PWM (motor control)
5. UART (I/O port Powered Up device communication)

---

## 5. Development Cycle

### Standard Cycle

```
Edit code
  ↓
make -f scripts/nuttx.mk          # Build in Docker (~30s)
  ↓
Enter DFU mode                     # Button sequence (~5s)
  ↓
make -f scripts/nuttx.mk flash    # DFU flash (~10s)
  ↓
Monitor serial output              # screen /dev/ttyACM0 115200
```

Estimated cycle time: ~1 minute

### Build Only (No Flash)

```bash
make -f scripts/nuttx.mk build
```

### Configuration Changes

```bash
make -f scripts/nuttx.mk menuconfig   # Interactive config
make -f scripts/nuttx.mk savedefconfig # Update defconfig
```

### Clean Build

```bash
make -f scripts/nuttx.mk clean        # Build artifacts only
make -f scripts/nuttx.mk distclean    # Full clean
```
