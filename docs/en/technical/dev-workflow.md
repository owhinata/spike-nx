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

## 2. SWD Debug Feasibility

### Constraints

The SPIKE Prime Hub repurposes SWD debug pins for power control:

| Pin | SWD Function | Hub Usage |
|---|---|---|
| PA13 | SWDIO | BAT_PWR_EN (battery power hold) |
| PA14 | SWCLK | PORT_3V3_EN (I/O port 3.3V power) |

**SWD connection is lost once PA13 is configured as GPIO output.**

### Possibilities

1. **USB-powered operation**: PA13 doesn't need to be HIGH when USB-powered. Debug builds could delay PA13 reconfiguration to maintain SWD
2. **Hardware modification**: If SWD test pads exist on the PCB, an external probe (J-Link, ST-Link) could connect. Requires hub disassembly
3. **Practically difficult**: Assume SWD is unavailable for normal development

### Alternative Debug Methods

| Method | Phase | Details |
|---|---|---|
| I/O port UART | Initial bring-up | Connect USB-UART adapter to any I/O port. Simplest debug method |
| USB CDC/ACM | After USB works | NSH console. No extra hardware needed |
| LED indication | Always | Check if LEDs can be controlled via direct GPIO before TLC5955 driver |
| Hard fault output | Always | NuttX hard fault handler dumps registers to UART/USB |

---

## 3. Serial Console Strategy

### Phase 1: Initial Bring-up (USB Not Working)

**Use I/O port UART.**

Each SPIKE Hub I/O port has a UART. Connect a 3.3V USB-UART adapter:

| Port | UART | TX Pin | RX Pin | Recommended Because |
|---|---|---|---|---|
| A | UART7 | PE8 | PE7 | UART7 supported in NuttX |
| B | UART4 | PD1 | PD0 | UART4 supported in NuttX |

NuttX defconfig for UART console:
```
CONFIG_STM32_UART7=y
CONFIG_UART7_SERIALDRIVER=y
CONFIG_UART7_SERIAL_CONSOLE=y
CONFIG_UART7_BAUD=115200
CONFIG_UART7_BITS=8
CONFIG_UART7_PARITY=0
CONFIG_UART7_2STOP=0
```

**Note**: I/O port connector TX/RX pin layout — refer to pybricks pin map. LEGO proprietary connector requires appropriate breakout cable.

### Phase 2: After USB Works

**Switch to USB CDC/ACM.**

```
CONFIG_STM32_OTGFS=y
CONFIG_USBDEV=y
CONFIG_CDCACM=y
CONFIG_CDCACM_CONSOLE=y
CONFIG_NSH_USBCONSOLE=y
```

Recognized as `/dev/ttyACM0` (Linux) or `/dev/cu.usbmodemXXXX` (macOS) on host PC.

---

## 4. Minimal Bring-up Sequence

### Step 1: Clock Config + Power Hold

**Goal**: Hub stays powered on without hard faulting

- PLL setup: 16 MHz HSE → 96 MHz SYSCLK
- Drive BAT_PWR_EN (PA13) HIGH
- Success: Hub maintains power, doesn't shut down

### Step 2: UART Output

**Goal**: Serial debug output available

- Enable I/O port UART (UART7 recommended)
- Early UART init via `stm32_lowsetup()`
- Success: Boot messages visible via USB-UART adapter

### Step 3: NSH Shell (UART)

**Goal**: NuttX OS running with command shell

- NuttX kernel boot → NSH shell
- Command input/output via UART console
- Success: `nsh>` prompt displayed, `help` command works

### Step 4: USB CDC/ACM

**Goal**: Console over USB

- Enable USB OTG FS driver
- Configure CDC/ACM device class
- Success: Host PC recognizes `/dev/ttyACM0`, NSH works over USB

### Step 5: GPIO / LED

**Goal**: Visual feedback

- GPIO control of status LEDs
- TLC5955 driver deferred; check if any LED is directly GPIO-controllable
- Success: LED on/off control works

### Step 6: Additional Peripherals

Enable incrementally:
1. SPI (W25Q256 flash → filesystem)
2. I2C (LSM6DS3TR-C IMU)
3. ADC (battery monitoring)
4. PWM (motor control)
5. SPI (TLC5955 LED matrix)
6. Additional UARTs (I/O port communication)

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
