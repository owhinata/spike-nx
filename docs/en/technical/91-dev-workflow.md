# Development Workflow

> **Note**: This document replaces 06-dev-workflow.md.

## 1. Development Environment Overview

The SPIKE Prime Hub is not yet available. Initial bring-up uses the **STM32F413H-Discovery Kit** which has the same MCU family (STM32F413). Powered Up device driver implementation is deferred until Hub is available.

### STM32F413H-Discovery vs SPIKE Prime Hub

| Item | Discovery Kit | SPIKE Prime Hub |
|---|---|---|
| MCU | STM32F413**ZH**T6 (144-pin) | STM32F413**VG**T6 (100-pin) |
| Flash | 1.5 MB | 1 MB (VG nominal. May be 1.5MB physically — needs verification) |
| RAM | 320 KB | 320 KB |
| HSE | 8 MHz | 16 MHz |
| Flash origin | 0x08000000 | 0x08008000 (after LEGO bootloader) |
| SWD | ST-Link/V2-1 **available** | PA13/PA14 repurposed, **not available** |
| NSH Console | USB CDC/ACM | USB CDC/ACM |
| USART6 | Connected to ST-Link VCP (debug reserve) | Unassigned (pybricks debug only) |
| LED | GPIO direct (LD1=PE3, LD2=PC5) | TLC5955 via SPI1 (no direct GPIO) |
| PA13/PA14 | SWDIO/SWCLK (debug) | BAT_PWR_EN / PORT_3V3_EN (power) |
| USB OTG FS | Yes | Yes |
| User Button | Yes | Yes (Bluetooth button) |

> **Flash size note**: SPIKE Hub MCU is STM32F413VG (nominal 1MB Flash). However, pybricks linker script uses 992K after bootloader, and STM32F413 may physically have 1.5MB across all variants. Needs hardware verification.

### NuttX Fork

- https://github.com/owhinata/nuttx (`f413-support` branch)
- https://github.com/owhinata/nuttx-apps

---

## 2. Bring-up Plan

### Phase A: Build Environment Setup

| File | Content |
|---|---|
| `docker/Dockerfile.nuttx` | Ubuntu 24.04 + gcc-arm-none-eabi + python3-kconfiglib etc. |
| `scripts/nuttx.mk` | Build script (follows pybricks.mk pattern) |
| `.gitmodules` (update) | Add owhinata/nuttx + owhinata/nuttx-apps as submodules |

**nuttx.mk targets:**

| Target | Action |
|---|---|
| `build` (default) | submodule init → Docker build → configure → make |
| `configure` | Run `tools/configure.sh` |
| `clean` | `make clean` |
| `distclean` | Full clean (Docker image removal + submodule deinit) |
| `menuconfig` | Interactive configuration |
| `savedefconfig` | Update defconfig |
| `flash` | Discovery: OpenOCD SWD / Hub: dfu-util DFU |

**Board selection:**

```bash
make -f scripts/nuttx.mk                          # Default: stm32f413-discovery
make -f scripts/nuttx.mk BOARD=spike-prime-hub     # SPIKE Hub
```

### Phase B: F412 Workaround Bring-up (Discovery)

Use F412ZG config as workaround to run NSH on Discovery Kit. Console is **USB CDC/ACM** (same as SPIKE Hub). USART6 is kept free.

**Board definition:**

```
boards/stm32f413-discovery/
  Kconfig
  configs/nsh/defconfig           # F412ZG workaround, USB CDC/ACM console
  include/board.h                 # 8MHz HSE → 96MHz SYSCLK
  scripts/Make.defs
  scripts/ld.script               # 0x08000000, 1024K Flash, 256K SRAM (F412 limit)
  src/Make.defs
  src/stm32_boot.c
  src/stm32_bringup.c
  src/stm32_usbdev.c              # USB device initialization
  src/stm32f413_discovery.h
```

**Success criteria:** NSH works on USB CDC/ACM, LEDs light up

### Phase C: STM32F413 Chip Support (NuttX Fork)

Apply 10-file patch to `f413-support` branch of owhinata/nuttx:

1. Kconfig: F413 family + UART9/10
2. chip.h: peripheral counts, 1.5MB Flash / 320KB SRAM
3. stm32f40xxx_irq.h: UART9 IRQ=88, UART10 IRQ=89
4. stm32f40xxx_memorymap.h: UART9/10 base addresses
5. stm32f40xxx_rcc.h: APB2ENR UART9/10 bits
6. stm32f40xxx_rcc.c: add UART9/10 to rcc_enableapb2()
7. stm32_serial.c: UART9/10 device instances (uses PCLK2)
8. stm32f413xx_pinmap.h: new F413 pinmap
9. stm32_pinmap.h: F413 include branch
10. boards/Kconfig: board entry

**Success criteria:** `free` shows ~320KB RAM, no hard faults

### Phase D: UART9/10 Verification (Discovery)

Verify F413-specific UART9/10 on Discovery Kit:

| UART | TX | RX | Notes |
|---|---|---|---|
| UART9 | PD15 (AF11) | PD14 (AF11) | |
| UART10 | PG12 (AF11) | PG11 (AF11) | Avoid PE3 (conflicts with LD1) |

**Success criteria:** 115200 baud send/receive via USB-UART adapter

### Phase E: SPIKE Hub Board Definition (Build Only)

Create Hub board definition. Hardware testing deferred until Hub is available.

```
boards/spike-prime-hub/
  configs/nsh/defconfig           # USB CDC/ACM console
  include/board.h                 # 16MHz HSE → 96MHz SYSCLK
  scripts/ld.script               # 0x08008000, 992K Flash, 320K SRAM
  src/stm32_boot.c                # PA13 BAT_PWR_EN, PA14 PORT_3V3_EN
```

**Success criteria:** Build succeeds, binary linked at 0x08008000

### Execution Order

```
Phase A ──→ Phase B ──→ Phase C ──→ Phase D
                           │
                           └──→ Phase E
```

---

## 3. Development Cycle

### Discovery Kit

```
Edit code
  ↓
make -f scripts/nuttx.mk                         # Docker build
  ↓
make -f scripts/nuttx.mk flash                   # OpenOCD SWD flash
  ↓
screen /dev/ttyACM0 115200                        # USB CDC/ACM console
```

Flash (OpenOCD SWD):
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program nuttx/nuttx.bin 0x08000000 verify reset exit"
```

GDB debugging (Discovery only):
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c '$_TARGETNAME configure -rtos nuttx' -c 'init; reset halt'
# In another terminal:
arm-none-eabi-gdb nuttx/nuttx -ex 'target extended-remote localhost:3333'
```

### SPIKE Prime Hub (DFU) — After Hub Available

```
Edit code
  ↓
make -f scripts/nuttx.mk BOARD=spike-prime-hub    # Docker build
  ↓
Enter DFU mode                                     # Bluetooth button + USB
  ↓
make -f scripts/nuttx.mk BOARD=spike-prime-hub flash  # dfu-util DFU flash
  ↓
screen /dev/ttyACM0 115200                         # USB CDC/ACM console
```

DFU mode entry:
1. Unplug USB cable
2. Hold Bluetooth button for 5 seconds
3. While holding, plug in USB cable
4. Status LED blinks → DFU mode
5. Release button

DFU flash command:
```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

### Common Operations

```bash
make -f scripts/nuttx.mk build                    # Build only
make -f scripts/nuttx.mk menuconfig                # Interactive config
make -f scripts/nuttx.mk savedefconfig             # Update defconfig
make -f scripts/nuttx.mk clean                     # Clean build artifacts
make -f scripts/nuttx.mk distclean                 # Full clean
```

---

## 4. Debug Strategy

### Discovery Kit

| Method | Use Case |
|---|---|
| **SWD + OpenOCD + GDB** | Step execution, breakpoints, memory inspection |
| **USB CDC/ACM NSH** | NSH console + syslog + `dmesg` |
| **USART6 VCP** | Debug reserve (not used for NSH. Available for secondary syslog etc.) |
| **NSH commands** | `ps`, `free`, `top`, `dmesg`, `/proc` |
| **LEDs (PE3, PC5)** | Boot progress indication |

### SPIKE Prime Hub (After Available)

SWD not available. Limited debug methods:

| Method | Use Case |
|---|---|
| **USB CDC/ACM NSH** | Primary day-to-day method |
| **RAMLOG + dmesg** | Log retention on USB disconnect / early boot |
| **coredump** | Post-crash analysis (backup SRAM persistence) |
| **NSH commands** | `ps`, `free`, `top`, `/proc` |

See [13-debugging-strategy.md](13-debugging-strategy.md) for details.
