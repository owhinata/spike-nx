# Getting Started

Build, flash, and connect procedure for the NuttX firmware targeting SPIKE Prime Hub.

## Build

The Docker container handles everything (submodule init, Docker image build, configure, make).

```bash
make
```

The build produces a kernel blob `nuttx/nuttx.bin` (~139 KB) and a user blob `nuttx/nuttx_user.bin` (~146 KB). The default `usbnsh` configuration is BUILD_PROTECTED, so both blobs must be flashed to the device.

### Kconfig Configuration

```bash
# Interactive menu configuration
make nuttx-menuconfig

# Save minimal defconfig from the current .config
make nuttx-savedefconfig
```

### Clean

```bash
# Remove build artifacts only (.config is preserved)
make nuttx-clean

# Full clean including .config
make nuttx-distclean

# Delete Docker image + submodule deinit
make distclean
```

## DFU Flashing

### Entering DFU Mode

1. Disconnect the USB cable
2. Press and hold the Bluetooth button for 5 seconds
3. While holding the button, connect the USB cable
4. Release the button

### Flash Command

The default `usbnsh` configuration is BUILD_PROTECTED, so flash the kernel blob at `0x08008000` and the user blob at `0x08080000`:

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
```

| Option | Description |
|---|---|
| `-d 0694:0008` | VID/PID of the SPIKE Prime Hub |
| `-a 0` | Alternate interface 0 (internal flash) |
| `-s 0x08008000` | Kernel blob start address (right after the 32 KB LEGO bootloader) |
| `-s 0x08080000:leave` | User blob start address (2^n aligned) + exit DFU after writing |
| `-D nuttx/nuttx.bin` / `-D nuttx/nuttx_user.bin` | Binary to download |

On macOS, install with `brew install dfu-util`.

## Serial Connection

```bash
picocom /dev/tty.usbmodem01
```

Connects to the NSH console via USB CDC/ACM.

## Out-of-tree Board Definition

Define a board without modifying the NuttX source tree. The board definition is placed in `boards/spike-prime-hub/`:

```
boards/spike-prime-hub/
├── configs/nsh/defconfig     # Minimal configuration for NSH build
├── include/board.h           # Clock settings, pin definitions
├── scripts/
│   ├── Make.defs             # Toolchain, LDSCRIPT, compiler flags
│   └── ld.script             # Linker script (MEMORY, SECTIONS)
├── src/
│   ├── Make.defs             # CSRCS = stm32_boot.c ...
│   ├── stm32_boot.c          # stm32_boardinitialize() implementation
│   ├── stm32_bringup.c       # Peripheral initialization
│   └── spike_prime_hub.h     # Board-internal header
└── Kconfig                   # Board-specific Kconfig
```

Specify the custom board in defconfig:

```
CONFIG_ARCH="arm"
CONFIG_ARCH_ARM=y
CONFIG_ARCH_CHIP="stm32"
CONFIG_ARCH_CHIP_STM32F413VG=y
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../boards/spike-prime-hub"
CONFIG_ARCH_BOARD_CUSTOM_NAME="spike-prime-hub"
CONFIG_APPS_DIR="../nuttx-apps"
```

## Docker Image

Defined in `docker/Dockerfile.nuttx`. Based on Ubuntu 24.04:

| Package | Purpose |
|---|---|
| `gcc-arm-none-eabi`, `libnewlib-arm-none-eabi` | ARM cross-compiler |
| `python3-kconfiglib` | Kconfig tool (pure Python implementation officially recommended by NuttX) |
| `bison`, `flex`, `gperf`, `xxd` | Build dependencies |
| `genromfs` | ROMFS image generation |
| `dfu-util` | DFU flashing |

> **Note**: The C-based `kconfig-frontends` is not used because it causes syntax errors with newer NuttX Kconfig files.
