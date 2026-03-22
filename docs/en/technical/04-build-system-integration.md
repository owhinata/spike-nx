# Build System Integration

## 1. Toolchain Requirements

### Existing Docker Image Packages

Current `docker/Dockerfile` (Ubuntu 24.04) includes:
- `build-essential`, `gcc-arm-none-eabi`, `libnewlib-arm-none-eabi`
- `git`, `python3`, `python3-pip`, `python3-venv`, `pipx`, `unzip`, `zip`
- pip: `pycryptodomex`
- pipx: `poetry`

### Additional Packages for NuttX

```bash
apt-get install -y --no-install-recommends \
    bison flex gettext texinfo \
    libncurses5-dev libncursesw5-dev xxd \
    gperf automake libtool pkg-config genromfs \
    libgmp-dev libmpc-dev libmpfr-dev libisl-dev \
    python3-kconfiglib \
    u-boot-tools util-linux
```

| Package | Purpose |
|---|---|
| `python3-kconfiglib` | Kconfig tools (menuconfig, etc.). Pure Python, officially recommended by NuttX |
| `genromfs` | ROMFS image generation (optional) |
| `bison`, `flex` | Parser generators |
| `gperf` | Hash function generator |
| `xxd` | Binary-to-C-header conversion |
| Others | Build dependencies |

> **Note**: C-based `kconfig-frontends` can cause syntax errors with newer NuttX Kconfig files — do not use ([issue #2405](https://github.com/apache/nuttx/issues/2405)).

### For DFU Flashing

```bash
apt-get install -y dfu-util
```

---

## 2. Source Integration Strategy

### Decision: git submodules

Following the same pattern as pybricks:

```
spike-nx/
├── nuttx/              # git submodule: apache/nuttx (tag nuttx-12.12.0)
├── nuttx-apps/         # git submodule: apache/nuttx-apps (tag nuttx-12.12.0)
├── boards/             # custom out-of-tree board definitions
│   └── spike-prime-hub/
│       ├── configs/nsh/defconfig
│       ├── include/board.h
│       ├── scripts/
│       │   ├── Make.defs
│       │   └── ld.script
│       ├── src/
│       │   ├── Make.defs
│       │   ├── stm32_boot.c
│       │   ├── stm32_bringup.c
│       │   └── spike_prime_hub.h
│       └── Kconfig
├── docker/
├── scripts/
└── pybricks/
```

### Rationale

- No NuttX kernel code changes needed (out-of-tree custom board)
- Upstream updates via tag switching
- Forking has higher maintenance cost
- Note: F413 chip support addition may require a fork

### Out-of-tree Custom Board Configuration

defconfig settings:

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

### Configure Command

```bash
cd nuttx
./tools/configure.sh -l ../boards/spike-prime-hub/configs/nsh
```

### Caveats

- `make distclean` erases `.config`; re-run `configure.sh` afterwards
- Default apps directory is `../apps`, but submodule is `nuttx-apps` — specify `CONFIG_APPS_DIR` explicitly
- F413 chip support doesn't exist in NuttX; initially use F412 config or fork NuttX

---

## 3. Build Script Design

### `scripts/nuttx.mk`

Following the `scripts/pybricks.mk` pattern:

```makefile
DOCKER_IMAGE := nuttx-builder
DOCKER_FILE  := docker/Dockerfile.nuttx
NUTTX_DIR    := nuttx
APPS_DIR     := nuttx-apps
BOARD_DIR    := boards/spike-prime-hub
BOARD_CONFIG := nsh

# Docker run with UID/GID mapping
DOCKER_RUN := docker run --rm \
    -v $(CURDIR):$(CURDIR) \
    -w $(CURDIR) \
    --user $(shell id -u):$(shell id -g) \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/group:/etc/group:ro \
    $(DOCKER_IMAGE)
```

### Targets

| Target | Action |
|---|---|
| `build` (default) | submodule init → Docker image build → configure → make |
| `configure` | Run `tools/configure.sh` |
| `clean` | `make clean` (build artifacts only) |
| `distclean` | `make distclean` + Docker image removal + submodule deinit |
| `flash` | Flash firmware via `dfu-util` (runs on host) |
| `menuconfig` | `make menuconfig` (interactive config) |

### Build Flow

```
make -f scripts/nuttx.mk
  1. git submodule update --init nuttx nuttx-apps (if not initialized)
  2. docker build (if image not present)
  3. docker run: cd nuttx && ./tools/configure.sh -l ../boards/spike-prime-hub/configs/nsh
  4. docker run: cd nuttx && make -j$(nproc)
  5. Output: nuttx/nuttx.bin
```

---

## 4. DFU Binary Generation

### Method 1: Direct binary flash (Recommended)

No DFU file conversion needed. Flash `nuttx.bin` directly with address:

```bash
dfu-util -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

| Option | Meaning |
|---|---|
| `-a 0` | Alternate interface 0 (internal flash) |
| `-s 0x08008000:leave` | Start address + exit DFU mode after flash |
| `-D nuttx.bin` | Binary to download |

Simplest approach for development.

### Method 2: DfuSe .dfu File

Using `dfuse-pack.py` (from dfu-util source repository):

```bash
python3 dfuse-pack.py -b 0x08008000:nuttx.bin firmware.dfu
dfu-util -a 0 -D firmware.dfu
```

The `.dfu` file embeds the target address, so no `-s` flag needed when flashing.

### NuttX Binary Output

NuttX build produces:
- `nuttx` — ELF file (with debug info)
- `nuttx.bin` — raw binary (via `arm-none-eabi-objcopy -O binary`)

`nuttx.bin` is the flash target.

### VID/PID

SPIKE Prime Hub DFU uses LEGO-specific VID/PID:
```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx.bin
```

### Flash Script (make flash)

```bash
# Enter DFU mode:
# 1. Unplug USB cable
# 2. Hold Bluetooth button for 5 seconds
# 3. While holding, plug in USB cable
# 4. Release button
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```
