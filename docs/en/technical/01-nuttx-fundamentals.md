# NuttX Fundamentals Research

## 1. Version Selection

### Latest Releases

| Version | Release Date |
|---|---|
| **12.12.0** | 2025-12-31 |
| 12.11.0 | 2025-10-05 |
| 12.10.0 | 2025-07-07 |
| 12.9.0 | 2025-04-14 |
| 12.8.0 | 2025-01-06 |

### STM32F4 Support Status

STM32F4 is a mature platform in NuttX. Officially supported boards include:

- Nucleo F401RE, F410RB, F411RE, F412ZG, F429ZI, F446RE
- STM32F4-Discovery, STM32F411E-Discovery, STM32F429I-DISCO
- Olimex STM32-E407, H405, H407, P407
- OMNIBUSF4, ODrive V3.6, Axoloti, and others

### STM32F413 Support Status

**STM32F413 is NOT supported at the chip level.** No `CONFIG_ARCH_CHIP_STM32F413` entry exists in `arch/arm/src/stm32/Kconfig`. The F41x coverage goes: F410, F411, F412 — F413 is missing.

The closest existing chip is **STM32F412**. The F413 adds:
- Additional UART channels (UART7-10)
- Additional timers (including LPTIM)
- Additional SPI/I2C
- DFSDM
- Larger flash (1.5 MB vs 1 MB on F412)

ChibiOS has precedent for adding F413 by cloning F412 support.

### Decision

**Use NuttX 12.12.0.**

- STM32F4 platform is mature; latest release has the most bug fixes
- F413 chip support needs to be added, but can be based on F412
- For initial bring-up, using F412 config directly and incrementally adding F413-specific support is a viable approach

---

## 2. Build System

### Two-Repository Structure

```
workspace/
  nuttx/          # apache/nuttx — kernel/OS
  nuttx-apps/     # apache/nuttx-apps — applications (a.k.a. "apps")
```

The apps repo defaults to `../apps` relative to the nuttx directory. Override with `-a <path>`.

**nuttx repository:**
- `arch/` — processor architecture and chip-specific code
- `boards/` — board definitions (`boards/<arch>/<chip>/<board>/`)
- `drivers/` — OS device drivers
- `sched/`, `fs/`, `net/` — scheduler, filesystems, networking
- `tools/` — build tools including `configure.sh`

**nuttx-apps repository:**
- `examples/` — sample apps (hello, etc.)
- `nshlib/` — NuttShell library
- `system/` — system utilities
- `builtin/` — built-in app registration

### Kconfig / defconfig

NuttX uses the Linux Kconfig system:

- **Kconfig**: distributed throughout the source tree, defining configurable options per directory
- **defconfig**: stored at `boards/<arch>/<chip>/<board>/configs/<config>/defconfig`. Contains only settings that differ from defaults
- **.config**: full expanded configuration file generated at the nuttx root during builds

```
# Example defconfig
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32"
CONFIG_ARCH_CHIP_STM32F412ZG=y
CONFIG_ARCH_BOARD="nucleo-f412zg"
```

Configuration workflow:
1. `make menuconfig` — interactive menu-based configuration
2. `make olddefconfig` — apply defaults for unset options
3. `make savedefconfig` — generate minimal defconfig from current .config

### configure.sh Workflow

```bash
cd nuttx
./tools/configure.sh -l <board>:<config>   # -l: Linux, -m: macOS
make -j$(nproc)
```

Steps performed:
1. Parse `<board>:<config>` → resolve to `boards/*/*/<board>/configs/<config>/`
2. Locate and copy `Make.defs`
3. Copy `defconfig` to `nuttx/.config`
4. Set host OS in .config
5. Run `make olddefconfig` to expand to full .config

### Board Definition Files

```
boards/arm/stm32/<board>/
  Kconfig                          # Board-specific Kconfig
  CMakeLists.txt                   # add_subdirectory(src)
  configs/nsh/defconfig            # Minimal NSH build config
  include/board.h                  # Clock config, pin mappings, LED/button defs
  scripts/Make.defs                # Toolchain, LDSCRIPT, compiler flags
  scripts/ld.script                # Linker script (MEMORY, SECTIONS)
  src/Make.defs                    # CSRCS = stm32_boot.c ...
  src/CMakeLists.txt               # CMake source list
  src/stm32_boot.c                 # stm32_boardinitialize() implementation
  src/stm32_bringup.c              # Peripheral initialization
  src/<board>.h                    # Board internal header
```

Also requires an entry in `boards/Kconfig`:
```kconfig
config ARCH_BOARD_SPIKE_PRIME_HUB
    bool "LEGO SPIKE Prime Hub"
    depends on ARCH_CHIP_STM32F413VG
```

### Out-of-tree Custom Board

Board can be defined without modifying the NuttX source tree:
```
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../custom-boards/spike-prime-hub"
```

```bash
./tools/configure.sh -l ../custom-boards/spike-prime-hub/configs/nsh
```

### Build Dependencies

| Tool | Purpose |
|---|---|
| `arm-none-eabi-gcc` | ARM cross-compiler |
| `python3-kconfiglib` | Kconfig tools (pure Python, officially recommended by NuttX) |
| `make` (GNU Make) | Build system |
| `genromfs` | ROMFS image generation (optional) |
| `git`, `bison`, `flex`, `gperf`, `xxd` | Build dependencies |

---

## 3. License Compatibility

### Compatibility Matrix

| Combination | Compatible? | Notes |
|---|---|---|
| Apache 2.0 (NuttX) + MIT (project) | Yes | Apache 2.0 governs the combined work |
| Apache 2.0 + BSD 3-Clause (STM32 HAL) | Yes | Each file retains its original license header |
| MIT + BSD 3-Clause | Yes | All permissive licenses |
| pybricks as reference (no code copy) | No issue | Learning/reference triggers no obligation |

### Attribution Requirements

| License | Requirements |
|---|---|
| Apache 2.0 | Include license copy. Retain NOTICE files. State changes to Apache-licensed files |
| MIT | Include copyright notice and license text in copies |
| BSD 3-Clause | Include copyright/license in distributions. No endorsement using contributor names |

### Policy

- NuttX-derived code retains Apache 2.0 headers
- Original project code uses MIT
- STM32 HAL files retain BSD 3-Clause headers
- pybricks is reference only (no code copied) — no obligation. Acknowledge in docs as courtesy
