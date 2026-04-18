# BUILD_PROTECTED Migration

Tracking Issue #37 — migrating to Cortex-M4 MPU-based kernel / user space separation. **Status: verified working on the SPIKE Prime Hub `usbnsh` configuration (2026-04-18).**

## Motivation

With `CONFIG_BUILD_FLAT=y` (the default), user code under `apps/` runs in privileged thread mode, so privileged instructions such as `MSR BASEPRI` are accessible — user code can mask interrupts directly. Moving to `CONFIG_BUILD_PROTECTED=y` uses the MPU to:

- Run user code in unprivileged mode, blocking privileged instructions
- Catch illegal interrupt-mask / kernel-memory / peripheral-register access with HardFault/MemFault
- Substantially improve system robustness

## Memory map

Designed for SPIKE Prime Hub (STM32F413VG, 1.5 MB Flash / 320 KB RAM).

### Flash (starting at 0x08008000; 0x08000000–0x08008000 is the LEGO bootloader)

| Region | Range | Size | Purpose |
|---|---|---|---|
| `kflash` | `0x08008000..0x08080000` | 480K | Privileged kernel code (absorbs alignment gap) |
| `uflash` | `0x08080000..0x08100000` | 512K | User-space code (2^n aligned) |
| `xflash` | `0x08100000..0x08180000` | 512K | User-space spare |

`CONFIG_NUTTX_USERSPACE = 0x08080000`

### SRAM (at 0x20000000, 320 KB)

| Region | Range | Size | Purpose |
|---|---|---|---|
| `ksram` | `0x20000000..0x20010000` | 64K | Kernel .data/.bss (linker only) |
| `usram` | `0x20010000..0x20020000` | 64K | User .data/.bss (64K aligned) |
| `xsram` | `0x20020000..0x20050000` | 192K | Runtime heap (kernel heap + user heap) |

`CONFIG_MM_KERNEL_HEAPSIZE = 32768` (lower bound; `stm32_allocateheap.c` partitions the remainder at runtime).

`SRAM1_END = 0x20050000` is not a power-of-two boundary, so rounding the largest MPU-protectable user heap (128 K) down to the next 2^n boundary leaves roughly a 60 K **tail region** unused. The two settings below recover it (see [NuttX-side fixes](#required-nuttx-side-fixes) for details):

- `CONFIG_MM_REGIONS=2`
- `CONFIG_STM32_CCMEXCLUDE=y` (the F413 has no CCM SRAM, but this declaration repurposes the second region slot for tail recovery)

## Implementation files

| Path | Contents |
|---|---|
| `boards/spike-prime-hub/configs/usbnsh/defconfig` | All BUILD_PROTECTED-related config |
| `boards/spike-prime-hub/kernel/Makefile` | PASS1 (user blob) build rules |
| `boards/spike-prime-hub/kernel/stm32_userspace.c` | `struct userspace_s` at user blob origin |
| `boards/spike-prime-hub/kernel/CMakeLists.txt` | Same, for CMake |
| `boards/spike-prime-hub/scripts/memory.ld` | Physical memory layout |
| `boards/spike-prime-hub/scripts/kernel-space.ld` | Kernel blob section placement |
| `boards/spike-prime-hub/scripts/user-space.ld` | User blob section placement |
| `boards/spike-prime-hub/scripts/Make.defs` | `ARCHSCRIPT` now uses `memory.ld + kernel-space.ld` |
| `apps/imu/*.c` | Replaced `clock_systime_ticks()` with POSIX `clock_gettime(CLOCK_MONOTONIC)` (kernel symbol replaced by a user-callable syscall) |

## Key CONFIG flags

```
# BUILD_PROTECTED core
CONFIG_BUILD_PROTECTED=y
CONFIG_NUTTX_USERSPACE=0x08080000
CONFIG_PASS1_BUILDIR="../boards/spike-prime-hub/kernel"

# MPU
CONFIG_ARM_MPU=y
CONFIG_ARM_MPU_EARLY_RESET=y
CONFIG_ARM_MPU_RESET=y

# Heap / stack
CONFIG_MM_KERNEL_HEAPSIZE=32768
CONFIG_MM_REGIONS=2
CONFIG_STM32_CCMEXCLUDE=y
CONFIG_INIT_STACKSIZE=4096
CONFIG_SYSTEM_NSH_STACKSIZE=4096
```

## Required NuttX-side fixes

The owhinata fork (`f413-support-12.13.0` branch) carries these three commits:

| Commit | Contents |
|---|---|
| `97716f5a2a` | `arch/armv7-m`: preserve the caller's `r11` across `arm_dispatch_syscall` (the BUILD_PROTECTED syscall path was breaking AAPCS) |
| `b35c473a58` | `arch/stm32`: when `SRAM1_END` is not a power-of-two boundary, the MPU-aligned user heap loses its tail; recover it through an extra `arm_addregion()` (requires `CONFIG_MM_REGIONS>=2`) |
| `7c116a6de2` | `arch/arm`: disable `fork()` support under BUILD_PROTECTED (kernel/user stacks cannot be shared across the boundary) |

## Related changes

| Item | Action | Reason |
|---|---|---|
| `CONFIG_APP_LED` disabled | Tracked separately | `led_main.c` calls `tlc5955_set_duty()` directly — needs refactor to a `/dev/rgbled`-style char driver |
| `CONFIG_ARCH_PERF_EVENTS` disabled | Tracked separately | `nuttx-apps/testing/ostest/perf_gettime.c` calls `perf_gettime()` which has no syscall proxy |

## Build

```bash
make nuttx-distclean && make
```

Artifacts:

- `nuttx/nuttx.bin` (~139 KB) — kernel blob @ `0x08008000`
- `nuttx/nuttx_user.bin` (~146 KB) — user blob @ `0x08080000`

## Flash (two steps)

```bash
# Enter DFU: unplug USB, hold BT button, plug USB, wait 5 s
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
```

## Hardware verification (2026-04-18, commit `55347a8`)

| Item | Result |
|---|---|
| NSH boot (USB CDC) | ✅ `NuttShell (NSH) NuttX-12.13.0` prompt reached |
| `free` Umem | ✅ 196 608 B (192 K) — tail recovery delivers the planned size |
| `free` Kmem | ✅ 62 536 B |
| CoreMark 2000 iter | ✅ **170.46 iter/sec** |
| `sleep 10` (watchdog interaction) | ✅ Returns after 10.0 s, no watchdog fire |
| `pytest -m "not slow and not interactive"` | ✅ **28 passed**, 2 expected failures (LED-related, derived from `APP_LED` being disabled) |
| Crash tests (assert / null deref / divzero / stack overflow) | ✅ 4/4 |
| Driver tests (battery / IMU / I2C) | ✅ 6/6 |
| Sound tests | ✅ 9/9 |
| System tests (watchdog / cpuload / stackmonitor) | ✅ 3/3 |

## Known follow-ups

- **`ostest` aborts mid-run** ([#38](https://github.com/owhinata/spike-nx/issues/38)): launching `ostest` causes the serial port to drop and the board to reset. `sleep 10` is fine, so this is not a simple watchdog fire — more likely a specific subtest hits an assertion or HardFault that triggers the `BOARD_RESET_ON_ASSERT=2` reset. The same test passes on `stm32f413-discovery/knsh`, so an interaction with SPIKE-Hub-specific peripherals (battery / IMU / sound) during init is suspected.
- **APP_LED refactor** ([#39](https://github.com/owhinata/spike-nx/issues/39)): replace the `tlc5955_set_duty()` direct call with a `/dev/rgbled`-style char driver.
- **ARCH_PERF_EVENTS syscall** ([#40](https://github.com/owhinata/spike-nx/issues/40)): register `perf_gettime()` as a syscall, or rewrite the ostest helper on top of `clock_gettime`.

## References

- Upstream BUILD_PROTECTED reference: `nuttx/boards/arm/stm32/stm32f4discovery/configs/kostest/`
- STM32F4 MPU init: `nuttx/arch/arm/src/stm32/stm32_mpuinit.c`
- Heap partitioning: `nuttx/arch/arm/src/stm32/stm32_allocateheap.c`
- Kconfig: `nuttx/Kconfig` (BUILD_PROTECTED / PASS1_* / NUTTX_USERSPACE)
- Debug scaffold: `boards/stm32f413-discovery/configs/knsh/`
