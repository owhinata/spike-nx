# BUILD_PROTECTED Migration

Tracking Issue #37 — migrating to Cortex-M4 MPU-based kernel / user space separation. **Status: WIP (builds succeed, hardware boot unverified).**

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

## Implementation files

| Path | Contents |
|---|---|
| `boards/spike-prime-hub/configs/usbnsh/defconfig` | Adds `CONFIG_BUILD_PROTECTED=y`, MPU, USERSPACE, etc. |
| `boards/spike-prime-hub/kernel/Makefile` | PASS1 (user blob) build rules |
| `boards/spike-prime-hub/kernel/stm32_userspace.c` | `struct userspace_s` at user blob origin |
| `boards/spike-prime-hub/kernel/CMakeLists.txt` | Same, for CMake |
| `boards/spike-prime-hub/scripts/memory.ld` | Physical memory layout |
| `boards/spike-prime-hub/scripts/kernel-space.ld` | Kernel blob section placement |
| `boards/spike-prime-hub/scripts/user-space.ld` | User blob section placement |
| `boards/spike-prime-hub/scripts/Make.defs` | `ARCHSCRIPT` now uses `memory.ld + kernel-space.ld` |
| `apps/imu/*.c` | Replaced `clock_systime_ticks()` with POSIX `clock_gettime(CLOCK_MONOTONIC)` (kernel symbol replaced by a user-callable syscall) |

## Related changes

| Item | Action | Reason |
|---|---|---|
| `CONFIG_APP_LED` disabled | Temporary | `led_main.c` calls `tlc5955_set_duty()` directly — needs refactor to a `/dev/rgbled`-style char driver |
| `CONFIG_ARCH_PERF_EVENTS` disabled | — | `nuttx-apps/testing/ostest/perf_gettime.c` calls `perf_gettime()` which has no syscall proxy |
| `CONFIG_STM32_IWDG`, `CONFIG_WATCHDOG_AUTOMONITOR` disabled | Debugging | Turned off while isolating the early crash; re-evaluate once BUILD_PROTECTED boots |

## Build

```bash
make nuttx-distclean && make
```

Artifacts:

- `nuttx/nuttx.bin` (~138 KB) — kernel blob @ `0x08008000`
- `nuttx/nuttx_user.bin` (~146 KB) — user blob @ `0x08080000`

## Flash (two steps)

```bash
# Enter DFU: unplug USB, hold BT button, plug USB, wait 5 s
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
```

## Open issue (2026-04-18)

The build passes cleanly and both blobs flash via DFU, but **the device never enumerates USB CDC** after reboot — the LED blinks yellow at ~1 Hz, consistent with a crash→reset loop driven by `CONFIG_BOARD_RESET_ON_ASSERT=2`.

### Things already tried (no improvement)

- Added `CONFIG_ARM_MPU_EARLY_RESET=y` / `CONFIG_ARM_MPU_RESET=y`
- Disabled `CONFIG_WATCHDOG_AUTOMONITOR`
- Disabled `CONFIG_STM32_IWDG`
- Minimal build (all apps off, drivers stripped, `CONFIG_BOARD_LATE_INITIALIZE` disabled) — same symptom

### Diagnostic constraints

The SPIKE Prime Hub SWD pads are internal to the case and difficult to access. The only console is CDCACM, and the crash happens before USB enumeration, so `syslog` / `RAMLOG` output never reaches the host. Blind bisection without a live-debug channel is no longer productive.

### Plan forward

1. Reproduce on **STM32F413 Discovery** (ST eval board, SWD readily accessible)
2. Locate the crash point there (GDB over SWD, or printf via USART)
3. Port the fix back to the SPIKE Hub-specific parts (memory map, LEGO bootloader interaction)

### Work preservation

This implementation lives on the `feat/build-protected` branch. It is **not merged to main**; we will resume after SWD-based debugging.

## References

- Upstream BUILD_PROTECTED reference: `nuttx/boards/arm/stm32/stm32f4discovery/configs/kostest/`
- STM32F4 MPU init: `nuttx/arch/arm/src/stm32/stm32_mpuinit.c`
- Heap partitioning: `nuttx/arch/arm/src/stm32/stm32_allocateheap.c`
- Kconfig: `nuttx/Kconfig` (BUILD_PROTECTED / PASS1_* / NUTTX_USERSPACE)
