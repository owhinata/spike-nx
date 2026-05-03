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

### SRAM (at 0x20000000, 320 KB) — post Issue #98

| Region | Range | Size | Purpose |
|---|---|---|---|
| `ksram` | `0x20000000..0x20010000` | 64K | Kernel .data/.bss + idle stack (linker only); ksram tail above `g_idle_topstack` is reused as kernel heap |
| _kheap slot_ | `0x20010000..0x20020000` | 64K | Runtime kernel heap slot (NOT in the linker MEMORY block; `up_allocate_kheap` returns `g_idle_topstack..0x20020000` which merges the ksram tail with this slot into a single contiguous kernel heap) |
| `usram` | `0x20020000..0x20040000` | 128K | User .data/.bss + heap tail. 128K-aligned. `stm32_mpuinitialize` exposes the full 128 KB as a single MPU region so the .bss / heap boundary can slide |
| `xsram` | `0x20040000..0x20050000` | 64K | User heap top portion (64K-aligned MPU region added by `up_allocate_heap` so the user heap is one contiguous span from `us_bssend` to SRAM1_END) |

`CONFIG_MM_KERNEL_HEAPSIZE = 32768` (lower bound; the dedicated 64 KB slot satisfies it even if the ksram tail were ever consumed entirely by the idle stack — asserted at compile time in `stm32_allocateheap.c`).

Both heaps use **sliding boundaries**:

- **Kernel heap** = `g_idle_topstack..0x20020000` (~96 KB). Dedicated 64 KB slot plus the ksram tail above the idle stack.
- **User heap** = `us_bssend..0x20050000` (~136 KB at the current `.bss = 56 KB`). The MPU usram region (128 KB) covers the in-usram portion; the MPU xsram-top region (64 KB) covers the rest. Both regions are user RW, so single allocations crossing the `0x20040000` boundary are transparent.

User `.data + .bss` budget is **128 KB** (was 64 KB before #98). Link-time `ASSERT(_ebss <= 0x20040000)` in `user-space.ld` enforces the boundary.

Required CONFIGs:

- `CONFIG_MM_REGIONS=2`
- `CONFIG_STM32_CCMEXCLUDE=y` (the F413 has no CCM SRAM)
- `CONFIG_BOARD_SPIKE_PRIME_HUB=y` (always-on identifier injected into stm32 common code so the SPIKE-only heap path is selectable from a preprocessor symbol)

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

The owhinata fork (`f413-support-12.13.0` branch) carries these commits:

| Commit | Contents |
|---|---|
| `97716f5a2a` | `arch/armv7-m`: preserve the caller's `r11` across `arm_dispatch_syscall` (the BUILD_PROTECTED syscall path was breaking AAPCS) |
| `7c116a6de2` | `arch/arm`: disable `fork()` support under BUILD_PROTECTED (kernel/user stacks cannot be shared across the boundary) |
| Issue #98 (post-`a277a13`) | `arch/stm32` (this branch): SPIKE Prime Hub-specific heap path in `stm32_allocateheap.c` + `stm32_mpuinit.c` — pinned to `CONFIG_BOARD_SPIKE_PRIME_HUB`. The earlier `b35c473a58` (tail-region recovery via `arm_addregion`) is **superseded** by this path; the log2 alignment loop and the tail-add block are removed. The new path returns a single contiguous user heap from `us_bssend` to `SRAM1_END`, hardcodes a 128 KB MPU region for usram, and uses `g_idle_topstack` as the kernel heap base so the ksram tail is reclaimed. |

## Related changes

| Item | Action | Reason |
|---|---|---|
| `CONFIG_APP_LED` enabled | Fixed by Issue #39 (2026-04-19) | Refactored to use the `/dev/rgbled0` char driver (ioctl) so the user blob can drive TLC5955 |
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

## Hardware verification (2026-05-03, post Issue #98)

| Item | Result |
|---|---|
| NSH boot (USB CDC) | ✅ `NuttShell (NSH) NuttX-12.13.0` prompt reached |
| `free` Umem | ✅ 138 856 B (~136 K) — `us_bssend..0x20050000` sliding |
| `free` Kmem | ✅ 98 648 B (~96 K) — `g_idle_topstack..0x20020000` sliding |
| `heap 0x1f000` (124 KB single alloc, MPU boundary crossing) | ✅ allocator-served from one contiguous region |
| LSM6DSL boot init (Issue #95 regression check) | ✅ no `-110 ETIMEDOUT` |
| User `.data + .bss` budget | ✅ 128 KB usram (was 64 KB) — link-time `ASSERT(_ebss <= 0x20040000)` enforces the boundary |

## Hardware verification (2026-04-18, commit `55347a8`, pre Issue #98)

| Item | Result |
|---|---|
| NSH boot (USB CDC) | ✅ `NuttShell (NSH) NuttX-12.13.0` prompt reached |
| `free` Umem | ✅ 196 608 B (192 K) — tail recovery delivers the planned size |
| `free` Kmem | ✅ 62 536 B |
| CoreMark 2000 iter | ✅ **170.46 iter/sec** |
| `sleep 10` (watchdog interaction) | ✅ Returns after 10.0 s, no watchdog fire |
| `pytest -m "not slow and not interactive"` | ✅ **30 passed** (LED tests also pass after Issue #39 re-enabled `APP_LED`) |
| Crash tests (assert / null deref / divzero / stack overflow) | ✅ 4/4 |
| Driver tests (battery / IMU / I2C) | ✅ 6/6 |
| Sound tests | ✅ 9/9 |
| System tests (watchdog / cpuload / stackmonitor) | ✅ 3/3 |

## Known follow-ups

- **`ostest` aborts mid-run** ([#38](https://github.com/owhinata/spike-nx/issues/38)): launching `ostest` causes the serial port to drop and the board to reset. `sleep 10` is fine, so this is not a simple watchdog fire — more likely a specific subtest hits an assertion or HardFault that triggers the `BOARD_RESET_ON_ASSERT=2` reset. The same test passes on `stm32f413-discovery/knsh`, so an interaction with SPIKE-Hub-specific peripherals (battery / IMU / sound) during init is suspected.
- **ARCH_PERF_EVENTS syscall** ([#40](https://github.com/owhinata/spike-nx/issues/40)): register `perf_gettime()` as a syscall, or rewrite the ostest helper on top of `clock_gettime`.

## References

- Upstream BUILD_PROTECTED reference: `nuttx/boards/arm/stm32/stm32f4discovery/configs/kostest/`
- STM32F4 MPU init: `nuttx/arch/arm/src/stm32/stm32_mpuinit.c`
- Heap partitioning: `nuttx/arch/arm/src/stm32/stm32_allocateheap.c`
- Kconfig: `nuttx/Kconfig` (BUILD_PROTECTED / PASS1_* / NUTTX_USERSPACE)
- Debug scaffold: `boards/stm32f413-discovery/configs/knsh/`
