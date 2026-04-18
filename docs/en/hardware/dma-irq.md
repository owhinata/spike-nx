# DMA Stream Allocation and IRQ Priority

## Overview

DMA stream allocation and NVIC IRQ priority design for the SPIKE Prime Hub (STM32F413). Based on the pybricks v3.6.1 implementation, adapted to NuttX IRQ management constraints.

## DMA Stream Allocation

Based on RM0430 Table 27/28 (DMA request mapping).

### DMA1

| Stream | Channel | Peripheral | Status | Priority |
|--------|---------|-----------|--------|----------|
| S3 | Ch0 | Flash SPI2 RX | Not implemented (pybricks reserved) | HIGH |
| S4 | Ch0 | Flash SPI2 TX | Not implemented (pybricks reserved) | HIGH |
| S5 | Ch7 | **DAC1 CH1 (Sound)** | ✅ Implemented | **HIGH** |
| S6 | Ch4 | BT UART2 TX | Not implemented (pybricks reserved) | VERY_HIGH |
| S7 | Ch6 | BT UART2 RX | Not implemented (pybricks reserved) | VERY_HIGH |

### DMA2

| Stream | Channel | Peripheral | Status | Priority |
|--------|---------|-----------|--------|----------|
| S0 | Ch0 | **ADC1** | ✅ Implemented | **MEDIUM** |
| S2 | Ch3 | **TLC5955 SPI1 RX** | ✅ Implemented (NuttX SPI) | LOW |
| S3 | Ch3 | **TLC5955 SPI1 TX** | ✅ Implemented (NuttX SPI) | LOW |

### USB OTG FS

FIFO-based. No DMA used.

## NVIC IRQ Priority

### STM32F413 NVIC Specification

- Priority bits: 4 (16 levels, 0x00 = highest / 0xF0 = lowest)
- Step: `NVIC_SYSH_PRIORITY_STEP = 0x10`
- pybricks priority group: `NVIC_PRIORITYGROUP_4` (preempt 4-bit / sub 0-bit) — subpriority is effectively ignored
- NuttX default: `NVIC_SYSH_PRIORITY_DEFAULT = 0x80` (level 8)

### Critical-Section Mechanism Difference

pybricks and NuttX disable interrupts over fundamentally different scopes.

| Aspect | pybricks | NuttX (current SPIKE Prime Hub) |
|---|---|---|
| API | `__disable_irq()` (`clock_stm32.c:31`, `reset_stm32.c:63`, etc.) | `up_irq_save()` (`armv7-m/irq.h`) |
| Masking mechanism | **PRIMASK** — blocks all IRQs (except NMI/HardFault) | **BASEPRI = 0x80** — masks only IRQs with priority `>= 0x80` |
| Effect | IRQs at any priority are blocked during critical sections | IRQs with priority `< 0x80` still run during critical sections (but cannot call NuttX APIs) |
| SVCall | 0x00 (highest) — PRIMASK-exempt by design | `NVIC_SYSH_SVCALL_PRIORITY = 0x70` — runs above BASEPRI |

!!! warning "BASEPRI implications"
    On NuttX, if an IRQ is configured with priority `< 0x80`, its ISR keeps running during critical sections — but **the ISR must not call any NuttX API**. Such ISRs must not touch NuttX shared data; they can only raise signals/events, following the `CONFIG_ARCH_HIPRI_INTERRUPT` convention.

### Background: why NuttX APIs are unsafe in ISRs above BASEPRI

NuttX kernel protects shared data (scheduler ready/wait queues, semaphore waiter lists, work queues) via `enter_critical_section()`, implemented internally as `BASEPRI = 0x80` to mask all IRQs with priority value `>= 0x80`.

When an ISR is configured with priority `< 0x80`, it **bypasses** the kernel's critical section and can preempt kernel code mid-operation. If that ISR calls any NuttX API:

```
kernel thread          priority-0x60 ISR
--------------         -----------------
enter_critical()       (not yet firing)
BASEPRI = 0x80
  modifying wait
  queue list...      ← ISR at 0x60 preempts (0x60 < 0x80, not masked)
                      calls nxsem_post_slow()
                        enter_critical() (no-op, already 0x80)
                        modifies same wait queue ← RACE
                      ISR returns
  ...continues         ← list corrupted
leave_critical()
```

`nxsem_post` has two code paths:

- **Fast path** — atomic `cmpxchg` increments the counter. Used when no waiters exist. ISR-safe at any priority.
- **Slow path** — `nxsem_post_slow()` enters the critical section and **walks the wait-queue linked list**. Whenever at least one thread is waiting on the semaphore, execution enters this path.

As soon as a waiter exists, the slow path runs and races against the preempted kernel code.

### Audit: NuttX driver ISRs and NuttX API usage

| IRQ | pybricks value | ISR path | Calls `nxsem_post`? | Safe above BASEPRI? |
|---|---|---|---|---|
| IMU I2C2 ER/EV | 0x30 | `stm32_i2c_isr` → `nxsem_post(&priv->sem_isr)` | Yes (slow path certain) | ❌ |
| Sound DMA1S5 | 0x40 | `stm32_dmainterrupt` (callback=NULL) | No | ✅ |
| USB OTG FS | 0x60 | `stm32_usbinterrupt` → `cdcacm_rdcomplete` → `uart_wakeup` → `nxsem_post` | Yes (slow path certain) | ❌ |
| ADC DMA2S0 | 0x70 | `stm32_dmainterrupt` (callback=NULL) | No | ✅ |
| TLC5955 SPI1 DMA | 0x70 | `spi_dmarx/txcallback` → `nxsem_post` | Yes (slow path certain) | ❌ |

USB/I2C/SPI cannot be raised above BASEPRI without modifying the NuttX upstream drivers.

### Note: `BASEPRI = 0` is a special "masking disabled" value

"Setting BASEPRI to 0 to defeat critical sections" is **not a solution**. On ARM Cortex-M, `BASEPRI = 0` is a special value meaning **"no priority masking at all"**. No IRQ is masked — critical sections provide zero protection, and NuttX kernel shared data can be corrupted by any IRQ.

### Note: IRQ priorities are expressed as absolute values (Cortex-M NVIC constraint)

The Cortex-M NVIC stores each IRQ's priority as an **absolute value** (STM32F4 uses the top 4 bits only, giving 16 levels from 0x00 to 0xF0). There is **no hardware mechanism that computes relative ordering** — priority grouping (PRIGROUP) only divides the value into preempt/sub-priority bit fields.

NuttX still allows expressing priorities **relatively in source code** via the following constants:

| Constant | Value | Meaning |
|---|---|---|
| `NVIC_SYSH_PRIORITY_MAX` | 0x00 | Highest priority |
| `NVIC_SYSH_PRIORITY_DEFAULT` | 0x80 | Midpoint (also the BASEPRI boundary) |
| `NVIC_SYSH_PRIORITY_MIN` | 0xF0 | Lowest priority |
| `NVIC_SYSH_PRIORITY_STEP` | 0x10 | One step |

The current `stm32_adc_dma.c` writes `DEFAULT + 3 * STEP` (= 0xB0), which reads as "3 steps below default".

### pybricks IRQ Priority Inventory (Prime Hub)

`HAL_NVIC_SetPriority(IRQ, preempt, sub)` — `sub` is effectively a no-op under `NVIC_PRIORITYGROUP_4`. Since pybricks uses PRIMASK to blanket-mask all IRQs, absolute priority only affects microsecond-scale response differences. The relative ordering is what reflects design intent.

| preempt | NVIC value | IRQ | Source |
|---|---|---|---|
| 0 | 0x00 | LUMP UART (UART4/5/7-10) | `uart_stm32f4_ll_irq.c:283` |
| 1 | 0x10 | Bluetooth UART TX DMA (sub=2) | `bluetooth_btstack_uart_block_stm32_hal.c:98` |
| 1 | 0x10 | Bluetooth UART RX DMA (sub=1) | ibid :100 |
| 1 | 0x10 | Bluetooth USART2 (sub=0) | ibid :102 |
| 3 | 0x30 | IMU I2C2_ER (sub=1) | `platform.c:188` |
| 3 | 0x30 | IMU I2C2_EV (sub=2) | `platform.c:190` |
| 3 | 0x30 | IMU EXTI4 (INT1, sub=3) | `platform.c:200` |
| **4** | **0x40** | **Sound DAC DMA (DMA1_S5)** | `sound_stm32_hal_dac.c:70` |
| 5 | 0x50 | Flash W25QXX SPI2 DMA (tx/rx) | `block_device_w25qxx_stm32.c:525, 542` |
| **6** | **0x60** | **USB OTG FS** | `platform.c:963` |
| 6 | 0x60 | USB VBUS (EXTI9_5, sub=1) | `platform.c:965` |
| 6 | 0x60 | Flash W25QXX SPI2 IRQ (sub=2) | `block_device_w25qxx_stm32.c:556` |
| 7 | 0x70 | ADC DMA (DMA2_S0) | `adc_stm32_hal.c:147` |
| 7 | 0x70 | TLC5955 LED SPI1+DMA (sub=0-2) | `pwm_tlc5955_stm32.c:197, 214, 234` |
| 15 | 0xF0 | SysTick (HAL tick) | `stm32f4xx_hal_conf.h:153` (`TICK_INT_PRIORITY`) |
| 15 | 0xF0 | RNG | `random_stm32_hal.c:54` |

### NuttX IRQ Priority Design (adopted: option ε)

Compresses the pybricks **relative** priority order into the 0x80–0xF0 band (below BASEPRI) so every IRQ stays at or below `NVIC_SYSH_DISABLE_PRIORITY (0x80)`. Every ISR can safely call NuttX APIs (including `nxsem_post`'s slow path), and the NuttX upstream USB/I2C/SPI drivers run unmodified. All priority assignments are centralised in a single block at the top of `stm32_bringup.c`.

| NVIC value | Level | Peripheral | pybricks mapping | Configured in |
|---|---|---|---|---|
| 0x80 | 8 | TIM9 tickless tick | (pybricks SysTick 0xF0) | NuttX default (kept) |
| 0x90 | 9 | IMU I2C2 EV/ER + EXTI4 | base=3 | `stm32_bringup.c` (step 6) |
| 0xA0 | 10 | Sound DAC DMA1_S5 | base=4 (HIGH) | `stm32_bringup.c` (step 5) |
| 0xB0 | 11 | USB OTG FS | base=6 | `stm32_bringup.c` (step 4) |
| 0xD0 | 13 | ADC DMA2_S0 | base=7 (MEDIUM) | `stm32_bringup.c` (step 3) |
| 0xD0 | 13 | TLC5955 SPI1 + DMA2_S2/S3 | base=7 (LOW) | `stm32_bringup.c` (step 2) |
| 0xF0 | 15 | PendSV, SysTick | base=15 | `stm32_bringup.c` (step 1) |

!!! success "Why this was adopted"
    - Preserves pybricks's relative priority order (IMU > Sound > USB > ADC = TLC5955 > PendSV/SysTick).
    - Every IRQ sits at or below BASEPRI ⇒ no `nxsem_post` slow-path race from any ISR; NuttX USB/I2C/SPI upstream drivers work unchanged.
    - No NuttX core change required — everything lives in board code (`stm32_bringup.c`).
    - Resolves Issue #36: full regression remains at 30/30 pass across all six staged steps.

!!! note "Why TIM9 stays at 0x80"
    With `CONFIG_SCHED_TICKLESS=y` + `CONFIG_STM32_TICKLESS_TIMER=9`, TIM9 drives the scheduler tick. It must stay at the highest priority (within ε) to keep scheduling responsive. The TIM9 ISR is short (tickless timer math only), so it does not starve lower-priority peripherals.

### Actual Behavior of `CONFIG_ARCH_IRQPRIO`

After auditing `nuttx/arch/arm/src/stm32/stm32_irq.c`, **`CONFIG_ARCH_IRQPRIO` does not change the NVIC initialization logic during board boot**. The Kconfig option only gates the **definition** of `up_prioritize_irq()`. `up_irqinitialize()` always initializes every IRQ to `NVIC_SYSH_PRIORITY_DEFAULT (0x80)` regardless of this setting.

The ε implementation turns `CONFIG_ARCH_IRQPRIO=y` on in defconfig and uses the `#ifdef CONFIG_ARCH_IRQPRIO` block in `stm32_bringup.c` to call `up_prioritize_irq()` ten times (system handlers + five peripheral groups), overwriting each NVIC priority register individually.

### DMA Priority (DMA_SCR PL Field)

Separate from NVIC priority, this controls arbitration within a single DMA controller when multiple streams request simultaneously.

| DMA Priority | Peripheral | Configured in |
|---|---|---|
| HIGH | Sound DMA1_S5 | `stm32_sound.c` (DMA_SCR_PRIHI) |
| MEDIUM | ADC DMA2_S0 | `stm32_adc_dma.c` (DMA_SCR_PRIMED) |
| LOW | TLC5955 SPI1 | NuttX SPI driver (SPI_DMA_PRIO) |

## Timer Allocation

| Timer | Purpose | Status | pybricks Usage |
|---|---|---|---|
| TIM1 | Motor PWM (Port A/B) | Not implemented | 12 kHz PWM, PSC=8, ARR=1000 |
| TIM2 | ADC trigger (TRGO 1 kHz) | ✅ Implemented | Same |
| TIM3 | Motor PWM (Port E/F) | Not implemented | 12 kHz PWM |
| TIM4 | Motor PWM (Port C/D) | Not implemented | 12 kHz PWM |
| TIM5 | Charger ISET PWM | In defconfig, not implemented | 96 kHz, CH1 |
| TIM6 | DAC sample rate (TRGO) | ✅ Implemented | Same |
| TIM8 | BT 32.768 kHz clock | Not implemented | CH4 PWM |
| TIM9 | NuttX tickless timer | ✅ Implemented | (pybricks uses SysTick) |
| TIM12 | TLC5955 GSCLK | ✅ Implemented | Same (CH2, ~8.7 MHz) |

## Resolution: Issue #36 — option ε adopted

Adding `CONFIG_ARCH_IRQPRIO=y` to `defconfig` used to cause the USB CDC/ACM device to drop after 7-8 tests during a full regression run, with subsequent tests failing with `OSError: [Errno 6] Device not configured`. Investigation traced this to **the structure created by the `stm32_adc_dma.c` `up_prioritize_irq(..., 0xB0)` call**, not simply "that particular value is wrong".

### Investigation timeline

1. **Bisection step 1 (2026-04-18)**: With the `stm32_adc_dma.c` `up_prioritize_irq()` call wrapped in `#if 0`, the regression passes 30/30. ADC DMA priority change is the direct trigger.
2. **Pivot**: rather than continuing a value-level bisection, port the pybricks priority design (Sound > USB > ADC ordering) via **option ε**.
3. **Staged ε implementation (lowest-priority first, steps 1–6)**:
   - step 1: PendSV + SysTick → 0xF0 ✅
   - step 2: TLC5955 SPI1 + DMA2S2/S3 → 0xD0 ✅
   - step 3: ADC DMA2S0 → 0xD0 ✅ (the 0xB0 value broke USB, but 0xD0 combined with step 1+2 does not)
   - step 4: USB OTG FS → 0xB0 ✅
   - step 5: Sound DAC DMA1S5 → 0xA0 ✅
   - step 6: IMU I2C2 EV/ER + EXTI4 → 0x90 ✅
4. After every step the regression remains 30/30 pass with CoreMark ~170. Issue #36 is resolved.

### Root-cause reasoning

The fact that `0xB0` alone fails while `0xD0` (together with PendSV/SysTick/TLC5955 lowered) passes implies:

- **Not** "ADC DMA being lower than other IRQs" per se (in ε, ADC is still lower than USB).
- **Likely** related to the interaction between PendSV (default 0x80) and peripheral IRQs:
  - Before ε: ADC 0xB0 < PendSV 0x80 (= everything else at 0x80). ADC ISR could always be preempted by a PendSV scheduled from within a peripheral ISR.
  - After ε: PendSV 0xF0 < every peripheral. Deferred context switch runs only after all pending ISRs tail-chain — the standard Cortex-M pattern.
- A precise root cause remains future work; empirically ε provides a stable configuration.

### Options not adopted (kept for reference)

Multiple remediation paths were evaluated. Below is the record of alternatives.

#### Option A: Revert ADC DMA priority to default (0x80) — not adopted

Remove the `up_prioritize_irq()` call in `stm32_adc_dma.c` and drop `CONFIG_ARCH_IRQPRIO=y` from `defconfig`.

- Pros: Minimal change, zero regression risk, full 30/30 pass.
- Cons: **pybricks's relative priority design (ADC=7 < Sound=4) cannot be expressed in NuttX**. When motor/BT are added later, leaving every IRQ at 0x80 is design-inconsistent.
- Verdict: Abandons the pybricks design. Rejected because ε achieves the same relative order without this trade-off.

#### Option B: Raise USB OTG FS to just below BASEPRI (e.g. 0x70) — not adopted

Following pybricks's USB=0x60 design, raise the USB OTG FS IRQ priority to just below BASEPRI so ADC DMA at 0xB0 can be preempted by USB, improving USB responsiveness.

- Pros: Faithfully reproduces pybricks's design intent (USB > ADC).
- Cons:
  - The NuttX USB CDC ISR (`stm32_usbinterrupt` in `stm32_otgfsdev.c`) internally calls `usbdev_*` APIs and work-queue submissions. Whether these stay race-free when invoked past BASEPRI needs verification.
  - 0x70 equals `NVIC_SYSH_SVCALL_PRIORITY`. Putting USB IRQ at the same priority as SVCall introduces possible races during context switches — needs analysis.
  - IRQs above BASEPRI fall into `CONFIG_ARCH_HIPRI_INTERRUPT` territory: NuttX critical sections do not protect them. Unless USB CDC TX/RX buffer locking already uses spinlocks/atomics, it will break.
- Verdict: **Decide only after auditing internal locking in the USB driver**. Do not raise lightly.

#### Option C: Raise NuttX BASEPRI — not adopted

Bumping `NVIC_SYSH_DISABLE_PRIORITY` to, say, 0xA0 would change critical-section semantics so that 0xB0 < 0xA0.

- Pros: Architecturally consistent.
- Cons: Changes `nuttx/arch/arm/include/armv7-m/nvicpri.h` — **invasive to NuttX core**, diverges from upstream. Violates the project policy of minimal changes to the nuttx submodule.
- Verdict: Rejected.

#### Option D: Drop NVIC-level ADC DMA demotion; rely on DMA PL only — not adopted

Keep every IRQ at `NVIC_SYSH_PRIORITY_DEFAULT` and express the Sound-vs-ADC priority difference only through the DMA_SCR PL field (Sound=HIGH, ADC=MEDIUM).

- Pros: No NVIC changes.
- Cons: Same trade-off as Option A — defers the problem to a future motor/BT expansion.
- Verdict: Equivalent to Option A.

#### Option ε: Compress the pybricks relative order into 0x80–0xF0 (**adopted**)

Rather than porting pybricks's **absolute values** (0x00–0xF0 full range), preserve only the **relative order** and fit every IRQ at priority >= 0x80. Every IRQ is then automatically masked during critical sections, so even the NuttX USB/I2C/SPI drivers (which call APIs from their ISRs) remain safe.

See the "NuttX IRQ Priority Design (adopted: option ε)" section above for the final mapping.

- Pros:
  - Fully preserves pybricks's **relative priority order**.
  - Every IRQ is `>= BASEPRI`, so NuttX APIs remain safe in every ISR — the upstream USB/I2C/SPI drivers work unchanged.
  - Requires no NuttX core change; can be implemented entirely in board-level code (single block in `stm32_bringup.c`).
- Cons:
  - Only 8 priority levels available (0x80–0xF0). Seven are already claimed, so further expansion (motor/BT) may need revisiting.
  - Approaches a "pybricks-like PRIMASK" behaviour in practice, abandoning the low-latency advantage of NuttX's BASEPRI model.
- Verdict: **Adopted**. The staged rollout (lowest-priority first, 30/30 pass at every step) confirmed that Issue #36 does not recur.

#### Option C1: Lower NuttX's BASEPRI alone (override `NVIC_SYSH_DISABLE_PRIORITY`) — deferred

Reduce BASEPRI to ~0x30 so the pybricks absolute values (IMU 0x30, Sound 0x40, USB 0x60, ADC 0x70) all fall within the BASEPRI-masked range. pybricks values can then be ported verbatim.

```c
/* 1-line patch in nvicpri.h, or board-header override */
#undef NVIC_SYSH_DISABLE_PRIORITY
#define NVIC_SYSH_DISABLE_PRIORITY 0x30
```

- Pros: Faithful port of pybricks absolute values. NuttX APIs safe in every peripheral ISR.
- Cons:
  - Invasive change to NuttX core (`armv7-m/nvicpri.h`).
  - Breaks the relationship with `NVIC_SYSH_SVCALL_PRIORITY` (currently 0x70); needs coordinated adjustment.
  - All IRQs at `>= 0x30` are masked during critical sections → worse latency than today.
  - Diverges from upstream; maintenance burden on each submodule update.

#### Option δ: Switch NuttX's critical section to PRIMASK — rejected

Replace `up_irq_save()` from a BASEPRI-based to a `__disable_irq()`-based implementation — same model as pybricks.

- Pros: Fully decouples IRQ priority from ISR-API safety (pybricks-equivalent).
- Cons:
  - Large invasive NuttX change (`armv7-m/irq.h` rewrite).
  - All IRQs blocked during critical sections → latency regression across the board.
  - Overturns the design intent behind NuttX's BASEPRI model.
- Verdict: Rejected — out of scope for this issue.

### Future expansion considerations

- **Eight priority levels are available** (0x80–0xF0, `STEP=0x10`). Seven are claimed today (TIM9 / IMU / Sound / USB / ADC+TLC5955 / PendSV+SysTick). Motor PWM or Bluetooth may exhaust the band.
- Escalation options at that point:
  - Share a level between IRQs that never need to preempt each other (as TLC5955 and ADC already do).
  - Activate option C1 (lower BASEPRI to e.g. 0x30) to widen the usable band to 0x30–0xF0 (13 levels).
- When adding a new peripheral IRQ, consult the pybricks relative ordering and place it in the appropriate ε slot by appending to the `stm32_bringup.c` epsilon block.

## Work-queue thread priorities (HPWORK / LPWORK)

When an ISR defers processing through `work_queue(HPWORK, ...)` or `work_queue(LPWORK, ...)`, the work runs in a kernel thread at one of two priorities. These thread priorities live on a **separate axis** from NVIC priorities (NVIC: lower number = higher; NuttX scheduler: higher number = higher). The only relationship that matters is who the deferred work has to share CPU time with after the ISR has returned.

### Chosen values

| Worker | NuttX PRI (1-255) | Setting |
|---|---|---|
| `hpwork` | 224 (RR) | `CONFIG_SCHED_HPWORKPRIORITY=224` (NuttX default) |
| `lpwork` | **176** (RR) | `CONFIG_SCHED_LPWORKPRIORITY=176` (explicit override) |
| `lpwork` PI ceiling | 176 | `CONFIG_SCHED_LPWORKPRIOMAX=176` (NuttX default) |

`SCHED_PRIORITY_DEFAULT = 100` (`<sys/types.h>`), so `nsh_main` / `imu` / `sound` / `coremark` and other builtin apps run at PRI=100 by default.

### Why LPWORK was raised from 100 to 176

With NuttX's default `LPWORKPRIORITY=100`, LPWORK shares a priority with user apps, which causes:

- The LPWORK thread time-slices against any CPU-hungry user app at the `RR_INTERVAL=200 ms` quantum
- BCD detection and other non-time-critical defers tolerate this, but the moment any future ISR offloads time-sensitive work to LPWORK, user-space CPU load directly delays it

176 is the existing `LPWORKPRIOMAX` ceiling and sits between HPWORK (224) and user apps (100). With this change:

- LPWORK **always preempts** user apps (low-latency defer)
- HPWORK (224) still preempts LPWORK (the most time-critical defers stay on HPWORK)
- The effective ceiling already reached 176 through priority inheritance, so pinning the base to 176 does not change the upper bound

### Driver defer routing

| Source | NVIC IRQ | Defer target | Reason |
|---|---|---|---|
| TIM9 (tickless tick) | 0x80 | — | Handled inside the tick handler |
| LSM6DSL DRDY (I2C2 / EXTI4) | 0x90 | **HPWORK** (224) | Pull sensor sample on the highest-priority worker |
| Sound DAC DMA1_S5 | 0xA0 | (DMA half/full callback, no work queue) | Ring-buffer refill stays inside the ISR |
| USB OTG FS | 0xB0 | (driver-internal) | — |
| ADC DMA2_S0 | 0xD0 | (callback into battery driver) | — |
| TLC5955 SPI1 + DMA2_S2/S3 | 0xD0 | **HPWORK** (224) | Maintain 2 ms LED frame-sync cadence |
| Battery charger poll | (timer) | **HPWORK** (224) | Periodic VBUS state monitor |
| Battery charger BCD detect | (re-scheduled from HPWORK) | **LPWORK** (176) | BCD includes blocking I2C — push it off the HPWORK band |
| Power button monitor | (timer) | **HPWORK** (224) | Periodic poll |
| PendSV / SysTick | 0xF0 | — | — |

### Verification points

- `nsh> ps` shows `hpwork` PRI=224 and `lpwork` PRI=176
- The LPWORK bump was added to `boards/spike-prime-hub/configs/usbnsh/defconfig` as a follow-up to Issue #37 (BUILD_PROTECTED). It is **not** propagated to the alternate `nsh` defconfig — extend it there too if needed.
- No regression against Issue #36 (NVIC) or Issue #37 (BUILD_PROTECTED). Verified with `pytest -m "not slow and not interactive"`.

## Future Expansion

When implementing Motor / Bluetooth / Flash, follow the pybricks DMA/IRQ allocation directly. Note that pybricks sets Bluetooth to NVIC level 1 (DMA VERY_HIGH), but NuttX's BASEPRI constraint may force it to 0x80. Whether this impacts BT communication quality must be verified at implementation time. The Issue #36 discussion above should be re-opened concurrently.

## References

- RM0430 Table 27: DMA1 request mapping
- RM0430 Table 28: DMA2 request mapping
- pybricks `lib/pbio/platform/prime_hub/platform.c`
- pybricks `lib/pbio/drv/sound/sound_stm32_hal_dac.c`
- pybricks `lib/pbio/drv/adc/adc_stm32_hal.c`
- pybricks `lib/pbio/drv/pwm/pwm_tlc5955_stm32.c`
- pybricks `lib/pbio/drv/bluetooth/bluetooth_btstack_uart_block_stm32_hal.c`
- pybricks `lib/pbio/drv/block_device/block_device_w25qxx_stm32.c`
- pybricks `lib/pbio/drv/uart/uart_stm32f4_ll_irq.c`
- NuttX `arch/arm/src/stm32/stm32_irq.c`
- NuttX `arch/arm/include/armv7-m/nvicpri.h`
