# Tickless Timer Configuration

This document describes the NuttX tickless scheduler timer configuration on the SPIKE Prime Hub and explains the behavior when using 16-bit timers.

## Configuration

| Item | Setting |
|------|---------|
| Mode | Tickless (`CONFIG_SCHED_TICKLESS=y`) |
| Free-running counter | TIM9 (16-bit, APB2 96 MHz) |
| Interval timer | TIM7 (16-bit, APB1 48 MHz) |
| Tick period | 10 µs (`CONFIG_USEC_PER_TICK=10`) |
| System time type | 64-bit (`CONFIG_SYSTEM_TIME64=y`) |

### Why TIM2/TIM5 Are Not Used

TIM2 and TIM5 are the only 32-bit timers on the STM32F413. However, in the pybricks firmware, TIM2 is used for ADC triggering and TIM5 for battery charger PWM. To maintain compatibility for future porting of pybricks features to NuttX, TIM2/TIM5 are not allocated to the tickless timer.

## Overflow Handling with 16-bit Timers

TIM9 is 16-bit (0–65535) and overflows approximately every **655 ms** at 100 kHz (10 µs/tick). The NuttX tickless driver (`arch/arm/src/stm32/stm32_tickless.c`) handles this correctly.

### System Time Retrieval (`up_timer_gettime`)

```c
overflow = g_tickless.overflow;
counter  = STM32_TIM_GETCOUNTER(g_tickless.tch);
pending  = STM32_TIM_CHECKINT(g_tickless.tch, GTIM_SR_UIF);
verify   = STM32_TIM_GETCOUNTER(g_tickless.tch);

if (pending) {
    overflow++;
    counter = verify;
    g_tickless.overflow = overflow;
}

usec = ((((uint64_t)overflow << 16) + (uint64_t)counter) * USEC_PER_SEC) /
       g_tickless.frequency;
```

- `g_tickless.overflow` (32-bit) tracks the number of overflows
- `(overflow << 16) + counter` extends to a 48-bit equivalent continuous time value
- The counter is read twice within a critical section; if a pending interrupt is detected, the post-overflow value is used, avoiding race conditions with the overflow interrupt

### Interval Timer (`up_timer_start`)

```c
DEBUGASSERT(period <= UINT16_MAX);
g_tickless.period = (uint16_t)(period + count);
```

- The compare match value is cast to `(uint16_t)`, so 16-bit wraparound occurs naturally
- Code comment: *"Rollover is fine, channel will trigger on the next period."*
- The maximum delay per single interval is approximately 655 ms, but the NuttX scheduler re-arms the interval as needed, so long waits such as `sleep()` work correctly

### Rollover Correction on Cancel (`up_timer_cancel`)

```c
if (count > period) {
    period += UINT16_MAX;  /* Handle rollover */
}
```

When the counter has passed the compare value (rollover occurred), the remaining time calculation is corrected.

## Tickless wdog race protection (Issue #74 / #75 / #123)

The tickless wdog plumbing converts an absolute deadline (`next_tick`) into a relative interval for the lower-half timer.  Under sustained load the conversion can read the system clock twice with non-trivial work in between, opening a window where the second read overshoots `next_tick` and the unsigned subtraction underflows — `up_timer_start()` then trips `DEBUGASSERT(period <= UINT16_MAX)` because the bogus interval ends up as a multi-second period on a 16-bit timer.

Three patches live in the `owhinata/nuttx` fork (branch `f413-support-12.13.0`) and harden this conversion end-to-end.

### Issue #74 — lower-half compare retry (`arch/arm/src/stm32/stm32_tickless.c`)

`up_timer_start()` reads TIM9 `CNT`, computes `compare = CNT + period`, and writes `CCR1`.  If a high-priority interrupt slips in between the read and the write, the counter can already have passed the target compare value, and the next interrupt would not fire until the counter wraps a full 16-bit cycle (~655 ms).  After SETCOMPARE the driver re-reads the counter and, if it has already overtaken the compare, re-arms to `now + 1` (bounded retry loop, up to 4 attempts).

### Issue #75 — `wd_adjust_next_tick()` anchor + final guard (`sched/wdog/wdog.h`)

`wd_adjust_next_tick()` was previously anchoring the per-chunk interval on `g_wdexpired`, which could lag `clock_systime_ticks()` (e.g., when the scheduler inserts an `interval=0` round-robin event during a timer ISR).  With a stale anchor, `next_tick = g_wdexpired + chunk` lands at or behind `now`, and the caller's unsigned `next_tick - now` underflows.  The fix anchors the chunk on `max(g_wdexpired, now)` and adds a final guard `next_tick >= now + 1`, so the value returned by `wd_adjust_next_tick()` is always strictly in the future relative to its local `now`.

### Issue #123 — `wd_timer_start()` outer clamp (`sched/wdog/wdog.h`)

The Issue #75 guard only covers the read that happens *inside* `wd_adjust_next_tick()`.  The caller (`wd_timer_start()`) then reads the clock again to convert the returned absolute `next_tick` into a relative delta for `up_timer_tick_start()`.  Anything that preempts the small gap between the two reads can advance the clock past `next_tick`, which makes the unsigned subtraction wrap again — the inner guard has no effect on this outer pass.

A normal `enter_critical_section()` (BASEPRI 0x80) masks this race on most platforms, but the spike-nx LUMP UART direct vectors run at NVIC priority 0x00 (above BASEPRI) and *can* preempt critical sections.  Issue #123 was the resulting assertion observed in the wild (~80 min runtime, six LUMP ports + Bluetooth + capture traffic, one occurrence on `feat/122-capture`).

The fix re-reads the clock in `wd_timer_start()` and clamps the delta with `clock_compare()`:

```c
clock_t now = clock_systime_ticks();
clock_t delta;

if (clock_compare(now, next_tick))
  {
    delta = next_tick - now;
#ifdef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
    if (delta > g_oneshot_maxticks)
      {
        delta = g_oneshot_maxticks;
      }
#endif
  }
else
  {
    delta = 1u;  /* deadline already past — let the lower-half retry catch it */
  }

up_timer_tick_start(delta);
```

A deadline that has already slipped into the past becomes a 1-tick re-arm (Issue #74's retry loop then handles the actual late-compare), and an overshoot that exceeds the lower-half timer's representable range is clamped to the maximum.  Either way the `period <= UINT16_MAX` assert can no longer fire.

The fix is independent of any board-specific HIPRI policy — the clamp is a general guard that applies anywhere the absolute-to-relative conversion straddles a clock read.

## Performance Counter

`CONFIG_ARCH_PERF_EVENTS=y` enables high-precision time measurement using the STM32F413 DWT CYCCNT (Data Watchpoint and Trace cycle counter).

| Item | Value |
|------|-------|
| Counter | DWT CYCCNT (32-bit) |
| Frequency | 96 MHz (SYSCLK) |
| Resolution | ~10.4 ns |
| Overflow | ~44.7 seconds |

### API

```c
#include <nuttx/clock.h>

clock_t t0 = perf_gettime();       /* Get counter value */
unsigned long freq = perf_getfreq(); /* Get frequency (96000000) */

/* Calculate elapsed time */
clock_t t1 = perf_gettime();
uint64_t elapsed_ns = ((uint64_t)(t1 - t0) * 1000000000ULL) / freq;
```

- `perf_gettime()` — Returns the current DWT CYCCNT value
- `perf_getfreq()` — Returns the counter frequency in Hz

The 32-bit counter overflows approximately every 44.7 seconds. Use `clock_systime_ticks()` for longer measurements.

## ostest wdog Test WARNINGs

The ostest wdog test produces many warnings like:

```
WARNING: wdog latency ticks 65538 (> 10 may indicate timing error)
```

This is a **measurement artifact in the test code** and does not affect OS operation.

### Cause

The test measures latency using `clock_systime_ticks()` differences. `wdtest_rand` repeats 1024 iterations with random delays up to 12,345 ns. During this process, timing discrepancies between `clock_systime_ticks()` calls outside critical sections and actual callback firing produce large apparent latency values.

### Evidence That wdog Operates Correctly

- `wdtest_assert(diff - delay_tick >= 0)` — the test would abort if a callback fired earlier than expected, but it does not
- WARNINGs only indicate "latency exceeded 10 ticks," not that the wdog failed to fire
- The entire test completes successfully through `wdog_test end...`
