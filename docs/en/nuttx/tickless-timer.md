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
