# Watchdog Timer

This document describes the NuttX watchdog timer configuration on the SPIKE Prime Hub.

## Overview

| Item | Setting |
|------|---------|
| Hardware watchdog | STM32 IWDG (Independent Watchdog) |
| Timeout | 3000 ms (`CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT=3000`) |
| Ping interval | 1000 ms (`CONFIG_WATCHDOG_AUTOMONITOR_PING_INTERVAL=1000`) |
| Feed method | Kernel automatic (NuttX wdog timer) |
| Device path | `/dev/watchdog0` |

## IWDG Characteristics

The IWDG is a hardware watchdog that operates independently on the LSI (32 kHz) oscillator.

- Does not conflict with other timer resources (TIM2, TIM5, TIM9, etc.)
- **Cannot be stopped once started** (hardware limitation)
- Resets the MCU if the system hangs and the timeout expires
- Firmware in Flash is preserved after reset; NuttX reboots normally

## Configuration

### defconfig

```
CONFIG_STM32_IWDG=y
CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG=y
CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT=3000
CONFIG_WATCHDOG_AUTOMONITOR_PING_INTERVAL=1000
```

### Board Initialization

The IWDG device is registered in `stm32_bringup()`:

```c
#ifdef CONFIG_STM32_IWDG
  stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif
```

`STM32_LSI_FREQUENCY` (32000 Hz) is defined in `board.h`.

## Automonitor Operation

When `CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG` is enabled, the IWDG is automatically fed using the NuttX sw wdog timer mechanism.

1. `/dev/watchdog0` is registered during board initialization
2. The driver starts the IWDG, and the automonitor sets up a sw wdog timer
3. `keepalive` is called every 1000 ms, reloading the IWDG counter
4. If the kernel hangs and the sw wdog timer cannot fire, the IWDG triggers a reset after 3 seconds

If an application explicitly opens `/dev/watchdog0` and issues `WDIOC_START`, the automonitor stops and the application assumes ping responsibility.

## Comparison with pybricks

| Item | pybricks | NuttX |
|------|----------|-------|
| Watchdog | IWDG | IWDG |
| Timeout | 3 seconds | 3 seconds |
| Prescaler | /64, reload=1500 | Calculated by driver |
| Feed method | Every cycle in supervisor poll loop | sw wdog timer at 1-second intervals |
| Reset detection | Checks RCC_CSR_IWDGRSTF | Not implemented |

## Debugging Notes

The IWDG cannot be stopped, so pausing the program with a debugger will trigger a reset after 3 seconds. When debugging, either disable `CONFIG_STM32_IWDG` or configure the STM32 DBGMCU register to freeze the IWDG during debug halt.
