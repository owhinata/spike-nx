# Battery and Charger Drivers

The SPIKE Prime Hub has a 2S Li-ion battery pack (nominal 7.2V) and an MPS MP2639A USB battery charger IC. Two NuttX drivers provide battery monitoring and charge control.

## Hardware Configuration

### Battery Sensing (ADC)

All battery measurements use the existing ADC1 DMA continuous conversion (1 kHz, TIM2 trigger).

| Measurement | ADC Channel | Pin | Rank | Scaling |
|-------------|-------------|-----|------|---------|
| Battery current | CH10 | PC0 | 0 | raw × 7300 / 4096 mA |
| Battery voltage | CH11 | PC1 | 1 | raw × 9900 / 4096 + I × 3/16 mV |
| Battery temperature | CH8 | PB0 | 2 | NTC thermistor (103AT) |
| USB charger current | CH3 | PA3 | 3 | (raw × 35116 >> 16) − 123 mA |

- **Voltage correction**: The measured voltage is corrected for path resistance (0.1875 ohm) using the measured current.
- **Boot suppression**: During the first 1 second after boot, if the measured voltage is below 7000 mV, 7000 mV is reported instead (voltage is unreliable during power-on).
- **Temperature**: Uses a 103AT NTC thermistor with B-parameter equation (B=3435, R0=10k @ 25C, voltage divider 2.4k/7.5k).

### MP2639A Charger IC

| Signal | Connection | Description |
|--------|-----------|-------------|
| MODE | TLC5955 channel 14 | Charging enable (active low via LED driver) |
| ISET | TIM5 CH1 (PA0, AF2) | Charge current limit (96 kHz PWM) |
| /CHG | Resistor ladder DEV_0 CH2 | Charge status (shared with center button ADC) |
| IB | ADC CH3 (PA3) | USB charger current measurement |

#### ISET Current Limit (PWM Duty Cycle)

| Duty | Current Limit | Use Case |
|------|--------------|----------|
| 0% | 0 mA | Disabled |
| 2% | 100 mA | USB standard min |
| 15% | 500 mA | USB standard max |
| 100% | 1.5 A | Dedicated charger |

#### CHG Status Detection

The /CHG pin is on a resistor ladder shared with the center button (PC4, ADC rank 4). The resistor ladder decoder extracts the CHG signal as channel 2.

- **CHG on** (pin low): Battery is charging
- **CHG off** (pin high, after settling): Charging complete
- **CHG blinking** (~1 Hz): Charger fault

The charger polls the CHG signal at 4 Hz (250 ms interval) using a 7-sample circular buffer. If more than 2 transitions are detected in the window, a fault condition is reported.

#### USB Detection

USB presence is detected by reading VBUS on PA9 (GPIO). When VBUS is present, charging is enabled with a 500 mA current limit. Full USB Battery Charging Detection (BCD) is not implemented because it conflicts with the active CDC/ACM console.

#### Charge Timeout

After 60 minutes of continuous charging, the charger pauses for 30 seconds and then restarts. This handles charger-battery pairs that may not properly restart charging after reaching full.

## NuttX Device Interface

### Battery Gauge (`/dev/bat0`)

Registered via `battery_gauge_register()`. Supports these IOCTL commands:

| IOCTL | Description | Unit |
|-------|-------------|------|
| `BATIOC_STATE` | Battery state | `BATTERY_DISCHARGING` |
| `BATIOC_ONLINE` | Battery present | Always `true` |
| `BATIOC_VOLTAGE` | Battery voltage | mV |
| `BATIOC_CURRENT` | Battery current | mA |
| `BATIOC_CAPACITY` | State of Charge | % (voltage-based estimate) |
| `BATIOC_TEMPERATURE` | Battery temperature | millidegrees C |

### Battery Charger (`/dev/charge0`)

Registered via `battery_charger_register()`. Supports these IOCTL commands:

| IOCTL | Description | Unit |
|-------|-------------|------|
| `BATIOC_STATE` | Charger state | `BATTERY_CHARGING` / `BATTERY_FULL` / `BATTERY_DISCHARGING` / `BATTERY_FAULT` |
| `BATIOC_HEALTH` | Charger health | `BATTERY_HEALTH_GOOD` / `BATTERY_HEALTH_UNSPEC_FAIL` |
| `BATIOC_ONLINE` | USB connected | `true` / `false` |
| `BATIOC_CURRENT` | Set charge current | 0 / 100 / 500 / 1500 mA |
| `BATIOC_VOLTAGE_INFO` | Target charge voltage | 8400 mV |
| `BATIOC_CHIPID` | Charger chip ID | `0x2639` |

## Battery LED Indication

The battery LED (TLC5955 channels 0-2: B/G/R) is updated by the charger driver's 4 Hz polling loop.

| Charger State | Condition | LED Color | Pattern |
|---------------|-----------|-----------|---------|
| DISCHARGING | USB not connected | OFF | - |
| CHARGING | Voltage < 8190 mV | Red | Solid |
| CHARGING | Voltage >= 8190 mV | Green | Solid |
| COMPLETE | Charge complete | Green | Blink (2.75s on / 0.25s off) |
| FAULT | Charger error | Yellow | Blink (0.5s on / 0.5s off) |

The full-charge threshold (8190 mV = 4.095V per cell) uses an exponential moving average (127/128 coefficient) to avoid flickering.

## Test App

The `battery` command reads gauge and charger status via IOCTL.

```
nsh> battery
=== Battery Gauge (/dev/bat0) ===
  State:       DISCHARGING
  Online:      yes
  Voltage:     8588 mV
  Current:     39 mA
  Capacity:    100 %
  Temperature: 29.184 C

=== Battery Charger (/dev/charge0) ===
  State:       CHARGING
  Health:      GOOD
  USB online:  yes
  Chip ID:     0x2639
  Target V:    8400 mV
```

| Command | Description |
|---------|-------------|
| `battery` | Show gauge + charger info |
| `battery gauge` | Gauge only |
| `battery charger` | Charger only |
| `battery monitor [N]` | Monitor N times at 1s interval (default: 10) |

Enabled by `CONFIG_APP_BATTERY=y`.

## Source Files

| File | Description |
|------|-------------|
| [`stm32_battery_gauge.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_battery_gauge.c) | Battery gauge lower-half driver |
| [`stm32_battery_charger.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_battery_charger.c) | MP2639A charger lower-half driver |
| [`stm32_resistor_ladder.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_resistor_ladder.c) | Resistor ladder decoder |
| [`stm32_adc_dma.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_adc_dma.c) | ADC DMA continuous conversion |

## Pybricks Reference

Ported from:

- [`battery_adc.c`](https://github.com/pybricks/pybricks-micropython/blob/v3.6.1/lib/pbio/drv/battery/battery_adc.c) — Battery voltage/current/temperature
- [`charger_mp2639a.c`](https://github.com/pybricks/pybricks-micropython/blob/v3.6.1/lib/pbio/drv/charger/charger_mp2639a.c) — MP2639A charger control
- [`resistor_ladder.c`](https://github.com/pybricks/pybricks-micropython/blob/v3.6.1/lib/pbio/drv/resistor_ladder/resistor_ladder.c) — Resistor ladder decoder

### Differences from Pybricks

| Feature | Pybricks | NuttX Port |
|---------|----------|------------|
| USB detection | Full BCD (GCCFG register) | VBUS GPIO only |
| Default current limit | Based on BCD type | 500 mA fixed |
| Framework | Custom driver API | NuttX battery gauge/charger |
| Polling | Contiki protothread | NuttX HPWORK queue |

## STM32F413 OTG FS Fix

The NuttX OTG FS driver originally treated STM32F413 as a legacy F4 variant, incorrectly setting GCCFG bits 18/19 as VBUS sensing (VBUSASEN/VBUSBSEN). On F413, these bits are BCD-related (DCDEN/PDEN), causing USB disconnect/reconnect to fail.

The fix adds F412/F413 to the F446/F469 code path in the NuttX OTG FS driver:

- **GCCFG**: Only `PWRDWN` is set (BCD bits are left untouched)
- **GOTGCTL**: `BVALOEN | BVALOVAL` forces B-session valid (replaces `NOVBUSSENS`)
- **SEDET/SRQ**: Session end and session request interrupt handlers are implemented for proper VBUS disconnect/reconnect detection

This change is in the NuttX submodule (`arch/arm/src/stm32/stm32_otgfsdev.c` and `hardware/stm32fxxxxx_otgfs.h`).
