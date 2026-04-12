# Power Control

This document describes the SPIKE Prime Hub power control and center button shutdown.

## Hardware Configuration

### Power Control

| Pin | Function | Description |
|-----|----------|-------------|
| PA13 | BAT_PWR_EN | HIGH: power ON, LOW: power OFF |
| PA14 | PORT_3V3_EN | I/O port 3.3V power enable |

PA13 is set HIGH in `stm32_boardinitialize()` immediately at boot. Setting it LOW cuts the battery power supply.

### Center Button (ADC Resistor Ladder)

The center button is read via an ADC resistor ladder, not GPIO.

| Item | Value |
|------|-------|
| GPIO | PC4 (analog input) |
| ADC channel | ADC1 CH14 (note: PC4 maps to CH14 on STM32F413) |
| Unpressed ADC value | ~3645 |
| Pressed ADC value | ~2872 |
| Press threshold | Below 3200 = pressed |

Resistor ladder circuit (from pybricks):
```
  ^ 3.3V
  |
  Z 10k
  |
  +-------+-------+----> PC4 (ADC)
  |       |       |
  [A]   [B]     [C]
  |       |       |
  Z 18k  Z 33k  Z 82k
  |       |       |
  +-------+-------+
         GND
```

## Behavior

### Power ON

Handled by the LEGO bootloader. Pressing the center button powers the MCU and the bootloader loads the firmware.

### Normal Operation

- Center button LED lights green on NSH startup
- Button state is polled every 50ms via HPWORK queue
- Button monitoring is disabled for 3 seconds after boot (prevents false triggers)

### Power OFF (Long Press >= 2 seconds)

1. Center button LED turns blue (shutdown indicator)
2. Wait for button release
3. Set PA13 (BAT_PWR_EN) LOW → power cut
4. If USB is connected, the MCU remains powered via USB, so `board_reset()` is called to reset

### DFU Mode

During DFU mode, the LEGO bootloader lights the Bluetooth LED in rainbow colors (when battery is connected).

## ADC DMA Continuous Conversion

Button reading uses ADC1 DMA continuous conversion. TIM2 TRGO triggers a 6-channel scan at 1 kHz.

| Rank | Channel | Pin | Purpose |
|------|---------|-----|---------|
| 0 | CH10 | PC0 | Battery current (IBAT) |
| 1 | CH11 | PC1 | Battery voltage (VBAT) |
| 2 | CH8 | PB0 | Battery temperature (NTC) |
| 3 | CH3 | PA3 | USB charger current (IBUSBCH) |
| 4 | CH14 | PC4 | Center button resistor ladder |
| 5 | CH5 | PA1 | Left/Right/BT button resistor ladder |

The DMA buffer is continuously updated by DMA2 Stream0 in circular mode. `stm32_adc_read(rank)` returns the latest value instantly.

## defconfig

```
CONFIG_STM32_ADC1=y
```

## Comparison with pybricks

| Item | pybricks | NuttX |
|------|----------|-------|
| Power off trigger | Center button 2s long press | Same |
| ADC reading | HAL ADC + DMA + TIM2 trigger | ADC: direct register, DMA + TIM2: NuttX abstraction API |
| ADC scan rate | 1 kHz (6 channels) | 1 kHz (6 channels) |
| Button polling | Contiki event loop (50ms) | HPWORK queue (50ms) |
| Power off with USB | Power stays on (op-amp dependent) | PA13 LOW then reset |
| ADC clock | APB2/4 = 24 MHz | APB2/4 = 24 MHz |

## Source Files

- `boards/spike-prime-hub/src/stm32_adc_dma.c` — ADC DMA continuous conversion
- `boards/spike-prime-hub/src/stm32_power.c` — Power control and button monitor
- `boards/spike-prime-hub/src/spike_prime_hub.h` — GPIO/ADC rank definitions
- `boards/spike-prime-hub/src/stm32_boot.c` — PA13/PA14 early initialization
