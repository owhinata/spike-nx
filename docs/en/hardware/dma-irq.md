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
- NuttX default: `NVIC_SYSH_PRIORITY_DEFAULT = 0x80` (level 8)
- NuttX BASEPRI: `0x80` — IRQs above this threshold (< 0x80) are not masked by critical sections

### pybricks IRQ Priority (Reference)

| Priority (base) | NVIC value | Peripheral |
|---|---|---|
| 1 | 0x10 | Bluetooth UART + DMA |
| 3 | 0x30 | IMU I2C + EXTI |
| 4 | 0x40 | Sound DMA |
| 5 | 0x50 | Flash SPI2 DMA |
| 6 | 0x60 | USB OTG FS + VBUS |
| 7 | 0x70 | ADC DMA + TLC5955 SPI DMA |

### NuttX IRQ Priority Design

Preserves the relative priority order from pybricks while keeping all priorities at or below NuttX BASEPRI (0x80). IRQs above BASEPRI (< 0x80) are not blocked during critical sections and cannot safely call NuttX APIs.

| NVIC value | Level | Peripheral | pybricks mapping | Configured in |
|---|---|---|---|---|
| 0x80 | 8 | Sound DMA1_S5 | base=4 → highest | `stm32_sound.c` |
| 0x80 | 8 | IMU I2C2 / EXTI | base=3 → NuttX I2C driver managed | NuttX default |
| 0x80 | 8 | USB OTG FS | base=6 → NuttX USB driver managed | NuttX default |
| 0x80 | 8 | TLC5955 SPI1 DMA | base=7 → NuttX SPI driver managed | NuttX default |
| 0xB0 | 11 | ADC DMA2_S0 | base=7 (MEDIUM) | `stm32_adc_dma.c` |

!!! note "NuttX-Managed IRQs"
    I2C / SPI / USB IRQ priorities are internally managed by NuttX drivers and are not changed at the board level. In pybricks, I2C (level 3) > Sound (level 4), but NuttX I2C drivers call NuttX APIs from ISR context and must remain below BASEPRI.

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

## Future Expansion

When implementing Motor / Bluetooth / Flash, follow the pybricks DMA/IRQ allocation directly. Note that pybricks sets Bluetooth to NVIC level 1 (DMA VERY_HIGH), but NuttX's BASEPRI constraint may force it to 0x80. Whether this impacts BT communication quality must be verified at implementation time.

## References

- RM0430 Table 27: DMA1 request mapping
- RM0430 Table 28: DMA2 request mapping
- pybricks `lib/pbio/platform/prime_hub/platform.c`
- pybricks `lib/pbio/drv/sound/sound_stm32_hal_dac.c`
- pybricks `lib/pbio/drv/adc/adc_stm32_hal.c`
- pybricks `lib/pbio/drv/pwm/pwm_tlc5955_stm32.c`
