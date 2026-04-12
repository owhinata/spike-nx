# DMA ストリーム割当と IRQ 優先度

## 概要

SPIKE Prime Hub (STM32F413) で使用する DMA ストリーム割当と NVIC IRQ 優先度の設計。pybricks v3.6.1 の実装を基準とし、NuttX の IRQ 管理制約に適合させる。

## DMA ストリーム割当

RM0430 Table 27/28 (DMA request mapping) に基づく。

### DMA1

| Stream | Channel | ペリフェラル | 状態 | 優先度 |
|--------|---------|-------------|------|--------|
| S3 | Ch0 | Flash SPI2 RX | 未実装 (pybricks 予約) | HIGH |
| S4 | Ch0 | Flash SPI2 TX | 未実装 (pybricks 予約) | HIGH |
| S5 | Ch7 | **DAC1 CH1 (Sound)** | ✅ 実装済 | **HIGH** |
| S6 | Ch4 | BT UART2 TX | 未実装 (pybricks 予約) | VERY_HIGH |
| S7 | Ch6 | BT UART2 RX | 未実装 (pybricks 予約) | VERY_HIGH |

### DMA2

| Stream | Channel | ペリフェラル | 状態 | 優先度 |
|--------|---------|-------------|------|--------|
| S0 | Ch0 | **ADC1** | ✅ 実装済 | **MEDIUM** |
| S2 | Ch3 | **TLC5955 SPI1 RX** | ✅ 実装済 (NuttX SPI) | LOW |
| S3 | Ch3 | **TLC5955 SPI1 TX** | ✅ 実装済 (NuttX SPI) | LOW |

### USB OTG FS

FIFO ベース。DMA 非使用。

## NVIC IRQ 優先度

### STM32F413 の NVIC 仕様

- 優先度ビット: 4 bit (16 レベル、0x00 が最高 / 0xF0 が最低)
- ステップ: `NVIC_SYSH_PRIORITY_STEP = 0x10`
- NuttX デフォルト: `NVIC_SYSH_PRIORITY_DEFAULT = 0x80` (レベル 8)
- NuttX BASEPRI: `0x80` — これ以上の優先度 (< 0x80) の IRQ はクリティカルセクションでマスクされない

### pybricks の IRQ 優先度 (参考)

| 優先度 (base) | NVIC 値 | ペリフェラル |
|---|---|---|
| 1 | 0x10 | Bluetooth UART + DMA |
| 3 | 0x30 | IMU I2C + EXTI |
| 4 | 0x40 | Sound DMA |
| 5 | 0x50 | Flash SPI2 DMA |
| 6 | 0x60 | USB OTG FS + VBUS |
| 7 | 0x70 | ADC DMA + TLC5955 SPI DMA |

### NuttX での IRQ 優先度設計

pybricks の相対的な優先順序を維持しつつ、NuttX の BASEPRI (0x80) 以下に収める。BASEPRI より高い優先度 (< 0x80) の IRQ はクリティカルセクションでブロックされず、NuttX API を安全に呼び出せないため使用しない。

| NVIC 値 | レベル | ペリフェラル | pybricks 対応 | 設定箇所 |
|---|---|---|---|---|
| 0x80 | 8 | Sound DMA1_S5 | base=4 → 最上位 | `stm32_sound.c` |
| 0x80 | 8 | IMU I2C2 / EXTI | base=3 → NuttX I2C driver 管理 | NuttX デフォルト |
| 0x80 | 8 | USB OTG FS | base=6 → NuttX USB driver 管理 | NuttX デフォルト |
| 0x80 | 8 | TLC5955 SPI1 DMA | base=7 → NuttX SPI driver 管理 | NuttX デフォルト |
| 0xB0 | 11 | ADC DMA2_S0 | base=7 (MEDIUM) | `stm32_adc_dma.c` |

!!! note "NuttX 管理の IRQ"
    I2C / SPI / USB の IRQ 優先度は NuttX ドライバが内部管理しているため、ボード側からは変更しない。pybricks では I2C (level 3) > Sound (level 4) だが、NuttX では I2C ドライバが ISR 内で NuttX API を呼ぶため BASEPRI 以下に留める必要がある。

### DMA 優先度 (DMA_SCR PL フィールド)

NVIC 優先度とは別に、DMA コントローラ内部のアービトレーション優先度。同一 DMA コントローラ内で複数ストリームが同時にリクエストした場合の調停に使われる。

| DMA 優先度 | ペリフェラル | 設定箇所 |
|---|---|---|
| HIGH | Sound DMA1_S5 | `stm32_sound.c` (DMA_SCR_PRIHI) |
| MEDIUM | ADC DMA2_S0 | `stm32_adc_dma.c` (DMA_SCR_PRIMED) |
| LOW | TLC5955 SPI1 | NuttX SPI driver (SPI_DMA_PRIO) |

## タイマー割当

| Timer | 用途 | 状態 | pybricks 用途 |
|---|---|---|---|
| TIM1 | Motor PWM (Port A/B) | 未実装 | 12 kHz PWM, PSC=8, ARR=1000 |
| TIM2 | ADC trigger (TRGO 1 kHz) | ✅ 実装済 | 同一 |
| TIM3 | Motor PWM (Port E/F) | 未実装 | 12 kHz PWM |
| TIM4 | Motor PWM (Port C/D) | 未実装 | 12 kHz PWM |
| TIM5 | Charger ISET PWM | defconfig 有、未実装 | 96 kHz, CH1 |
| TIM6 | DAC sample rate (TRGO) | ✅ 実装済 | 同一 |
| TIM8 | BT 32.768 kHz clock | 未実装 | CH4 PWM |
| TIM9 | NuttX tickless timer | ✅ 実装済 | (pybricks では SysTick) |
| TIM12 | TLC5955 GSCLK | ✅ 実装済 | 同一 (CH2, ~8.7 MHz) |

## 将来の拡張

Motor / Bluetooth / Flash 実装時に追加すべき DMA/IRQ 設定は、pybricks の割当をそのまま踏襲する。pybricks では Bluetooth が最高優先度 (NVIC 1, DMA VERY_HIGH) だが、NuttX では BASEPRI 制約のため NVIC 0x80 に制限される可能性がある。この制約が BT 通信品質に影響するかは実装時に検証が必要。

## 参照

- RM0430 Table 27: DMA1 request mapping
- RM0430 Table 28: DMA2 request mapping
- pybricks `lib/pbio/platform/prime_hub/platform.c`
- pybricks `lib/pbio/drv/sound/sound_stm32_hal_dac.c`
- pybricks `lib/pbio/drv/adc/adc_stm32_hal.c`
- pybricks `lib/pbio/drv/pwm/pwm_tlc5955_stm32.c`
