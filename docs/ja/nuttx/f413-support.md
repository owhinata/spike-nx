# STM32F413 チップサポート

NuttX における STM32F413 チップサポートの概要。

## フォークが必要な理由

STM32F413 は NuttX 12.12.0 でチップレベルで未サポート。Out-of-tree ボード定義だけでは対応できず、カーネルレベルの変更が必須:

- `CONFIG_ARCH_CHIP_STM32F413` が Kconfig に存在しない
- UART9/10 のインフラ (Kconfig, IRQ, RCC, シリアルドライバ) が一切ない
- 割り込みベクタテーブルサイズはカーネルのコンパイル時定数
- RCC クロック有効化コードはカーネル側

最も近い既存チップは STM32F412。F413 は F412 に対して UART9/10, FMPI2C, DFSDM, SAI1, CAN3, Flash 容量増 (1MB -> 1.5MB), SRAM 増 (256KB -> 320KB) が追加されている。

## 最小パッチセット (10 ファイル)

| # | ファイル | 変更内容 |
|---|---|---|
| 1 | `arch/arm/src/stm32/Kconfig` | `STM32_STM32F413` ファミリ設定、`ARCH_CHIP_STM32F413VG`、`STM32_HAVE_UART9/10` 追加 |
| 2 | `arch/arm/include/stm32/chip.h` | F413 のペリフェラル数、Flash 1.5MB (0x180000) / SRAM 320KB 定義 |
| 3 | `arch/arm/include/stm32/stm32f40xxx_irq.h` | UART9 IRQ=88, UART10 IRQ=89, `STM32_IRQ_NEXTINT` 更新 |
| 4 | `arch/arm/src/stm32/hardware/stm32f40xxx_memorymap.h` | UART9/10 ベースアドレス追加 |
| 5 | `arch/arm/src/stm32/hardware/stm32f40xxx_rcc.h` | APB2ENR に UART9/10 ビット追加 |
| 6 | `arch/arm/src/stm32/stm32f40xxx_rcc.c` | `rcc_enableapb2()` に UART9/10 クロック有効化追加 |
| 7 | `arch/arm/src/stm32/stm32_serial.c` | UART9/10 デバイスインスタンス追加 (UART7/8 パターン) |
| 8 | `arch/arm/src/stm32/hardware/stm32f413xx_pinmap.h` | 新規作成: F413 固有ピンマップ (F412 ベース + UART9/10, AF11) |
| 9 | `arch/arm/src/stm32/hardware/stm32_pinmap.h` | F413 ピンマップ include 分岐追加 |
| 10 | `boards/Kconfig` | ボードエントリ追加 |

## UART9/10 詳細

### IRQ 番号

| ペリフェラル | IRQ 番号 | バス |
|---|---|---|
| UART9 | 88 | APB2 |
| UART10 | 89 | APB2 |

### ベースアドレス

| ペリフェラル | ベースアドレス | バス |
|---|---|---|
| UART9 | 0x40011800 | APB2 |
| UART10 | 0x40011C00 | APB2 |

UART9/10 は APB2 バス上にあるため、ボーレート計算は PCLK2 (96 MHz) を使用する。USART1/USART6 と同じバス。

### RCC 有効化ビット

| ペリフェラル | レジスタ | ビット位置 | マスク |
|---|---|---|---|
| UART9 | RCC_APB2ENR | Bit 6 | 0x00000040 |
| UART10 | RCC_APB2ENR | Bit 7 | 0x00000080 |

### Alternate Function

UART9/10 はすべて AF11:

| 機能 | ピン | AF |
|---|---|---|
| UART9_TX | PD15 / PG1 | AF11 |
| UART9_RX | PD14 / PG0 | AF11 |
| UART10_TX | PE3 / PG12 | AF11 |
| UART10_RX | PE2 / PG11 | AF11 |

SPIKE Prime Hub のポート割当:

| ポート | UART | AF |
|---|---|---|
| A | UART7 | AF8 |
| B | UART4 | AF11 |
| C | UART8 | AF8 |
| D | UART5 | AF8 |
| E | UART10 | AF11 |
| F | UART9 | AF11 |

## クロック設定

SPIKE Prime Hub: 16 MHz HSE -> 96 MHz SYSCLK

```c
#define STM32_BOARD_XTAL        16000000      // 16 MHz HSE
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)    // VCO入力 = 2 MHz
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(96)    // VCO出力 = 192 MHz
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2      // SYSCLK = 96 MHz
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(4)     // USB = 48 MHz
#define STM32_SYSCLK_FREQUENCY  96000000ul

// バスクロック
// AHB  = 96 MHz (HPRE  = SYSCLK)
// APB1 = 48 MHz (PPRE1 = HCLKd2)
// APB2 = 96 MHz (PPRE2 = HCLK)
```

## F413 vs F412 のギャップ

| 項目 | F412 (NuttX 既存) | F413 (追加必要) |
|---|---|---|
| チップバリアント Kconfig | あり | なし (追加必要) |
| ペリフェラル数 (chip.h) | F412 用 | F413 用に更新必要 |
| ピンマップヘッダ | `stm32f412xx_pinmap.h` | `stm32f413xx_pinmap.h` 新規作成 |
| UART | 1-8 | 1-10 (9/10 追加必要) |
| Flash | 1 MB | 1.5 MB |
| SRAM | 256 KB | 320 KB |

## 後回し可能な機能

| 機能 | 理由 |
|---|---|
| FMPI2C ドライバ | SPIKE Prime Hub は通常の I2C2 を使用。I2C v2 レジスタインタフェースのため新規ドライバが必要 |
| DFSDM ドライバ | SPIKE Prime Hub で未使用。NuttX にコードなし |
| SAI1 ドライバ | SPIKE Prime Hub で未使用 |
| CAN3 ドライバ | SPIKE Prime Hub で未使用。CAN1/2 のみサポート |
