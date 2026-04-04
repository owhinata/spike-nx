# STM32F413 NuttX サポート状況調査

## 1. 既存 STM32F4 チップサポート

### NuttX 12.12.0 でサポート済みの F4 ファミリ

| ファミリ | 個別チップ数 | 備考 |
|---|---|---|
| STM32F401 | 12 | xBC, xDE サブバリアント |
| STM32F405 | 3 | RG, VG, ZG |
| STM32F407 | 6 | VE, VG, ZE, ZG, IE, IG |
| STM32F410 | 1 | RB |
| STM32F411 | 3 | CE, RE, VE |
| STM32F412 | 2 | CE, ZG |
| STM32F427 | 3 | V, Z, I |
| STM32F429 | 5 | V, Z, I, B, N |
| STM32F446 | 4 | M, R, V, Z |
| STM32F469 | 4 | A, I, B, N |

**STM32F413 / STM32F423 は未サポート。** Kconfig にエントリなし、ソースツリーに F413 関連コードなし。

### F4 ボード一覧 (主要)

- `nucleo-f401re`, `nucleo-f410rb`, `nucleo-f411re`, **`nucleo-f412zg`**
- `nucleo-f429zi`, `nucleo-f446re`
- `stm32f4discovery`, `stm32f411e-disco`, `stm32f429i-disco`
- `olimex-stm32-e407`, `olimex-stm32-h405`, `olimex-stm32-h407`, `olimex-stm32-p407`
- `omnibusf4`, `photon` 他 (計 55 ボード)

テンプレートとして最適: **`nucleo-f412zg`** (同じ F41x 系、96 MHz 動作)

---

## 2. F413 固有ペリフェラルサポート状況

### UART/USART

| ペリフェラル | NuttX サポート | 備考 |
|---|---|---|
| USART1, 2, 3, 6 | 対応済み | 全 F4 で共通 |
| UART4, 5 | 対応済み | F427/F429/F469 で有効化済み |
| UART7, 8 | 対応済み | F427/F429/F469 で有効化済み。IRQ, RCC, シリアルドライバ全て実装済み |
| **UART9** | **未対応** | IRQ ベクタ、RCC ビット、シリアルドライバ全て未実装 |
| **UART10** | **未対応** | 同上 |

UART9/10 は F413 固有のペリフェラル。SPIKE Prime Hub ではポート E (UART10) とポート F (UART9) で使用。

### I2C

| ペリフェラル | NuttX サポート | 備考 |
|---|---|---|
| I2C1, 2, 3 | 対応済み | 通常の I2C v1 ドライバ |
| FMPI2C1 | **RCC のみ** | `RCC_APB1ENR_FMPI2C1EN` は定義済み。ドライバ未実装 (I2C v2 レジスタインタフェースのため新規ドライバ必要) |

SPIKE Prime Hub の IMU は通常の I2C2 を使用するため、FMPI2C 未対応は初期段階では問題なし。

### その他

| ペリフェラル | NuttX サポート | 備考 |
|---|---|---|
| DFSDM | **未対応** | NuttX に一切コードなし |
| SAI1 | 未確認 | 他の STM32 ファミリには存在する可能性あり |
| CAN3 | 未対応 | CAN1/2 のみサポート |

---

## 3. クロック設定

### nucleo-f412zg の PLL 設定 (参考)

Nucleo-F412ZG は 8 MHz HSE → 96 MHz SYSCLK:

```c
#define STM32_BOARD_USEHSE      1
#define STM32_BOARD_XTAL        8000000       // 8 MHz HSE
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)    // VCO入力 = 1 MHz
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(384)   // VCO出力 = 384 MHz
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4      // SYSCLK = 96 MHz
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)     // USB = 48 MHz
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)
#define STM32_SYSCLK_FREQUENCY  96000000ul

// バスクロック
#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK   // AHB = 96 MHz
#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2   // APB1 = 48 MHz
#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK     // APB2 = 96 MHz

// DCKCFGR2 (F412/F413 固有クロック設定レジスタ)
#define STM32_RCC_DCKCFGR2_CK48MSEL    RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_FMPI2C1SEL  RCC_DCKCFGR2_FMPI2C1SEL_APB
#define STM32_RCC_DCKCFGR2_SDIOSEL     RCC_DCKCFGR2_SDIOSEL_48MHZ
```

### SPIKE Prime Hub 用の設定

SPIKE Prime Hub は 16 MHz HSE → 96 MHz SYSCLK (pybricks 実装に基づく):

```c
#define STM32_BOARD_USEHSE      1
#define STM32_BOARD_XTAL        16000000      // 16 MHz HSE
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)    // VCO入力 = 2 MHz
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(96)    // VCO出力 = 192 MHz
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2      // SYSCLK = 96 MHz
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(4)     // USB = 48 MHz
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)
#define STM32_SYSCLK_FREQUENCY  96000000ul

// バスクロック (pybricks と同一)
#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK   // AHB = 96 MHz
#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2   // APB1 = 48 MHz
#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK     // APB2 = 96 MHz
```

---

## 4. チップサポートのアーキテクチャ

NuttX での F412 チップサポートの構造:

1. **Kconfig**: `ARCH_CHIP_STM32F412ZG` → `STM32_STM32F412` → `STM32_STM32F4XXX`
2. **`arch/arm/include/stm32/chip.h`**: チップ別のペリフェラル数定義 (NUSART, NI2C 等)
3. **`arch/arm/include/stm32/stm32f40xxx_irq.h`**: 全 F4 チップ共通 IRQ ヘッダ (`#if` 分岐)
4. **`hardware/stm32f40xxx_memorymap.h`**: 全 F4 チップ共通メモリマップ
5. **`hardware/stm32f412xx_pinmap.h`**: F412 専用ピンマップ
6. **`hardware/stm32f40xxx_rcc.h`**: 全 F4 チップ共通 RCC ヘッダ (`#if` 分岐)
7. **`stm32f40xxx_rcc.c`**: 全 F4 チップ共通 RCC 初期化 (`#if` 分岐)

F412 は F40xxx インフラを共有しつつ、専用ピンマップを持つ。

---

## 5. ギャップ分析: F413 追加に必要な作業

### 必須 (最小限の動作に必要)

| 項目 | 工数 | 変更対象 |
|---|---|---|
| Kconfig: F413 チップバリアント追加 | 小 | `arch/arm/src/stm32/Kconfig` |
| Kconfig: F413 ファミリ設定 (HAVE_UARTx 等) | 小 | 同上 |
| チップペリフェラル数定義 | 小 | `arch/arm/include/stm32/chip.h` |
| ピンマップヘッダ作成 | 中 | `hardware/stm32f413xx_pinmap.h` (F412 ベース) |
| ピンマップ選択分岐追加 | 小 | `hardware/stm32_pinmap.h` |
| RCC 初期化に F413 分岐追加 | 小 | `stm32f40xxx_rcc.c` |
| RCC ヘッダに F413 追加 | 小 | `hardware/stm32f40xxx_rcc.h` |
| Flash/SRAM サイズ設定 | 小 | 1.5MB Flash (0x180000), 320KB SRAM |
| ボード定義作成 | 中 | `boards/arm/stm32/spike-prime-hub/` |

### UART9/10 対応 (ポート E, F 使用に必要)

| 項目 | 工数 | 変更対象 |
|---|---|---|
| UART9/10 Kconfig 追加 | 小 | `arch/arm/src/stm32/Kconfig` |
| UART9/10 IRQ ベクタ追加 | 中 | `arch/arm/include/stm32/stm32f40xxx_irq.h` |
| UART9/10 RCC ビット追加 | 小 | `hardware/stm32f40xxx_rcc.h` |
| UART9/10 シリアルドライバ追加 | 中 | `stm32_serial.c` (UART7/8 パターンに従う) |

### 後回し可能

| 項目 | 理由 |
|---|---|
| FMPI2C ドライバ | SPIKE Hub は通常 I2C2 を使用 |
| DFSDM ドライバ | SPIKE Hub で未使用 |
| SAI1 ドライバ | SPIKE Hub で未使用 |
| CAN3 ドライバ | SPIKE Hub で未使用 |

### 推奨アプローチ

1. `STM32_STM32F413` ファミリ設定を追加し、F412 の全ペリフェラルに加えて UART4-10 を選択
2. F412 の RCC インフラを再利用 (既存の `#if` ガードに F413 を追加)
3. F412 ピンマップをベースに F413 用ピンマップヘッダを作成
4. UART9/10 サポートを UART7/8 パターンに従ってシリアルドライバに追加
5. UART9/10 の IRQ ベクタを追加 (IRQ 88, 89)
