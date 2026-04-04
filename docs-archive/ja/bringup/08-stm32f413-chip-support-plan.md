# STM32F413 チップサポート追加計画

## 1. 結論: NuttX フォークが不可避

Out-of-tree ボード定義だけでは STM32F413 をサポートできない。以下の理由からカーネルレベルの変更が必須:

- チップバリアント (`CONFIG_ARCH_CHIP_STM32F413`) が Kconfig に存在しない
- UART9/10 のインフラ (Kconfig, IRQ, RCC, シリアルドライバ) が一切ない
- 割り込みベクタテーブルサイズはカーネルのコンパイル時定数
- RCC クロック有効化コードはカーネル側

**推奨**: apache/nuttx のフォークを作成し、F413 サポートパッチを適用。定期的に upstream にリベース。将来的に upstream PR として提出も検討。

---

## 2. UART9/10 ハードウェア詳細

### IRQ 番号 (stm32f413xx.h で確認済み)

| ペリフェラル | IRQ 番号 | ソース |
|---|---|---|
| UART4 | 52 | APB1 |
| UART5 | 53 | APB1 |
| UART7 | 82 | APB1 |
| UART8 | 83 | APB1 |
| **UART9** | **88** | **APB2** |
| **UART10** | **89** | **APB2** |

### ベースアドレス

| ペリフェラル | バス | ベースアドレス |
|---|---|---|
| UART7 | APB1 | 0x40007800 |
| UART8 | APB1 | 0x40007C00 |
| **UART9** | **APB2** | **0x40011800** |
| **UART10** | **APB2** | **0x40011C00** |

**重要**: UART9/10 は **APB2 バス**上にある (APB1 ではない)。ボーレート計算は PCLK2 (96 MHz) を使用する必要がある。USART1/USART6 と同じバス。

### RCC 有効化ビット

| ペリフェラル | レジスタ | ビット位置 | マスク |
|---|---|---|---|
| UART9 | RCC_APB2ENR | Bit 6 | 0x00000040 |
| UART10 | RCC_APB2ENR | Bit 7 | 0x00000080 |

### レジスタレイアウト

UART9/10 は他の USART/UART と**同一の USART_TypeDef レジスタレイアウト**を使用。NuttX の既存レジスタアクセスコードはそのまま動作する。

---

## 3. Alternate Function テーブル (F413 固有)

### UART9 ピンオプション

| 機能 | ピン | AF |
|---|---|---|
| UART9_TX | PD15 | AF11 |
| UART9_RX | PD14 | AF11 |
| UART9_TX | PG1 | AF11 |
| UART9_RX | PG0 | AF11 |

### UART10 ピンオプション

| 機能 | ピン | AF |
|---|---|---|
| UART10_TX | PE3 | AF11 |
| UART10_RX | PE2 | AF11 |
| UART10_TX | PG12 | AF11 |
| UART10_RX | PG11 | AF11 |

### I2C2 SDA on PB3

**確認済み**: PB3 = I2C2_SDA は **AF9**。`stm32f413_af.csv` で確認。

### SPIKE Hub ポートの AF 割当

| ポート | UART | AF | 備考 |
|---|---|---|---|
| A | UART7 | AF8 | 標準 |
| B | UART4 | AF11 | F413 固有の代替 AF (通常は AF8) |
| C | UART8 | AF8 | 標準 |
| D | UART5 | AF8 | 標準 |
| E | UART10 | AF11 | F413 固有 |
| F | UART9 | AF11 | F413 固有 |

---

## 4. 変更が必要な NuttX カーネルファイル

### 必須変更 (最小パッチセット)

| # | ファイル | 変更内容 |
|---|---|---|
| 1 | `arch/arm/src/stm32/Kconfig` | `STM32_STM32F413` ファミリ設定、`ARCH_CHIP_STM32F413VG` 等、`STM32_HAVE_UART9/10` 追加 |
| 2 | `arch/arm/include/stm32/chip.h` | F413 のペリフェラル数、Flash/SRAM サイズ定義 |
| 3 | `arch/arm/include/stm32/stm32f40xxx_irq.h` | UART9 IRQ=88, UART10 IRQ=89, `STM32_IRQ_NEXTINT` 更新 |
| 4 | `arch/arm/src/stm32/hardware/stm32f40xxx_memorymap.h` | UART9/10 ベースアドレス追加 |
| 5 | `arch/arm/src/stm32/hardware/stm32f40xxx_rcc.h` | APB2ENR に UART9/10 ビット追加 |
| 6 | `arch/arm/src/stm32/stm32f40xxx_rcc.c` | `rcc_enableapb2()` に UART9/10 クロック有効化追加 |
| 7 | `arch/arm/src/stm32/stm32_serial.c` | UART9/10 デバイスインスタンス追加 (UART7/8 パターン) |
| 8 | `arch/arm/src/stm32/hardware/stm32f413xx_pinmap.h` | 新規作成: F413 固有ピンマップ (F412 ベース + UART9/10, AF11 等) |
| 9 | `arch/arm/src/stm32/hardware/stm32_pinmap.h` | F413 ピンマップ include 分岐追加 |
| 10 | `boards/Kconfig` | ボードエントリ追加 |

### 初期ブリングアップの代替戦略

NuttX フォーク作成前に F412 設定で初期動作確認を行う:

```
CONFIG_ARCH_CHIP_STM32F412ZG=y
```

**制約**: Flash 1MB / SRAM 256KB として認識。UART9/10 使用不可。基本的な NSH + UART7 コンソール + USB CDC/ACM は動作する見込み。

---

## 5. フォーク運用方針

### リポジトリ構成

```
spike-nx/
├── nuttx/          # git submodule: owhinata/nuttx (フォーク、f413-support ブランチ)
├── nuttx-apps/     # git submodule: owhinata/nuttx-apps (タグ nuttx-12.12.0)
├── boards/         # カスタムボード定義 (out-of-tree)
└── ...
```

### ブランチ戦略

- `main`: apache/nuttx の nuttx-12.12.0 をベース
- `f413-support`: F413 チップサポートパッチを適用
- upstream 更新時: `main` をリベース → `f413-support` をリベース

### upstream PR の可能性

F413 は広く使われているチップ (STM32F413H-Discovery は Zephyr でサポート済み)。NuttX upstream への PR が受理される可能性は高い。
