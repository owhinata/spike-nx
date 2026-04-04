# ペリフェラルドライバ調査

## サマリ

| # | ペリフェラル | NuttX ドライバ | Kconfig | 状態 | 工数 |
|---|---|---|---|---|---|
| 1 | USB OTG FS (CDC/ACM) | `stm32_otgfsdev.c` | `STM32_OTGFS`, `CDCACM` | 成熟。F4 で実績多数 | 低 |
| 2 | UART (USART1-3,6, UART4-8) | `stm32_serial.c` | `STM32_USARTn` / `STM32_UARTn` | UART9/10 未対応 | 低 (9/10 除く) |
| 3 | SPI (SPI1, SPI2) | `stm32_spi.c` | `STM32_SPI1`, `STM32_SPI2` | 成熟。DMA 対応 | 低 |
| 4 | ADC + DMA | `stm32_adc.c` | `STM32_ADC1`, `ADC` | マルチチャンネルスキャン + DMA | 低〜中 |
| 5 | I2C (I2C2) | `stm32_i2c.c` | `STM32_I2C2` | PB3 AF9 の非標準ピン設定が課題 | 中 |
| 6 | Timer/PWM | `stm32_pwm.c` | `STM32_TIMn`, `PWM` | TIM1/3/4 全対応 | 低 |
| 7 | DAC | `stm32_dac.c` | `STM32_DAC1`, `DAC` | PA4 標準マッピング | 低 |
| 8 | TLC5955 LED | **なし** | — | カスタムドライバ必須 | 高 |
| 9 | W25Q256 Flash | **部分的** | `MTD_W25` (SPI) | 32bit アドレス未対応。要ドライバ拡張 | 中 |
| 10 | LSM6DS3TR-C IMU | **部分的** | `SENSORS_LSM6DSL` | LSM6DSL ドライバで代用可能 (微修正要) | 中 |
| 11 | Bluetooth CC2564C | あり | `WIRELESS_BLUETOOTH`, `BLUETOOTH_UART` | FW パッチロード必要。初期段階では後回し | 中〜高 |

---

## 詳細

### 1. USB OTG FS (CDC/ACM) — HIGH

**ドライバ**: `arch/arm/src/stm32/stm32_otgfsdev.c` — STM32F4 で成熟。

**必要な Kconfig**:
```
CONFIG_STM32_OTGFS=y
CONFIG_USBDEV=y
CONFIG_CDCACM=y
CONFIG_CDCACM_CONSOLE=y
CONFIG_BOARDCTL_USBDEVCTRL=y
CONFIG_NSH_USBCONSOLE=y
CONFIG_NSH_USBCONDEV="/dev/ttyACM0"
```

**ピン**: PA11 (DM), PA12 (DP) — 標準マッピング、リマップ不要。

**実績ボード**: stm32f4discovery:usbnsh, stm32f411-minimum, nucleo-f446re

---

### 2. UART — HIGH

| ペリフェラル | 対応 | 備考 |
|---|---|---|
| USART1, 2, 3, 6 | 対応済み | 全 F4 共通 |
| UART4, 5 | 対応済み | F427/F429/F469 で有効 |
| UART7, 8 | 対応済み | IRQ, RCC, ドライバ全て実装済み |
| **UART9** | **未対応** | F413 固有。IRQ/RCC/ドライバ全て未実装 |
| **UART10** | **未対応** | 同上 |

**注意**: UART9/10 は APB2 クロック。ST HAL にもボーレート計算バグがあった (APB1 クロックで計算)。カスタムドライバでは正しいクロックソースを使用する必要あり。

SPIKE Hub ポート割当: A=UART7, B=UART4, C=UART8, D=UART5, E=UART10, F=UART9

---

### 3. SPI — MEDIUM

**ドライバ**: `arch/arm/src/stm32/stm32_spi.c` — SPI1-5 対応。

**Kconfig**:
```
CONFIG_SPI=y
CONFIG_STM32_SPI1=y          # TLC5955 LED コントローラ
CONFIG_STM32_SPI2=y          # W25Q256 Flash
CONFIG_STM32_SPI1_DMA=y      # オプション: DMA 転送
CONFIG_STM32_SPI2_DMA=y
```

**DMA**: 完全対応。バッファが DMA 非対応メモリ (CCM RAM 等) にある場合は自動的に PIO フォールバック。

**制限**: `CONFIG_STM32_SPI_INTERRUPT` と `CONFIG_STM32_SPIx_DMA` は排他的。

---

### 4. ADC + DMA — MEDIUM

**ドライバ**: `arch/arm/src/stm32/stm32_adc.c` — ADC1-3 対応。

**Kconfig**:
```
CONFIG_ANALOG=y
CONFIG_ADC=y
CONFIG_STM32_ADC1=y
CONFIG_STM32_DMA2=y           # ADC1 は DMA2 使用
CONFIG_ADC_FIFOSIZE=16        # リングバッファサイズ
```

**マルチチャンネル**: スキャンモード + DMA でバッテリー監視 6 チャンネルに対応可能。チャンネルリストとサンプル時間はボードレベルコード (`stm32_adc.c`) で設定。

---

### 5. I2C (I2C2) — MEDIUM

**ドライバ**: `arch/arm/src/stm32/stm32_i2c.c` — I2C1-3 対応。

**Kconfig**:
```
CONFIG_I2C=y
CONFIG_STM32_I2C2=y
```

**課題**: PB3 を I2C2_SDA (AF9) として使用するのは非標準。

- PB10 (SCL, AF4) — 標準マッピング、問題なし
- **PB3 (SDA, AF9)** — F413 固有の代替機能。ピンマップヘッダ (`stm32f413xx_pinmap.h`) で正しく定義する必要あり

ボード初期化時に I2C2 バスを登録し、LSM6DS3TR-C (アドレス 0x6A) をデバイスとして登録。

---

### 6. Timer/PWM — MEDIUM

**ドライバ**: `arch/arm/src/stm32/stm32_pwm.c` — 全タイマー対応。

**Kconfig (ポート A-B 用)**:
```
CONFIG_PWM=y
CONFIG_STM32_TIM1=y
CONFIG_STM32_TIM1_PWM=y
CONFIG_STM32_TIM1_CHANNEL=1   # CH1-4 を使用
```

**モーター制御の考慮事項**:
- TIM1 (高機能タイマー): 相補出力 + デッドタイム挿入対応
- TIM3, TIM4 (汎用タイマー): 相補出力なし。H-bridge は別 GPIO で方向制御
- 標準 PWM API (`/dev/pwmN`) ではブレーキ・コースト等の複雑なパターンにはボードレベルのカスタムロジックが必要

---

### 7. DAC — LOW

**ドライバ**: `arch/arm/src/stm32/stm32_dac.c`

**Kconfig**:
```
CONFIG_DAC=y
CONFIG_STM32_DAC1=y
```

PA4 = DAC_OUT1 は固定ピン (アナログモード)。タイマートリガーで DMA 連続変換対応 (音声波形生成向き)。

---

### 8. TLC5955 LED ドライバ — LOW

**NuttX ドライバなし。** TLC5955 は TI の 48 チャンネル 16bit PWM LED ドライバ。769bit シフトレジスタの非標準 SPI プロトコル。

**NuttX LED フレームワーク**: USERLED (GPIO)、RGBLED (PWM)、I2C LED コントローラ (PCA9635, LP3943) が存在するが、いずれも TLC5955 のプロトコルには非対応。

**実装方針**: SPI1 上にカスタムドライバを実装。キャラクタデバイスまたは LED サブシステムとして公開。

---

### 9. W25Q256 SPI NOR Flash — LOW

**2つのドライバが存在するが、いずれも課題あり:**

| ドライバ | Kconfig | 対応チップ | 課題 |
|---|---|---|---|
| `w25.c` (SPI) | `MTD_W25` | W25Q16-128 | **24bit アドレスのみ**。W25Q256 非対応 |
| `w25qxxxjv.c` (QSPI) | `W25QXXXJV` | W25Q016-Q01 (W25Q256 含む) | **QSPI 専用**。標準 SPI 非対応 |

**推奨アプローチ**:
1. `w25.c` を 4byte アドレス対応に拡張 (中程度の工数)
2. `w25qxxxjv.c` を標準 SPI で動作するよう改修
3. 一時的に 3byte モードで先頭 16MB のみ使用

---

### 10. LSM6DS3TR-C IMU — LOW

**部分対応**: `drivers/sensors/lsm6dsl.c` (キャラクタデバイス) と `drivers/sensors/lsm6dso32.c` (uORB) が存在。

- LSM6DS3TR-C は LSM6DSL と基本的にレジスタ互換
- WHO_AM_I レジスタ値の違いに対応する微修正が必要
- I2C アドレス 0x6A
- Kconfig: `CONFIG_SENSORS_LSM6DSL=y`

---

### 11. Bluetooth CC2564C — LOW

**ドライバあり**: NuttX に HCI UART Bluetooth サポートと CC2564 固有の初期化シーケンスが存在。

**Kconfig**:
```
CONFIG_WIRELESS_BLUETOOTH=y
CONFIG_BLUETOOTH_UART=y
```

**課題**:
- CC2564C は起動時にファームウェアパッチロード (~7KB) が必要
- RTS/CTS ハードウェアフロー制御必須 (USART2: TX=PD5, RX=PD6, CTS=PD3, RTS=PD4)
- 初期段階では対象外。フル機能実現時に対応

---

## リスクまとめ

1. **F413 チップ定義の追加が最優先** — ペリフェラルドライバはチップ定義なしでは使用不可
2. **I2C2 PB3/AF9** — 非標準ピンマッピングが最も陥りやすい設定ミス
3. **W25Q256** — 既存ドライバでは 32bit アドレスに非対応。ドライバ開発が必要
4. **TLC5955** — 完全なカスタムドライバが必要
5. **UART9/10** — NuttX への新規追加が必要 (UART7/8 パターンに従う)
6. **標準ペリフェラル** (USB, SPI, ADC, DAC, PWM, I2C) は成熟したドライバが利用可能
