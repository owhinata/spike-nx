# 電源制御

SPIKE Prime Hub の電源制御と中央ボタンによるシャットダウンについて説明する。

## ハードウェア構成

### 電源制御

| ピン | 機能 | 説明 |
|------|------|------|
| PA13 | BAT_PWR_EN | HIGH: 電源 ON, LOW: 電源 OFF |
| PA14 | PORT_3V3_EN | I/O ポート 3.3V 電源イネーブル |

PA13 は `stm32_boardinitialize()` で起動直後に HIGH に設定される。LOW にするとバッテリーからの電源供給が切断される。

### 中央ボタン (ADC 抵抗ラダー)

中央ボタンは GPIO ではなく、抵抗ラダー回路を経由して ADC で読み取る。

| 項目 | 値 |
|------|-----|
| GPIO | PC4 (アナログ入力) |
| ADC チャネル | ADC1 CH14 (注: PC4 は STM32F413 で CH14) |
| 未押下時 ADC 値 | ~3645 |
| 押下時 ADC 値 | ~2872 |
| 判定閾値 | 3200 未満で押下 |

抵抗ラダー回路 (pybricks 参照):
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

## 動作

### 電源 ON

LEGO ブートローダーが処理する。中央ボタンを押すと MCU に電源が供給され、ブートローダーがファームウェアをロードする。

### 通常動作

- NSH 起動時に中央ボタン LED が緑に点灯
- 中央ボタンの状態を 50ms 間隔で HPWORK キューでポーリング
- 起動後 3 秒間はボタン監視を無効化（誤検出防止）

### 電源 OFF (長押し 2 秒以上)

1. 中央ボタン LED が青に変わる（シャットダウン表示）
2. ボタンリリースを待機
3. PA13 (BAT_PWR_EN) を LOW に設定 → 電源切断
4. USB 接続中は USB 給電で MCU が動作し続けるため、`board_reset()` でリセット

### DFU モード

DFU モード中は LEGO ブートローダーが Bluetooth LED を虹色に点灯する（バッテリー接続時）。

## ADC DMA 連続変換

ボタン読み取りは ADC1 DMA 連続変換で行う。TIM2 TRGO をトリガーとし、6 チャネルを 1 kHz で連続スキャンする。

| ランク | チャネル | ピン | 用途 |
|--------|----------|------|------|
| 0 | CH10 | PC0 | バッテリー電流 (IBAT) |
| 1 | CH11 | PC1 | バッテリー電圧 (VBAT) |
| 2 | CH8 | PB0 | バッテリー温度 (NTC) |
| 3 | CH3 | PA3 | USB 充電器電流 (IBUSBCH) |
| 4 | CH14 | PC4 | 中央ボタン抵抗ラダー |
| 5 | CH5 | PA1 | 左/右/BT ボタン抵抗ラダー |

DMA バッファは DMA2 Stream0 で循環更新される。`stm32_adc_read(rank)` で最新の値を即座に取得できる。

## defconfig

```
CONFIG_STM32_ADC1=y
```

## pybricks との比較

| 項目 | pybricks | NuttX |
|------|----------|-------|
| 電源 OFF トリガー | 中央ボタン 2 秒長押し | 同一 |
| ADC 読み取り | HAL ADC + DMA + TIM2 トリガー | ADC: レジスタ直接操作、DMA + TIM2: NuttX 抽象 API |
| ADC スキャンレート | 1 kHz (6 チャネル) | 1 kHz (6 チャネル) |
| ボタンポーリング | Contiki イベントループ (50ms) | HPWORK キュー (50ms) |
| USB 接続時の電源 OFF | 電源は切れない (op-amp 依存) | PA13 LOW 後リセット |
| ADC クロック | APB2/4 = 24 MHz | APB2/4 = 24 MHz |

## 対象ファイル

- `boards/spike-prime-hub/src/stm32_adc_dma.c` — ADC DMA 連続変換
- `boards/spike-prime-hub/src/stm32_power.c` — 電源制御・ボタン監視
- `boards/spike-prime-hub/src/spike_prime_hub.h` — GPIO/ADC ランク定義
- `boards/spike-prime-hub/src/stm32_boot.c` — PA13/PA14 初期設定
