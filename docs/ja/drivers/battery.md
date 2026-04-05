# バッテリー・充電ドライバ

SPIKE Prime Hub は 2S Li-ion バッテリーパック（公称 7.2V）と MPS MP2639A USB バッテリー充電 IC を搭載しています。2つの NuttX ドライバでバッテリー監視と充電制御を行います。

## ハードウェア構成

### バッテリーセンシング（ADC）

全てのバッテリー計測は既存の ADC1 DMA 連続変換（1 kHz、TIM2 トリガー）を使用します。

| 計測項目 | ADC チャネル | ピン | ランク | スケーリング |
|----------|-------------|------|--------|-------------|
| バッテリー電流 | CH10 | PC0 | 0 | raw × 7300 / 4096 mA |
| バッテリー電圧 | CH11 | PC1 | 1 | raw × 9900 / 4096 + I × 3/16 mV |
| バッテリー温度 | CH8 | PB0 | 2 | NTC サーミスタ (103AT) |
| USB 充電電流 | CH3 | PA3 | 3 | (raw × 35116 >> 16) − 123 mA |

- **電圧補正**: 経路抵抗（0.1875 Ω）による電圧降下を電流値で補正
- **起動抑制**: 起動後 1 秒以内に測定電圧が 7000 mV 未満の場合、7000 mV を報告（電源投入時の不安定な値を抑制）
- **温度**: 103AT NTC サーミスタの B パラメータ方程式（B=3435、R0=10k @ 25℃、分圧回路 2.4k/7.5k）

### MP2639A 充電 IC

| 信号 | 接続先 | 説明 |
|------|--------|------|
| MODE | TLC5955 チャネル 14 | 充電有効化（LED ドライバ経由、アクティブ Low） |
| ISET | TIM5 CH1 (PA0, AF2) | 充電電流リミット（96 kHz PWM） |
| /CHG | 抵抗ラダー DEV_0 CH2 | 充電ステータス（センターボタン ADC と共有） |
| IB | ADC CH3 (PA3) | USB 充電電流計測 |

#### ISET 電流リミット（PWM デューティ）

| デューティ | 電流リミット | 用途 |
|-----------|-------------|------|
| 0% | 0 mA | 無効 |
| 2% | 100 mA | USB 標準最小 |
| 15% | 500 mA | USB 標準最大 |
| 100% | 1.5 A | 専用充電器 |

#### CHG ステータス検出

/CHG ピンはセンターボタンと共有の抵抗ラダー（PC4、ADC ランク 4）上にあります。抵抗ラダーデコーダがチャネル 2 として CHG 信号を抽出します。

- **CHG on**（ピン Low）: 充電中
- **CHG off**（ピン High、安定後）: 充電完了
- **CHG 点滅**（〜1 Hz）: 充電器障害

充電器は 4 Hz（250 ms 間隔）で CHG 信号をポーリングし、7 サンプルの巡回バッファを使用します。ウィンドウ内で 2 回以上の遷移が検出された場合、障害状態を報告します。

#### USB 検出 (BCD)

VBUS 検出時に USB Battery Charging Detection (BCD) で充電器タイプを識別します:

| BCD タイプ | 電流リミット | 説明 |
|-----------|-------------|------|
| SDP | 500 mA | 標準 USB ポート |
| CDP | 1.5 A | 充電対応 USB ポート |
| DCP | 1.5 A | 専用充電器 |
| Non-standard | 1.5 A | 非標準充電器 |

BCD は LPWORK（低優先度ワークキュー）で実行され、HPWORK（IMU・ボタン監視）をブロックしません。検出には約 300ms かかり、その間 USB PHY が一時停止します。検出完了後に CDC/ACM が自動復帰します。

#### 充電タイムアウト

60 分間連続充電後、30 秒間充電を停止してから再開します。満充電後に正常に再起動しない充電器-バッテリーの組み合わせに対応します。

## NuttX デバイスインターフェース

### バッテリーゲージ（`/dev/bat0`）

`battery_gauge_register()` で登録。サポートする IOCTL コマンド:

| IOCTL | 説明 | 単位 |
|-------|------|------|
| `BATIOC_STATE` | バッテリー状態 | `BATTERY_DISCHARGING` |
| `BATIOC_ONLINE` | バッテリー有無 | 常に `true` |
| `BATIOC_VOLTAGE` | バッテリー電圧 | mV |
| `BATIOC_CURRENT` | バッテリー電流 | mA |
| `BATIOC_CAPACITY` | 充電率 | %（電圧ベース推定） |
| `BATIOC_TEMPERATURE` | バッテリー温度 | ミリ℃ |

### バッテリー充電器（`/dev/charge0`）

`battery_charger_register()` で登録。サポートする IOCTL コマンド:

| IOCTL | 説明 | 単位 |
|-------|------|------|
| `BATIOC_STATE` | 充電器状態 | `BATTERY_CHARGING` / `BATTERY_FULL` / `BATTERY_DISCHARGING` / `BATTERY_FAULT` |
| `BATIOC_HEALTH` | 充電器ヘルス | `BATTERY_HEALTH_GOOD` / `BATTERY_HEALTH_UNSPEC_FAIL` |
| `BATIOC_ONLINE` | USB 接続 | `true` / `false` |
| `BATIOC_CURRENT` | 充電電流設定 | 0 / 100 / 500 / 1500 mA |
| `BATIOC_VOLTAGE_INFO` | 目標充電電圧 | 8400 mV |
| `BATIOC_CHIPID` | 充電器チップ ID | `0x2639` |

## バッテリー LED 表示

バッテリー LED（TLC5955 チャネル 0-2: B/G/R）は充電器ドライバの 4 Hz ポーリングで更新されます。

| 充電器状態 | 条件 | LED 色 | パターン |
|-----------|------|--------|---------|
| DISCHARGING | USB 未接続 | OFF | - |
| CHARGING | 電圧 < 8190 mV | 赤 | 常時点灯 |
| CHARGING | 電圧 >= 8190 mV | 緑 | 常時点灯 |
| COMPLETE | 充電完了 | 緑 | 点滅（2.75 秒 on / 0.25 秒 off） |
| FAULT | 充電器エラー | 黄 | 点滅（0.5 秒 on / 0.5 秒 off） |

満充電判定閾値（8190 mV = 4.095V/セル）は指数移動平均（127/128 係数）で算出し、ちらつきを防止します。

## テストアプリ

`battery` コマンドで IOCTL 経由でゲージ・充電器のステータスを読み取ります。

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

| コマンド | 説明 |
|---------|------|
| `battery` | ゲージ + 充電器の全情報表示 |
| `battery gauge` | ゲージのみ |
| `battery charger` | 充電器のみ |
| `battery monitor [N]` | 1 秒間隔で N 回モニタリング（デフォルト: 10） |

`CONFIG_APP_BATTERY=y` で有効化。

## ソースファイル

| ファイル | 説明 |
|---------|------|
| [`stm32_battery_gauge.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_battery_gauge.c) | バッテリーゲージ lower-half ドライバ |
| [`stm32_battery_charger.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_battery_charger.c) | MP2639A 充電器 lower-half ドライバ |
| [`stm32_resistor_ladder.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_resistor_ladder.c) | 抵抗ラダーデコーダ |
| [`stm32_adc_dma.c`](https://github.com/owhinata/spike-nx/blob/main/boards/spike-prime-hub/src/stm32_adc_dma.c) | ADC DMA 連続変換 |

## Pybricks リファレンス

移植元:

- [`battery_adc.c`](https://github.com/pybricks/pybricks-micropython/blob/v3.6.1/lib/pbio/drv/battery/battery_adc.c) — バッテリー電圧/電流/温度
- [`charger_mp2639a.c`](https://github.com/pybricks/pybricks-micropython/blob/v3.6.1/lib/pbio/drv/charger/charger_mp2639a.c) — MP2639A 充電器制御
- [`resistor_ladder.c`](https://github.com/pybricks/pybricks-micropython/blob/v3.6.1/lib/pbio/drv/resistor_ladder/resistor_ladder.c) — 抵抗ラダーデコーダ

### Pybricks との差分

| 機能 | Pybricks | NuttX 移植版 |
|------|----------|-------------|
| USB 検出 | フル BCD（Contiki protothread） | フル BCD（NuttX LPWORK） |
| デフォルト電流リミット | BCD タイプに基づく | BCD タイプに基づく |
| フレームワーク | カスタムドライバ API | NuttX battery gauge/charger |
| ポーリング | Contiki protothread | NuttX HPWORK キュー |

## STM32F413 OTG FS 修正

NuttX の OTG FS ドライバは STM32F413 をレガシー F4 として扱い、GCCFG bits 18/19 に VBUS sensing（VBUSASEN/VBUSBSEN）を設定していた。F413 ではこれらは BCD 用ビット（DCDEN/PDEN）であり、USB ケーブルの抜き差しが失敗する原因となっていた。

NuttX submodule に以下の修正を適用:

- **GCCFG**: `PWRDWN` のみ設定（BCD ビットに触らない）
- **GOTGCTL**: `BVALOEN | BVALOVAL` で B-session を強制（`NOVBUSSENS` の代替）
- **SEDET/SRQ**: Session end / Session request 割り込みハンドラを実装

変更ファイル: `arch/arm/src/stm32/stm32_otgfsdev.c`、`hardware/stm32fxxxxx_otgfs.h`
