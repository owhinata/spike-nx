# LED ドライバ (TLC5955)

SPIKE Prime Hub の LED は TI TLC5955 (48 チャネル SPI PWM LED ドライバ) で制御する。GPIO 直結の LED は存在しない。

## ハードウェア構成

| 項目 | 設定 |
|------|------|
| LED ドライバ IC | TI TLC5955 (48ch, 16-bit PWM) |
| SPI | SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (AF5), 24 MHz |
| LAT (ラッチ) | PA15 — GPIO 出力, HIGH→LOW エッジでラッチ |
| GSCLK | TIM12 CH2 = PB15 (AF9), 9.6 MHz PWM |
| シフトレジスタ | 769 ビット (97 バイト) |

### 電源

LED はバッテリー電源で駆動される。**USB 給電のみでは LED は点灯しない**。

### Control Latch 設定

pybricks と同じ設定を使用:

| パラメータ | 値 | 説明 |
|------------|-----|------|
| Dot Correction (DC) | 127 | 100% |
| Max Current (MC) | 3.2 mA | 最小設定 |
| Global Brightness (BC) | 127 | 100% |
| Auto Display Repeat | ON | |
| Display Timing Reset | OFF | |
| Auto Refresh | OFF | |
| ES-PWM | ON | 拡張スペクトラム PWM |
| LSD Detection | 90% | |

Control Latch は初期化時に 2 回送信する（TLC5955 のハードウェア要件: max current を反映するため）。

## LED チャネルマッピング

チャネルは GS レジスタに逆順でマッピングされる (CH0→GSB15, CH47→GSR0)。

### ステータス LED (RGB)

| LED | R | G | B |
|-----|---|---|---|
| Status Top (中央ボタン上) | CH5 | CH4 | CH3 |
| Status Bottom (中央ボタン下) | CH8 | CH7 | CH6 |
| Battery | CH2 | CH1 | CH0 |
| Bluetooth | CH20 | CH19 | CH18 |

### 5x5 LED マトリクス

| 行 | Col0 | Col1 | Col2 | Col3 | Col4 |
|----|------|------|------|------|------|
| 0 | CH38 | CH36 | CH41 | CH46 | CH33 |
| 1 | CH37 | CH28 | CH39 | CH47 | CH21 |
| 2 | CH24 | CH29 | CH31 | CH45 | CH23 |
| 3 | CH26 | CH27 | CH32 | CH34 | CH22 |
| 4 | CH25 | CH40 | CH30 | CH35 | CH9 |

## API

```c
#include "spike_prime_hub.h"

/* 初期化 (stm32_bringup で自動呼び出し) */
int tlc5955_initialize(void);

/* チャネルの PWM 値を設定 (0=OFF, 0xFFFF=最大輝度) */
void tlc5955_set_duty(uint8_t ch, uint16_t value);

/* 遅延更新: HPWORK キューで SPI 転送をスケジュール */
int tlc5955_update(void);

/* 即時更新: 初期化/シャットダウン用 */
int tlc5955_update_sync(void);
```

### 使用例

```c
/* 中央ボタン LED を緑に設定 */
tlc5955_set_duty(TLC5955_CH_STATUS_TOP_G, 0xffff);
tlc5955_set_duty(TLC5955_CH_STATUS_BTM_G, 0xffff);
tlc5955_update();

/* Bluetooth LED を青に設定 */
tlc5955_set_duty(TLC5955_CH_BT_B, 0x8000);
tlc5955_update();
```

## defconfig

```
CONFIG_STM32_SPI1=y
CONFIG_STM32_SPI1_DMA=y
CONFIG_STM32_DMA2=y
```

## 更新方式

`tlc5955_set_duty()` はデータをバッファに書き込み `changed` フラグを立てるだけで、SPI 転送は行わない。`tlc5955_update()` を呼ぶと HPWORK キューに遅延転送がスケジュールされ、複数の `set_duty` 呼び出しが 1 回の SPI 転送にバッチ化される。

初期化時やシャットダウン時など即時反映が必要な場合は `tlc5955_update_sync()` を使用する。

## pybricks との比較

| 項目 | pybricks | NuttX |
|------|----------|-------|
| SPI 転送 | HAL SPI + DMA (非同期) | NuttX SPI ドライバ + DMA (同期) |
| GSCLK | TIM12 CH2 (HAL PWM) | TIM12 CH2 (レジスタ直接操作) |
| LAT | HAL GPIO | stm32_gpiowrite() |
| 更新方式 | Contiki プロトスレッド + changed フラグ | HPWORK キュー + changed フラグ |
| Control Latch | 同一パラメータ | 同一パラメータ |

SPI1 DMA チャネルマッピング (`DMACHAN_SPI1_RX/TX`) は NuttX に未定義だったため、`board.h` で定義:

- RX: DMA2 Stream2 Channel 3 (`DMAMAP_SPI1_RX_2`)
- TX: DMA2 Stream3 Channel 3 (`DMAMAP_SPI1_TX_1`)

## 対象ファイル

- `boards/spike-prime-hub/src/stm32_tlc5955.c` — ドライバ実装
- `boards/spike-prime-hub/src/spike_prime_hub.h` — チャネル定義
