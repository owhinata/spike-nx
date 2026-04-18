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

## ユーザ空間 API (`/dev/rgbled0`)

`CONFIG_BUILD_PROTECTED=y` では user blob から kernel シンボルを直接呼べないため、TLC5955 へのアクセスは char ドライバ `/dev/rgbled0` の ioctl 経由で行う (Issue #39)。

```c
#include <fcntl.h>
#include <sys/ioctl.h>
#include <arch/board/board_rgbled.h>

int fd = open("/dev/rgbled0", O_RDWR);

/* 単一チャネルの duty 設定 */
struct rgbled_duty_s d = { .channel = TLC5955_CH_STATUS_TOP_G,
                           .value   = 0xffff };
ioctl(fd, RGBLEDIOC_SETDUTY, (unsigned long)&d);

/* 全 48 チャネル一括 (arg に uint16_t の duty 値) */
ioctl(fd, RGBLEDIOC_SETALL, 0);         /* 全消灯 */

/* 即時 SPI 同期 (HPWORK を待たずに転送) */
ioctl(fd, RGBLEDIOC_UPDATE, 0);
```

| ioctl | arg 型 | 説明 |
|-------|--------|------|
| `RGBLEDIOC_SETDUTY` | `struct rgbled_duty_s *` | `channel` (0..47) を `value` (0..0xffff) に設定 |
| `RGBLEDIOC_SETALL` | `uint16_t` (arg 直接) | 全 48 チャネルを同一 duty に設定 |
| `RGBLEDIOC_UPDATE` | 0 | `tlc5955_update_sync()` を呼び即時反映 |

`board_rgbled.h` は `TLC5955_NUM_CHANNELS` および `TLC5955_CH_*` 定数も公開する (旧来は `spike_prime_hub.h` に定義)。

## カーネル内部 API

`stm32_bringup` や board 内部コードはカーネル内部関数を直呼びできる:

```c
#include "spike_prime_hub.h"

/* 初期化 (stm32_bringup で自動呼び出し) */
int tlc5955_initialize(void);

/* チャネルの PWM 値を設定 (0=OFF, 0xFFFF=最大輝度)
 * HPWORK キューで SPI 転送を自動スケジュール。
 * 複数の set_duty 呼び出しは 1 回の SPI 転送にバッチ化される。
 */
void tlc5955_set_duty(uint8_t ch, uint16_t value);

/* 即時更新: 初期化/シャットダウン用 (HPWORK 未稼働時に使用) */
int tlc5955_update_sync(void);

/* /dev/rgbled0 を登録 (stm32_bringup で自動呼び出し) */
int stm32_rgbled_register(void);
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
| GSCLK | TIM12 CH2 (HAL PWM) | TIM12 CH2 (NuttX TIM API) |
| LAT | HAL GPIO | stm32_gpiowrite() |
| 更新方式 | Contiki プロトスレッド + changed フラグ | HPWORK キュー + changed フラグ |
| Control Latch | 同一パラメータ | 同一パラメータ |

SPI1 DMA チャネルマッピング (`DMACHAN_SPI1_RX/TX`) は NuttX に未定義だったため、`board.h` で定義:

- RX: DMA2 Stream2 Channel 3 (`DMAMAP_SPI1_RX_2`)
- TX: DMA2 Stream3 Channel 3 (`DMAMAP_SPI1_TX_1`)

## テストアプリ

`led` NSH コマンドで全 LED 機能をテストできる:

```
led green     - ステータス LED を緑に点灯 (起動時のデフォルト)
led status    - ステータス LED の色サイクル: R → G → B → 白 → 消灯
led battery   - バッテリー LED の色サイクル
led bluetooth - Bluetooth LED の色サイクル
led rainbow   - レインボーアニメーション (HSV hue sweep)
led blink     - 緑のブリンク
led breathe   - ブリージング (フェードイン/アウト)
led matrix    - 5x5 マトリクス: 全点灯 → スキャン → 数字 0-9
led all       - 全テスト実行
led off       - 全 LED 消灯
```

## 対象ファイル

- `boards/spike-prime-hub/src/stm32_tlc5955.c` — ドライバ実装 (SPI + HPWORK)
- `boards/spike-prime-hub/src/stm32_rgbled.c` — `/dev/rgbled0` char ドライバ (ioctl 薄ラッパ)
- `boards/spike-prime-hub/include/board_rgbled.h` — チャネル定数・ioctl ABI (user/kernel 共有)
- `apps/led/led_main.c` — LED テストアプリ (ioctl ベース)
