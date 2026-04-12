# デバイスドライバ実装計画

## 1. 概要

SPIKE Prime Hub 上で NuttX からハードウェアを制御するためのデバイスドライバ群の実装計画。ブリングアップ調査の成果をもとに、段階的に実装する。

## 2. ドライバ一覧と優先度

| # | ドライバ | デバイスパス | 優先度 | 依存 |
|---|---|---|---|---|
| 1 | I/O ポート検出 (DCM) | `/dev/legoport[0-5]` | **P0** | GPIO |
| 2 | LUMP UART プロトコル | (内部) | **P0** | UART4/5/7/8/9/10 |
| 3 | H-Bridge モーター制御 | `/dev/legomotor[N]` | **P0** | TIM1/3/4 PWM |
| 4 | センサーデータ読取り | `/dev/legosensor[N]` | **P1** | LUMP |
| 5 | TLC5955 LED ドライバ | `/dev/leds` | **済** | SPI1 |
| 6 | IMU (LSM6DS3TR-C) | `/dev/imu0` | **済** | I2C2 |
| 7 | DAC オーディオ | `/dev/tone0` + `/dev/pcm0` | **済** | DAC1 + DMA1 + TIM6 |
| 8 | ADC バッテリー監視 | `/dev/bat0` | **済** | ADC1 (6ch) + DMA2 |
| 9 | USB CDC/ACM コンソール | `/dev/ttyACM0` | **済** | OTG FS |
| 10 | W25Q256 SPI Flash | `/dev/mtdblock0` | **P2** | SPI2 |
| 11 | 電源管理 | (board 初期化) | **済** | PA13/PA14 GPIO |
| 12 | MP2639A 充電制御 | `/dev/charge0` | **済** | TIM5 PWM + ADC + BCD |
| 13 | Bluetooth (CC256x) | -- | **P3** | USART2 + DMA |

### 優先度定義

- **P0**: 最低限のモーター・センサー動作に必須
- **P1**: 基本的なロボット操作に必要
- **P2**: データログ・ファームウェア更新に有用
- **P3**: 無線通信 (将来)

## 3. 実装フェーズ

### Phase 1: ポート基盤 (P0)

**目標**: I/O ポートのデバイス検出とモーター基本制御

#### 1a. 電源管理 (済)

PA13 (BAT_PWR_EN) と PA14 (PORT_3V3_EN) の初期化は `stm32_boot.c` に実装済み。

#### 1b. I/O ポート検出 (DCM)

- 6 ポート分のデバイス接続マネージャ (Device Connection Manager)
- 2ms 周期の GPIO ポーリングで接続デバイスをパッシブ検出
- 安定検出 (20 回連続一致 = 約 400ms) でデバイスタイプ確定
- NuttX の HPWORK キューでポーリング実装

#### 1c. LUMP UART プロトコル

- LEGO UART Messaging Protocol の実装
- 同期フェーズ: 2400 baud → デバイスモード情報取得 → 115200 baud 切替
- データフェーズ: 周期的データ受信 + キープアライブ (200ms)
- 専用カーネルスレッドで状態マシンを駆動

#### 1d. H-Bridge モーター制御

- PWM duty 設定 + GPIO/AF モード切替による方向制御
- Coast / Brake / Forward / Reverse の 4 状態
- duty 範囲: -1000 〜 +1000

### Phase 2: センサー・LED (P1)

#### 2a. センサーデータ読取り

- LUMP データフェーズで受信したセンサーデータを `/dev/legosensor[N]` で公開
- ioctl でモード切替、read() でデータ取得
- 対応: カラーセンサー (Type 61)、超音波センサー (Type 62)、フォースセンサー (Type 63)

#### 2b. TLC5955 LED ドライバ (済)

- SPI1 経由で 48ch 16bit PWM LED ドライバを制御
- 5x5 LED マトリクス (RGB x 25 = 75 ch のうち 48ch 使用)

#### 2c. IMU (LSM6DS3TR-C) (済)

- I2C2 経由、アドレス 0x6A、INT1 DRDY 割り込み
- 軸符号補正: X=-1, Y=+1, Z=-1

#### 2d. DAC オーディオ (済)

- DAC1 CH1 (PA4) → アンプ (PC10) → スピーカーのハードウェアパス
- TIM6 TRGO でサンプルレート制御、DMA1 Stream 5 Ch7 で `DAC1_DHR12L1` へ循環転送
- 低レベル層 `stm32_sound.c`: `stm32_sound_play_pcm/stop_pcm` (pybricks `pbdrv_sound_start` と 1:1 互換、冪等 stop)
- 高レベル層を 2 つの char device に分割:
    - **`/dev/tone0`**: カーネル内 pybricks 形式 tune パーサ (`"T120 C4/4 D#5/8. R/4 G4/4_"`)。`nxsig_usleep` 20 ms スライス + `atomic_bool stop_flag` で中断可能、`echo` から直接駆動可。
    - **`/dev/pcm0`**: 単一呼出 raw PCM ABI (`struct pcm_write_hdr_s` v1)。`magic/version/hdr_size/flags/sample_rate/sample_count` のヘッダ + `uint16_t` サンプル列を `write()` 一発で渡す。
- ioctl 空間: `TONEIOC_VOLUME_SET/GET/STOP` を `arch/board/board_sound.h` 内に `_BOARDIOC()` で定義 (NuttX 上流と衝突なし)
- `apps/sound` NSH ユーティリティで `beep` / `notes` / `volume` / `off` / `selftest` を提供
- STM32F413 固有の upstream 修正: `STM32_HAVE_DAC1` select と `GPIO_DAC1_OUT1_0` pinmap マクロを owhinata/nuttx fork に追加済み ([#27](https://github.com/owhinata/spike-nx/issues/27), [#28](https://github.com/owhinata/spike-nx/issues/28))。DMA/TIM は NuttX 抽象 API に移行済み ([#29](https://github.com/owhinata/spike-nx/issues/29))、DAC1 RCC enable のみ直接レジスタアクセスが残る
- 詳細: [`docs/ja/drivers/sound.md`](sound.md)

#### 2e. ADC バッテリー監視・ボタン入力 (済)

- ADC1 6 チャンネル + DMA2_Stream0 で 1kHz 連続変換（TIM2 トリガ）
- バッテリーゲージを `/dev/bat0` に登録（NuttX battery_gauge フレームワーク）
- 電圧・電流・温度（NTC）・SoC 推定
- 抵抗ラダーデコーダをセンターボタンと充電器 CHG 信号で共有

| チャンネル | ピン | 用途 |
|---|---|---|
| CH10 | PC0 | バッテリー電流 (最大 7300mA) |
| CH11 | PC1 | バッテリー電圧 (最大 9900mV) |
| CH8 | PB0 | バッテリー温度 (NTC サーミスタ) |
| CH3 | PA3 | USB 充電入力電流 |
| CH14 | PC4 | センターボタン + CHG 状態 (抵抗ラダー) |
| CH5 | PA1 | 左/右/BT ボタン (抵抗ラダー) |

**注意**: Hub のボタン入力は GPIO ではなく ADC 抵抗ラダー方式。抵抗ラダーデコーダで電圧レベルからボタン押下を判定する。

### Phase 3: ストレージ・充電 (P2)

#### 3a. W25Q256 SPI NOR Flash

- SPI2 経由で 32MB SPI NOR Flash にアクセス
- 4-byte アドレスモード
- NuttX の MTD ドライバ (`CONFIG_MTD_W25`) を使用
- LittleFS または SmartFS でフォーマット

#### 3b. MP2639A 充電制御 (済)

- 充電器を `/dev/charge0` に登録（NuttX battery_charger フレームワーク）
- MODE ピン: TLC5955 チャネル 14（充電有効化、アクティブ Low）
- ISET: TIM5 CH1 (PA0) 96kHz PWM（電流リミット制御）
- CHG 状態: ADC CH14 抵抗ラダーデコーダ（センターボタンと共有）
- USB BCD 検出を LPWORK で実行: SDP (500mA) / CDP (1.5A) / DCP (1.5A)
- 4Hz ポーリング: CHG フォルト検出、充電タイムアウト（60分→30秒停止）、バッテリー LED
- バッテリー LED: 赤=充電中、緑=満充電、緑点滅=完了、黄点滅=障害

### Phase 4: 無線通信 (P3)

#### 4a. Bluetooth (CC256x)

- USART2 (PD5/PD6) + フロー制御 (PD3/PD4)
- DMA1_Stream6/7 で送受信
- 詳細は TBD

## 4. ポート GPIO ピン割り当て

| ポート | UART TX | UART RX | AF | GPIO1 | GPIO2 | UART BUF |
|---|---|---|---|---|---|---|
| A | PE8 | PE7 | AF8 (UART7) | PA5 | PA3 | PB2 |
| B | PD1 | PD0 | AF11 (UART4) | PA4 | PA6 | PD3 |
| C | PE1 | PE0 | AF8 (UART8) | PB0 | PB14 | PD4 |
| D | PC12 | PD2 | AF8 (UART5) | PB4 | PB15 | PD7 |
| E | PE3 | PE2 | AF11 (UART10) | PC13 | PE12 | PB5 |
| F | PD15 | PD14 | AF11 (UART9) | PC14 | PE6 | PB10 |

## 5. DCM 検出アルゴリズム

```
1. GPIO1 を OUTPUT HIGH / LOW に切替えながら GPIO2 を読む
2. GPIO2 の応答パターンからデバイスカテゴリを判定:
   - 抵抗付き → パッシブデバイス (ライト、外部モーター等)
   - プルアップ → UART デバイス (スマートセンサー/モーター)
   - 無応答 → 未接続
3. UART デバイスの場合:
   - ピンを GPIO → UART AF に切替
   - UART BUF ピンを HIGH (RS485 トランシーバ有効化)
   - LUMP ハンドシェイク開始
```

## 6. ioctl インターフェース

### ポートマネージャ (`/dev/legoport[N]`)

```c
#define LEGOPORT_GET_DEVICE_TYPE    _LEGOPORTIOC(0)
#define LEGOPORT_GET_DEVICE_INFO    _LEGOPORTIOC(1)
#define LEGOPORT_WAIT_CONNECT       _LEGOPORTIOC(2)
#define LEGOPORT_WAIT_DISCONNECT    _LEGOPORTIOC(3)
```

### モーター (`/dev/legomotor[N]`)

```c
#define LEGOMOTOR_SET_DUTY          _LEGOMOTORIOC(0)  // int16: -1000〜+1000
#define LEGOMOTOR_COAST             _LEGOMOTORIOC(1)
#define LEGOMOTOR_BRAKE             _LEGOMOTORIOC(2)
#define LEGOMOTOR_GET_POSITION      _LEGOMOTORIOC(3)  // int32: 度数
#define LEGOMOTOR_GET_SPEED         _LEGOMOTORIOC(4)  // int16: deg/s
#define LEGOMOTOR_GET_ABS_POS       _LEGOMOTORIOC(5)  // int16: 絶対位置
#define LEGOMOTOR_RESET_POS         _LEGOMOTORIOC(6)
```

### センサー (`/dev/legosensor[N]`)

```c
#define LEGOSENSOR_SET_MODE         _LEGOSENSORIOC(0)
#define LEGOSENSOR_GET_MODE         _LEGOSENSORIOC(1)
#define LEGOSENSOR_GET_MODE_INFO    _LEGOSENSORIOC(2)
#define LEGOSENSOR_GET_DATA         _LEGOSENSORIOC(3)
```

## 7. ディレクトリ構成

```
boards/spike-prime-hub/src/
  stm32_boot.c          # 電源初期化 (PA13/PA14)
  stm32_bringup.c       # デバイスドライバ登録
  stm32_usbdev.c        # USB CDC/ACM
  stm32_legoport.c      # I/O ポートマネージャ (DCM)
  stm32_legomotor.c     # H-Bridge モーター制御
  stm32_tlc5955.c       # TLC5955 LED ドライバ
  stm32_lsm6dsl.c       # IMU (LSM6DS3TR-C) 初期化
  lsm6dsl_uorb.c        # IMU uORB パブリッシャ
  stm32_sound.c         # DAC1 低レベル PCM 再生 (stm32_sound_play_pcm/stop_pcm)
  stm32_sound.h         # board 内部共有状態 (g_sound: lock/owner/mode/volume/stop_flag)
  stm32_tone.c          # /dev/tone0 (カーネル内 pybricks tune パーサ)
  stm32_pcm.c           # /dev/pcm0 (単一呼出 raw PCM ABI v1)
  stm32_adc_dma.c       # ADC1 DMA 連続変換 (6ch, 1kHz)
  stm32_battery_gauge.c # バッテリーゲージ lower-half (/dev/bat0)
  stm32_battery_charger.c # MP2639A 充電器 lower-half (/dev/charge0)
  stm32_resistor_ladder.c # 抵抗ラダーデコーダ (ボタン + CHG)
  stm32_power.c         # センターボタン監視 + 電源制御

boards/spike-prime-hub/include/
  board_sound.h         # 公開 ABI: struct pcm_write_hdr_s + TONEIOC_*

apps/sound/             # NSH builtin "sound" (beep/tone/notes/volume/off/selftest)
  sound_main.c
  Kconfig
  Makefile
  Make.defs

drivers/lego/           # NuttX 汎用ドライバ (未実装)
  lump_uart.c           # LUMP UART プロトコルエンジン
  lump_uart.h
  legodev.h             # デバイスタイプ定義
  legomotor.c           # モーター上位ドライバ
  legosensor.c          # センサー上位ドライバ
```

## 8. 実装順序

```
Phase 1a: 電源管理          (済)
Phase 1b: DCM ポート検出    (次のタスク)
Phase 1c: LUMP UART         (1b と並行可能)
Phase 1d: H-Bridge モーター (1c 完了後)
    |
Phase 2a: センサー読取り    (1c 完了後)
Phase 2b: TLC5955 LED       (済)
Phase 2c: IMU               (済)
Phase 2d: DAC オーディオ    (済 - /dev/tone0 + /dev/pcm0 + apps/sound)
Phase 2e: ADC バッテリー    (済 - /dev/bat0)
    |
Phase 3a: W25Q256 Flash     (独立実装可能)
Phase 3b: MP2639A 充電      (済 - /dev/charge0, BCD on LPWORK)
    |
Phase 4a: Bluetooth         (将来)
```
