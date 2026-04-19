# SPIKE Prime Hub ペリフェラル詳細

---

## TLC5955 LED ドライバ

### 概要

| 項目 | 値 |
|---|---|
| デバイス | TLC5955 (Texas Instruments) |
| チャンネル数 | 48 (16 RGB グループ x 3) |
| PWM 分解能 | 16 bit |
| シフトレジスタ長 | 769 bit (97 bytes) |
| 接続 | SPI1 + DMA2 |
| 最大電流設定 | 3.2 mA (`TLC5955_MC_3_2`) |

### 通信プロトコル

TLC5955 は 769bit シフトレジスタの非標準 SPI プロトコルで通信する。データ送信後に LAT ピン (PA15) をトグルしてラッチする。

#### 制御データラッチ (769 bit)

| ビット位置 | フィールド | 説明 |
|---|---|---|
| 768 | ラッチ選択 | `1` = 制御データ |
| 767-760 | マジックバイト | `0x96` (識別用) |
| 370 | LSDVLT | LED 短絡検出電圧 |
| 369 | ESPWM | ES-PWM モード (有効推奨) |
| 368 | RFRESH | 自動リフレッシュ |
| 367 | TMGRST | タイミングリセット |
| 366 | DSPRPT | 自動表示リピート (有効推奨) |
| 365-345 | BC (3x7bit) | 全体輝度制御 (Blue, Green, Red) |
| 344-336 | MC (3x3bit) | 最大電流設定 (Blue, Green, Red) |
| 335-0 | DC (48x7bit) | ドット補正 (チャンネル別) |

#### グレースケールデータラッチ (769 bit)

| ビット位置 | フィールド | 説明 |
|---|---|---|
| 768 | ラッチ選択 | `0` = グレースケールデータ |
| 767-0 | GS データ | 48 チャンネル x 16 bit = 768 bit |

チャンネル順序: CH47 (MSB 側) → CH0 (LSB 側)。逆順マッピング。

### 初期化シーケンス

1. 制御データラッチ (97 bytes) を SPI DMA で送信
2. LAT ピン (PA15) をトグル (HIGH → LOW)
3. 制御データラッチを**再送** (最大電流設定を確定するため 2 回送信が必要)
4. LAT 再トグル
5. グレースケール更新ループ: 値変更時にグレースケールデータ送信 + LAT トグル

SPIKE Hub 設定: ES-PWM 有効、自動表示リピート有効、最大電流 3.2 mA。

### チャンネルマッピング

#### 5x5 LED マトリクス (単色, 25 チャンネル)

| | Col 0 | Col 1 | Col 2 | Col 3 | Col 4 |
|---|---|---|---|---|---|
| Row 0 | CH38 | CH36 | CH41 | CH46 | CH33 |
| Row 1 | CH37 | CH28 | CH39 | CH47 | CH21 |
| Row 2 | CH24 | CH29 | CH31 | CH45 | CH23 |
| Row 3 | CH26 | CH27 | CH32 | CH34 | CH22 |
| Row 4 | CH25 | CH40 | CH30 | CH35 | CH9 |

#### ステータス LED (RGB, 12 チャンネル)

| LED | R | G | B |
|---|---|---|---|
| Status Top (中央ボタン上) | CH5 | CH4 | CH3 |
| Status Bottom (中央ボタン下) | CH8 | CH7 | CH6 |
| バッテリーインジケータ | CH2 | CH1 | CH0 |
| Bluetooth インジケータ | CH20 | CH19 | CH18 |

色補正係数: R=1000, G=170, B=200。

#### チャンネル使用状況

- 25 チャンネル: 5x5 マトリクス (単色)
- 12 チャンネル: ステータス LED (4 個 x RGB)
- 11 チャンネル: 未使用 (CH10-17, CH42-44)
- 合計: 48 チャンネル

---

## W25Q256 外部 SPI NOR Flash

### 概要

| 項目 | 値 |
|---|---|
| デバイス | W25Q256 (Winbond) |
| 容量 | 32 MB (256 Mbit) |
| インターフェース | SPI (標準/Dual/Quad) |
| ページサイズ | 256 bytes |
| セクタサイズ | 4 KB |
| ブロックサイズ | 64 KB |
| デバイス ID | `0xEF` `0x40` `0x19` |
| 接続 | SPI2 + DMA1 |
| CS | PB12 (ソフトウェア NSS) |

### 4byte アドレッシング

W25Q256 は 32 MB のため 4byte アドレッシングが必要。pybricks は 4byte 専用コマンドを使用する方式 (アドレスモードレジスタに依存しないため堅牢):

| 操作 | 4byte コマンド | 備考 |
|---|---|---|
| Fast Read | `0x0C` | `[0x0C] [addr3-0] [dummy] [data...]` |
| Page Program | `0x12` | 256 bytes 単位 |
| Sector Erase | `0x21` | 4 KB 単位 |

### フラッシュメモリレイアウト

| 領域 | アドレス | サイズ | 用途 |
|---|---|---|---|
| ブートローダーデータ | `0x000000` - `0x07FFFF` | 512 KB | LEGO ブートローダー領域 |
| pybricks ブロックデバイス | `0x080000` - `0x0BFFFF` | 256 KB | ユーザープログラムバックアップ |
| 更新キー | `0x0FF000` - `0x0FFFFF` | 4 KB | mboot FS-load キー |
| ファイルシステム | `0x100000` - `0x1FFFFFF` | 31 MB | FAT ファイルシステム |

NuttX では先頭 1 MB を予約域として扱い、1 MB 以降をファイルシステムに使用するのが安全。

### NuttX ドライバ対応

NuttX 上流の `w25.c` (MTD_W25) は 3byte アドレスのみ (W25Q128 まで) で W25Q256 非対応。`w25qxxxjv.c` は QSPI 専用で標準 SPI 非対応。

**実装済み**: `boards/spike-prime-hub/src/stm32_w25q256.c` に board-local の専用 MTD ドライバを新規実装。常に 4byte 専用コマンド (Fast Read `0x0C` / Page Program `0x12` / Sector Erase `0x21`) を使用し、address mode register の状態に依存しない設計 (pybricks 同等)。先頭 1 MB を予約、`0x100000` 以降 31 MB を `mtd_partition()` で切り出して `/dev/mtdblock0` として LittleFS マウント (`/mnt/flash`)。詳細は [W25Q256 ドライバ](../drivers/w25q256.md) を参照。

---

## LSM6DS3TR-C IMU (6 軸慣性計測ユニット)

### 概要

| 項目 | 値 |
|---|---|
| デバイス | LSM6DS3TR-C (STMicroelectronics) |
| 機能 | 3 軸加速度センサー + 3 軸ジャイロスコープ |
| 接続 | I2C2 (SCL=PB10, SDA=PB3) |
| I2C アドレス | `0x6A` |

### レジスタ設定

| レジスタ | 設定 | 説明 |
|---|---|---|
| CTRL3_C | BDU + IF_INC | Block Data Update 有効 + アドレス自動インクリメント有効 |
| INT1_CTRL | DRDY | Data Ready 割込みを INT1 に出力 |

### データ読み取り

12 byte バースト読取りで加速度 (3 軸 x 2 bytes) + ジャイロ (3 軸 x 2 bytes) を一括取得。

### NuttX ドライバ対応

既存の `lsm6dsl.c` (`CONFIG_SENSORS_LSM6DSL`) で代用可能。LSM6DS3TR-C は LSM6DSL と基本的にレジスタ互換。WHO_AM_I レジスタ値の違いに対応する微修正が必要。

---

## CC2564C Bluetooth

### 概要

| 項目 | 値 |
|---|---|
| デバイス | CC2564C (Texas Instruments、BR/EDR + BLE デュアルモード) |
| 接続 | USART2 (TX=PD5, RX=PD6, CTS=PD3, RTS=PD4) |
| フロー制御 | RTS/CTS ハードウェアフロー制御必須 |
| DMA | TX: DMA1 Stream6 Ch4, RX: DMA1 Stream7 Ch6 (VERY_HIGH) |
| NVIC 優先度 | 0xA0 (USART2 + DMA1 S6 + DMA1 S7、Issue #50 予約枠) |
| nSHUTD (chip enable) | PA2 (GPIO output、Active HIGH) |
| 32.768 kHz slow clock | PC9 = TIM8 CH4 (AF3)、CC2564C の sleep clock として必須 |

### 起動シーケンス

1. **32.768 kHz slow clock 供給** — TIM8 CH4 PWM (50% duty) を安定化 (nSHUTD HIGH 前)
2. **nSHUTD トグル** — LOW → 50 ms → HIGH (chip reset → boot 開始)
3. **ROM boot 待機** (≈150 ms)
4. **ファームウェアパッチロード** — 約 6.6 KB の `.bts` init script を HCI コマンドで順送信 (USART2 115200 bps)
5. **ボーレート切替** — `HCI_VS_Update_UART_HCI_Baud_Rate` (opcode 0xFF36) で 3 Mbps に切替
6. **HCI stack 開始** — `bt_hcicore.c` 経由で netdev 登録

### 状態

Issue #47 で実装中。`boards/spike-prime-hub/src/stm32_btuart.c` で board-local の `struct btuart_lowerhalf_s` を自前実装し、NuttX generic upper-half (`CONFIG_BLUETOOTH_UART_OTHER`) と組み合わせて `CONFIG_NET_BLUETOOTH` 経路で netdev 登録する方針。

---

## ADC (バッテリー監視)

### 概要

| 項目 | 値 |
|---|---|
| ペリフェラル | ADC1 |
| DMA | DMA2 |
| チャンネル数 | 6 チャンネル (バッテリー監視) |

マルチチャンネルスキャンモード + DMA でバッテリー電圧を監視。チャンネルリストとサンプル時間はボードレベルコードで設定する。

---

## DAC (スピーカー出力)

### 概要

| 項目 | 値 |
|---|---|
| ペリフェラル | DAC1 |
| 出力ピン | PA4 (DAC_OUT1, 固定) |

タイマートリガーで DMA 連続変換に対応。音声波形生成に使用可能。
