# デバイスドライバ実装計画

## 1. 概要

SPIKE Prime Hub 上で NuttX からハードウェアを制御するためのデバイスドライバ群の実装計画。ブリングアップ調査 (bringup/) の成果をもとに、段階的に実装する。

---

## 2. ドライバ一覧と優先度

| # | ドライバ | デバイスパス | 優先度 | 依存 |
|---|---|---|---|---|
| 1 | I/O ポート検出 (DCM) | `/dev/legoport[0-5]` | **P0** | GPIO |
| 2 | LUMP UART プロトコル | (内部) | **P0** | UART4/5/7/8/9/10 |
| 3 | H-Bridge モーター制御 | `/dev/legomotor[N]` | **P0** | TIM1/3/4 PWM |
| 4 | センサーデータ読取り | `/dev/legosensor[N]` | **P1** | LUMP |
| 5 | TLC5955 LED ドライバ | `/dev/leds` | **P1** | SPI1 |
| 6 | USB CDC/ACM コンソール | `/dev/ttyACM0` | **済** | OTG FS |
| 7 | W25Q256 SPI Flash | `/dev/mtdblock0` | **P2** | SPI2 |
| 8 | 電源管理 | (board 初期化) | **P0** | PA13/PA14 GPIO |
| 9 | Bluetooth (TBD) | — | **P3** | USART2 |

### 優先度定義

- **P0**: 最低限のモーター・センサー動作に必須
- **P1**: 基本的なロボット操作に必要
- **P2**: データログ・ファームウェア更新に有用
- **P3**: 無線通信（将来）

---

## 3. 実装フェーズ

### Phase 1: ポート基盤 (P0)

**目標**: I/O ポートのデバイス検出とモーター基本制御

#### 1a. 電源管理 (済)

PA13 (BAT_PWR_EN) と PA14 (PORT_3V3_EN) の初期化は `stm32_boot.c` に実装済み。

#### 1b. I/O ポート検出 (DCM)

pybricks 参照: `legodev_pup.c`

- 6 ポート分のデバイス接続マネージャ (Device Connection Manager)
- 2ms 周期の GPIO ポーリングで接続デバイスをパッシブ検出
- 安定検出 (20 回連続一致 ≈ 400ms) でデバイスタイプ確定
- NuttX の HPWORK キューでポーリング実装

**ポート GPIO ピン割り当て:**

| ポート | UART TX | UART RX | AF | GPIO1 | GPIO2 | UART BUF |
|---|---|---|---|---|---|---|
| A | PE8 | PE7 | AF8 (UART7) | PA5 | PA3 | PB2 |
| B | PD1 | PD0 | AF11 (UART4) | PA4 | PA6 | PD3 |
| C | PE1 | PE0 | AF8 (UART8) | PB0 | PB14 | PD4 |
| D | PC12 | PD2 | AF8 (UART5) | PB4 | PB15 | PD7 |
| E | PE3 | PE2 | AF11 (UART10) | PC13 | PE12 | PB5 |
| F | PD15 | PD14 | AF11 (UART9) | PC14 | PE6 | PB10 |

**DCM 検出アルゴリズム:**
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

#### 1c. LUMP UART プロトコル

pybricks 参照: `legodev_pup_uart.c` (1265 行)

- LEGO UART Messaging Protocol の実装
- 同期フェーズ: 2400 baud → デバイスモード情報取得 → 115200 baud 切替
- データフェーズ: 周期的データ受信 + キープアライブ (200ms)
- 専用カーネルスレッドで状態マシンを駆動

**状態マシン:**
```
RESET → SYNC_TYPE → SYNC_MODES → SYNC_DATA → ACK → DATA
  ↑                                                  |
  └──────── タイムアウト / エラー ←──────────────────┘
```

**UART 所有権**: ポートマネージャが UART の初期化/解放を制御。NuttX の標準シリアルドライバ (`/dev/ttySx`) は I/O ポート UART には使用しない。低レベル UART レジスタアクセスの薄いラッパーを実装。

#### 1d. H-Bridge モーター制御

pybricks 参照: `motor_driver_hbridge_pwm.c`

- PWM duty 設定 + GPIO/AF モード切替による方向制御
- Coast / Brake / Forward / Reverse の 4 状態
- duty 範囲: -1000 〜 +1000

**タイマー割り当て:**

| タイマー | ポート | M1 ピン | M2 ピン | PWM 周波数 |
|---|---|---|---|---|
| TIM1 | A, B | PE9/CH1, PE13/CH3 | PE11/CH2, PE14/CH4 | 12 kHz |
| TIM3 | E, F | PC6/CH1, PC8/CH3 | PC7/CH2, PB1/CH4 | 12 kHz |
| TIM4 | C, D | PB6/CH1, PB8/CH3 | PB7/CH2, PB9/CH4 | 12 kHz |

### Phase 2: センサー・LED (P1)

#### 2a. センサーデータ読取り

- LUMP データフェーズで受信したセンサーデータを `/dev/legosensor[N]` で公開
- ioctl でモード切替、read() でデータ取得
- 対応センサー:
  - カラーセンサー (Type 61): COLOR, REFLT, AMBI, RGB_I, HSV
  - 超音波センサー (Type 62): DISTL, DISTS
  - フォースセンサー (Type 63): FRAW

#### 2b. TLC5955 LED ドライバ

pybricks 参照: `led_dual_pwm.c`, bringup 調査: `10-tlc5955-led-driver.md`

- SPI1 経由で 48ch 16bit PWM LED ドライバを制御
- 5×5 LED マトリクス (RGB × 25 = 75 ch のうち 48ch 使用)
- LATCH ピン: GPIO で制御
- NuttX の `/dev/leds` または `/dev/userleds` インターフェース

### Phase 3: ストレージ (P2)

#### 3a. W25Q256 SPI NOR Flash

bringup 調査: `11-w25q256-flash-driver.md`

- SPI2 経由で 32MB SPI NOR Flash にアクセス
- 4-byte アドレスモード
- NuttX の MTD ドライバ (`CONFIG_MTD_W25`) を使用
- LittleFS または SmartFS でフォーマット
- ユーザープログラム・データログの保存先

### Phase 4: 無線通信 (P3)

#### 4a. Bluetooth

- USART2 (PD5/PD6) 経由の BLE モジュール
- 詳細は TBD

---

## 4. ディレクトリ構成

```
boards/spike-prime-hub/src/
  stm32_boot.c          # 電源初期化 (PA13/PA14) ← 済
  stm32_bringup.c       # デバイスドライバ登録
  stm32_usbdev.c        # USB CDC/ACM ← 済
  stm32_legoport.c      # I/O ポートマネージャ (DCM)
  stm32_legomotor.c     # H-Bridge モーター制御
  stm32_tlc5955.c       # TLC5955 LED ドライバ

drivers/lego/           # NuttX 汎用ドライバ (apps/ または board 内)
  lump_uart.c           # LUMP UART プロトコルエンジン
  lump_uart.h
  legodev.h             # デバイスタイプ定義
  legomotor.c           # モーター上位ドライバ
  legosensor.c          # センサー上位ドライバ
```

---

## 5. ioctl インターフェース

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

---

## 6. 実装順序

```
Phase 1a: 電源管理          ← 済
Phase 1b: DCM ポート検出    ← 次のタスク
Phase 1c: LUMP UART         ← 1b と並行可能
Phase 1d: H-Bridge モーター ← 1c 完了後
    ↓
Phase 2a: センサー読取り    ← 1c 完了後
Phase 2b: TLC5955 LED       ← 独立実装可能
    ↓
Phase 3a: W25Q256 Flash     ← 独立実装可能
    ↓
Phase 4a: Bluetooth         ← 将来
```

---

## 7. 検証方法

### Phase 1 検証

```bash
# Discovery Kit で検証 (PU デバイスを直接接続可能な治具が必要)

# ポート検出確認
nsh> cat /dev/legoport0
Type: 65 (SPIKE Large Motor)

# モーター制御
nsh> echo 500 > /dev/legomotor0   # 50% 正転
nsh> echo 0 > /dev/legomotor0     # 停止 (brake)
nsh> echo -500 > /dev/legomotor0  # 50% 逆転
```

### Phase 2 検証

```bash
# センサー読取り
nsh> cat /dev/legosensor0
Color: 3 (Blue)

# LED 制御
nsh> echo "1,0,0" > /dev/leds     # 赤色表示
```
