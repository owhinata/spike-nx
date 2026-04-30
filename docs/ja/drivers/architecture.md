# デバイスドライバアーキテクチャ

## 1. レイヤードアーキテクチャ

```
                    ユーザー空間
  ┌─────────────────────────────────────────────┐
  │  /dev/legoport[0-5]        (DCM)             │
  │  /dev/uorb/sensor_<class>  (uORB センサー)   │
  │  /dev/legomotor[N]         (モーター制御)    │
  └────────────┬───────────────┬────────────────┘
               │               │
           カーネル空間         │
  ┌────────────┴───────────────┴────────────────┐
  │  LEGO ポートマネージャ (上位層)              │
  │  - ポート毎の状態マシンスレッド              │
  │  - DCM パッシブ検出 (2ms ポーリング)         │
  │  - 動的 UART 初期化/解放                    │
  │  - デバイスタイプ通知                        │
  ├─────────────────────────────────────────────┤
  │  LUMP UART プロトコルエンジン (中間層)       │
  │  - 同期ハンドシェイク                        │
  │  - モード情報パース                          │
  │  - データ送受信                              │
  │  - キープアライブ管理                        │
  ├─────────────────────────────────────────────┤
  │  STM32 UART + GPIO (下位層 / HAL)           │
  │  - stm32_configgpio() ピンモード切替         │
  │  - stm32_serial UART 通信                    │
  │  - HPWORK キュー (2ms DCM ポーリング)        │
  └─────────────────────────────────────────────┘
```

## 2. デバイスノード設計

### 永続デバイス (起動時に登録)

| デバイス | パス | 用途 |
|---|---|---|
| ポートマネージャ | `/dev/legoport0` 〜 `5` | ポート状態管理、デバイス検出通知 |

### LUMP センサー uORB トピック (boot 時に登録)

| デバイス | パス | LPF2 type_id | 用途 |
|---|---|---|---|
| カラーセンサー | `/dev/uorb/sensor_color` | 61 | LPF2 Color の telemetry。56 byte `struct lump_sample_s` envelope で publish。Issue #79 |
| 超音波センサー | `/dev/uorb/sensor_ultrasonic` | 62 | LPF2 Ultrasonic の telemetry |
| フォースセンサー | `/dev/uorb/sensor_force` | 63 | LPF2 Force の telemetry |
| Module/Arm Motor | `/dev/uorb/sensor_motor_m` | 49 | SPIKE Large Motor (アーム/マニピュレーター、高トルク) の encoder + status |
| 右輪 Motor | `/dev/uorb/sensor_motor_r` | 48 (port A/C/E) | SPIKE Medium Motor (driving wheel、port パリティで分離) |
| 左輪 Motor | `/dev/uorb/sensor_motor_l` | 48 (port B/D/F) | 同上 |

各 class topic は **同時に最大 1 port** を bind (1 topic = 1 port ルール)。複数 port が同 class に分類された場合は port 番号の小さい方が勝ち、残りは frame drop。詳細は [sensor.md](sensor.md) §2.2。

接続なしクラスも常時登録、`type_id=0,len=0` の sentinel sample で disconnect / takeover を通知 (subscriber fd の一貫性維持)。

### 動的デバイス (将来)

| デバイス | パス | 登録条件 |
|---|---|---|
| モーター | `/dev/legomotor0` 〜 | LUMP でモーターデバイス検出時 (Issue #44) |

## 3. ホットプラグライフサイクル

```
起動:
  /dev/legoport[N] を登録
  DCM ポーリング開始 (HPWORK, 2ms)

接続検出:
  DCM で安定タイプ検出 (20 回連続一致, ~400ms)
  ├─ UART デバイスの場合:
  │   ピンを GPIO → UART AF に切替 (stm32_configgpio)
  │   UART ドライバを動的初期化
  │   LUMP 同期ハンドシェイク実行
  │   成功 → /dev/legosensor[N] or /dev/legomotor[N] を登録
  │   データ受信ループ開始
  └─ パッシブデバイスの場合:
      即座にデバイスノード登録

切断検出:
  UART: 600ms データタイムアウト
  unregister_driver() でデバイスノード削除
  UART リセット、シリアルドライバ解放
  ピンを GPIO モードに戻す
  DCM ポーリング再開
```

## 4. UART 所有権戦略

### 課題

UART ピンはデバイス検出時に GPIO として、通信時に UART AF として使用される。NuttX シリアルドライバと競合する可能性がある。

### 方針

1. **起動時に I/O ポート UART のシリアルドライバを初期化しない**
2. ポートマネージャがピンを所有。DCM 中は `stm32_configgpio()` で GPIO 操作
3. UART デバイス検出後、`stm32_configgpio()` で AF モードに切替え、軽量 UART ドライバを初期化
4. LUMP エンジンは低レベル UART アクセス (バイト単位 TX/RX、ボーレート切替) が必要なため、STM32 UART レジスタの薄いラッパーを使用

### GPIO ↔ UART 動的切替

```c
// GPIO 入力モード (デバイス検出時)
stm32_configgpio(GPIO_INPUT | GPIO_FLOAT | GPIO_PORTE | GPIO_PIN8);

// UART AF モード (UART デバイス通信時)
stm32_configgpio(GPIO_AF8 | GPIO_PORTE | GPIO_PIN8);  // UART7_TX
```

## 5. スレッドモデル

| コンポーネント | 方式 | 理由 |
|---|---|---|
| DCM ポーリング | HPWORK (2ms) | 単純な GPIO 操作、専用スレッド不要 |
| LUMP プロトコル | 専用カーネルスレッド | 複雑な状態マシン + タイミング要件 (100ms keepalive, 250ms IO timeout) |

## 6. データフォーマット

LUMP プロトコルの同期フェーズでデータフォーマットが自己記述される:

| フィールド | 説明 |
|---|---|
| `num_values` | データ要素数 (int8: 1-32, int16: 1-16, int32/float: 1-8) |
| `data_type` | int8 / int16 / int32 / float (リトルエンディアン) |
| `raw_min/max` | 生データの範囲 |
| `pct_min/max` | パーセント範囲 |
| `si_min/max` | SI 単位範囲 |
| `units` | 単位文字列 |

最大ペイロード: 32 bytes/モード。

NuttX ドライバは `ioctl(LEGOSENSOR_GET_MODE_INFO)` でこのメタデータをユーザー空間に公開し、アプリケーションが生データを正しく解釈できるようにする。

## 7. ioctl インターフェース

### ポートマネージャ (`/dev/legoport[N]`)

```c
#define LEGOPORT_GET_DEVICE_TYPE    _LEGOPORTIOC(0)   // 接続デバイスタイプ取得
#define LEGOPORT_GET_DEVICE_INFO    _LEGOPORTIOC(1)   // モード数、フラグ
#define LEGOPORT_WAIT_CONNECT       _LEGOPORTIOC(2)   // デバイス接続まで待機
#define LEGOPORT_WAIT_DISCONNECT    _LEGOPORTIOC(3)   // デバイス切断まで待機
```

### センサー (`/dev/legosensor[N]`)

```c
#define LEGOSENSOR_SET_MODE         _LEGOSENSORIOC(0) // モード切替
#define LEGOSENSOR_GET_MODE         _LEGOSENSORIOC(1) // 現在モード取得
#define LEGOSENSOR_GET_MODE_INFO    _LEGOSENSORIOC(2) // モード情報 (値数、型、名前)
#define LEGOSENSOR_GET_DATA         _LEGOSENSORIOC(3) // 生データ取得
```

`read()` でも現在モードのデータを取得可能 (最大 32 bytes)。

### モーター (`/dev/legomotor[N]`)

```c
#define LEGOMOTOR_SET_DUTY          _LEGOMOTORIOC(0)  // int16: -1000〜+1000
#define LEGOMOTOR_COAST             _LEGOMOTORIOC(1)  // 惰性
#define LEGOMOTOR_BRAKE             _LEGOMOTORIOC(2)  // 制動
#define LEGOMOTOR_GET_POSITION      _LEGOMOTORIOC(3)  // int32: 度数
#define LEGOMOTOR_GET_SPEED         _LEGOMOTORIOC(4)  // int16: deg/s
#define LEGOMOTOR_GET_ABS_POS       _LEGOMOTORIOC(5)  // int16: 絶対位置
#define LEGOMOTOR_RESET_POS         _LEGOMOTORIOC(6)  // 位置リセット
```

## 8. SPIKE センサー詳細

### カラーセンサー (Type 61)

| モード | 名前 | データ | 方向 |
|---|---|---|---|
| 0 | COLOR | 1x int8 | Read: 検出色インデックス |
| 1 | REFLT | 1x int8 | Read: 反射光強度 |
| 2 | AMBI | 1x int8 | Read: 環境光強度 |
| 3 | LIGHT | 3x int8 | **Write**: LED 輝度設定 |
| 5 | RGB_I | 4x int16 | Read: RGBI 値 |
| 6 | HSV | 3x int16 | Read: HSV 値 |

### 超音波センサー (Type 62)

| モード | 名前 | データ | 方向 |
|---|---|---|---|
| 0 | DISTL | 1x int16 | Read: 距離 (遠距離), mm |
| 1 | DISTS | 1x int16 | Read: 距離 (近距離), mm |
| 3 | LISTN | 1x int8 | Read: 超音波リスニング |
| 5 | LIGHT | 4x int8 | **Write**: LED 輝度設定 |

### フォースセンサー (Type 63)

| モード | 名前 | データ | 方向 |
|---|---|---|---|
| 4 | FRAW | 1x int16 | Read: 生の力センサー値 |

## 9. 参照ファイル

- `pybricks/lib/pbio/drv/legodev/legodev_pup.c` -- ポートレベルドライバ
- `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c` -- LUMP UART プロトコル (1265 行)
- `pybricks/lib/pbio/drv/legodev/legodev_spec.c` -- デバイス固有知識
- `pybricks/lib/pbio/drv/motor_driver/motor_driver_hbridge_pwm.c` -- H-bridge ドライバ
- `pybricks/lib/pbio/include/pbdrv/legodev.h` -- デバイスタイプ定義、API
- `pybricks/lib/lego/lego_uart.h` -- LUMP プロトコル定義
