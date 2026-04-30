# LEGO センサー uORB ドライバ

## 1. 概要

LUMP UART プロトコルエンジン (Issue #43) の上に乗る **NuttX uORB sensor** ドライバ (Issue #45)。`/dev/uorb/sensor_lego[0..5]` の 6 トピックを boot 時に static register し、Powered Up デバイス (LPF2 sensor + motor encoder telemetry) を固定 56 byte の `struct lump_sample_s` envelope で publish する。

mode 切替・writable mode TX (例: Color sensor の RGB LED 制御) は `sensor_lowerhalf_s::ops->control()` 経由の custom ioctl で行う。multi-subscriber + 単一 control owner の arbitration は `LEGOSENSOR_CLAIM/RELEASE` で管理。

## 2. アーキテクチャ

```
[user]   apps/legosensor/legosensor_main.c (CLI)
         apps/system/sensortest, ユーザーアプリ
   │ open(/dev/uorb/sensor_legoN) + read() + ioctl()
   │
[kernel uORB upper-half]  nuttx/drivers/sensors/sensor.c
   │ push_event() → ring buffer (nbuffer=16) → poll() wake
   │
[kernel lower-half]  boards/spike-prime-hub/src/legosensor_uorb.c
   │   per-port struct legosensor_dev_s
   │   - sensor_lowerhalf_s, mutex, claim_owner, generation, ...
   │ ↑ on_sync / on_data / on_error callbacks
   │
[kernel] LUMP engine (#43)  stm32_legoport_lump.c
   │
[kernel] DCM (#42)
```

### 2.1 register 方針

- **boot 時 static register**: 6 port 分 `sensor_custom_register("/dev/uorb/sensor_legoN", sizeof(struct lump_sample_s))` を一斉に実行
- 接続なし port は何も publish しない (subscriber は `poll()` で待つ)
- `lump_attach` 維持契約: physical disconnect で attach は剥がれない、engine 側で ERR/backoff/re-SYNC を内部処理

### 2.2 callback コンテキスト

- `on_sync` / `on_data` / `on_error` は LUMP **per-port kthread context**、LUMP 側 lock 解放後に呼ばれる
- driver 側は per-port `nxmutex_t` で state を保護、`push_event()` 呼出時には mutex を release してから (lock order 厳守、deadlock 回避)

## 3. Sample envelope

```c
struct lump_sample_s
{
  uint64_t timestamp;       /* μs (sensor framework 慣習) */
  uint32_t seq;             /* per-port monotonic counter */
  uint32_t generation;      /* attach / SELECT 成功 / disconnect で +1 */
  uint8_t  port;            /* 0..5 (A..F) */
  uint8_t  type_id;         /* LPF2 type (0 = disconnected sentinel) */
  uint8_t  mode_id;         /* DATA frame の mode */
  uint8_t  data_type;       /* enum lump_data_type_e */
  uint8_t  num_values;      /* INFO_FORMAT[2] */
  uint8_t  len;             /* data 有効 byte 数 (0 = sentinel) */
  uint16_t reserved;
  union
  {
    uint8_t  raw [32];
    int8_t   i8  [32];
    int16_t  i16 [16];
    int32_t  i32 [8];
    float    f32 [8];
  } data;
};
_Static_assert(sizeof(struct lump_sample_s) == 56, "ABI");
_Static_assert(offsetof(struct lump_sample_s, data) == 24, "ABI");
```

合計 56 byte 固定。Consumer は `data_type` で switch して `data.i16[k]` 等を直アクセスでき、cast boilerplate 不要。

### 3.1 Sentinel sample 規約

| sentinel 種別 | 条件 | 意味 |
|---|---|---|
| sync sentinel | `type_id != 0 && len == 0` | SYNC 完了通知 + upper-half circbuf の lazy allocation を pre-warm |
| disconnect sentinel | `type_id == 0 && len == 0` | DCM disconnect (`on_error`) — 以降は新規接続まで sample なし |

### 3.2 generation の意味論

| イベント | generation |
|---|---|
| 初回 SYNC 完了 | +1 (envelope 内に新値、sync sentinel として publish) |
| `LEGOSENSOR_SELECT` 成功確定 | +1 (engine が新 mode を観測した次回 `on_data` で bump) |
| SELECT 失敗 (timeout 500 ms) | bump せず (subscriber は valid sample を捨てない) |
| disconnect | +1 (disconnect sentinel として publish) |

Subscriber は SELECT 直前の generation を記録しておけば、generation 跳ね + `mode_id` 変化のタイミングで新 mode samples を識別可能。

## 4. Custom ioctl

| cmd | arg | 用途 | claim |
|---|---|---|---|
| `LEGOSENSOR_GET_INFO` | `lump_device_info_s *` | per-mode schema 一覧 | -- |
| `LEGOSENSOR_GET_STATUS` | `lump_status_full_s *` | engine 状態 (state/baud/RX/TX 等) | -- |
| `LEGOSENSOR_CLAIM` | (なし) | 制御権を取得 (filep を owner に) | -- |
| `LEGOSENSOR_RELEASE` | (なし) | 制御権解放 | owner only |
| `LEGOSENSOR_SELECT` | uint8_t mode | mode 切替 (`lump_select_mode`) | required |
| `LEGOSENSOR_SEND` | `legosensor_send_arg_s *` | writable mode TX (`lump_send_data`) | required |

### 4.1 標準 sensor ioctl

- `SNIOC_SET_INTERVAL`: upper-half が直接 `lower->ops->set_interval()` を呼ぶ。本 driver は no-op (LUMP cadence は device firmware 依存)
- `SNIOC_ACTIVATE` は NuttX 12.13.0 の sensor ioctl には**存在しない**。activation は `nsubscribers` count 経由で `lower->ops->activate()` が呼ばれる (本 driver は no-op、LUMP engine は常時動作)

### 4.2 CLAIM / RELEASE 排他モデル

- 設計: read (publish 受信) と GET_INFO/GET_STATUS は誰でも可、SELECT/SEND だけが claim 必須 (multi-subscriber + 単一 control owner)
- 既に他 fd が CLAIM 中 → `-EBUSY`
- 同 filep からの再 CLAIM は idempotent (no-op、OK)
- SELECT/SEND を CLAIM していない fd から発行 → `-EACCES`
- **auto-release**: per-fd `close()` で `sensor_ops_s::close(lower, filep)` が呼ばれ (NuttX 12.13.0 sensor.c:788, 814)、driver が `claim_owner == filep` なら NULL クリア。明示 RELEASE 忘れも安全
- DCM disconnect (`on_error`) 時は `claim_owner` も即座に NULL クリア

### 4.3 SELECT の API contract

`LEGOSENSOR_SELECT` ioctl は LUMP engine への "送信成功" を返すだけで、**実際の mode 確定は subscriber が `seq` / `generation` / `mode_id` を観測して判定する**。理由:

- `lump_select_mode` は engine の TX queue に積むだけ
- デバイスが mode を切替えるのに ~10 ms かかる
- 切替確認は次回 `on_data` で `mode_id == requested_mode` を見るしかない

driver 側は `pending_select_mode` + `pending_select_deadline = now + 500 ms` を per-port mutex 下で保持し、`on_data` で:

```c
nxmutex_lock(&priv->lock);
now_ticks = clock_systime_ticks();
if (priv->pending_select_mode != UINT8_MAX)
{
    if ((int32_t)(now_ticks - priv->pending_select_deadline) >= 0)
        priv->pending_select_mode = UINT8_MAX;     /* 期限切れ */
    else if (frame->mode == priv->pending_select_mode)
    {
        priv->generation++;
        priv->pending_select_mode = UINT8_MAX;     /* 確定 */
    }
}
priv->mode_id = frame->mode;
priv->seq++;
sample = build_envelope(priv, frame);              /* ローカルコピー */
nxmutex_unlock(&priv->lock);
priv->lower.push_event(priv->lower.priv, &sample, sizeof(sample));
```

`(int32_t)(now - deadline) >= 0` で signed delta 比較 → tick wraparound (`uint32_t`) safe。

## 5. CLI (`legosensor`)

```
legosensor                     6 port の type / state / mode / RX / TX 一覧
legosensor list                同上
legosensor info <N>            per-mode schema (name, num_values, data_type, raw/pct/si min-max, units, writable)
legosensor mode <N> <m>        CLAIM → SELECT → poll(2) 500 ms → RELEASE
legosensor send <N> <m> <hex>...  CLAIM → SEND (writable mode payload) → RELEASE
legosensor watch <N> [ms]      open(O_NONBLOCK) → poll(2) → read → decoded 表示
legosensor claim <N>           明示 CLAIM (fd close で auto-release)
legosensor release <N>         明示 RELEASE
```

`N` は `0..5` または `A..F`。

### 5.1 `dd` / `sensortest` が使えない理由

- **`dd`**: poll(2) を使わず単一 `read()` を発行する。upper-half の non-fetch read path は新 sample がない時に `-ENODATA` を即返すため、push_event との race で `Unknown error 61` (ENODATA) を返す
- **`sensortest`**: `g_sensor_info[]` table の標準 sensor type 名 (accel0, gyro0 等) しか受け付けない。`sensor_lego[N]` は custom path なので `The sensor node name:sensor_legoN is invalid` で reject される

`legosensor watch` が正しい end-to-end 検証ツール。

## 6. デバイス対応

### 6.1 検証済 (HW で SYNCED 確認)

| Type | 名称 | num_modes | デフォルト mode |
|---|---|---|---|
| 48 | SPIKE Medium Motor | 6 | POWER (mode 0) |
| 49 | SPIKE Large Motor | 6 | POWER (mode 0) |
| 61 | SPIKE Color Sensor | 8 | COLOR (mode 0) |
| 62 | SPIKE Ultrasonic Sensor | 8 | DISTL (mode 0) |
| 63 | SPIKE Force Sensor | 7 | FORCE (mode 0) |

### 6.2 LUMP プロトコル対応 (理論上動作)

Type ID の網羅的な一覧は [port-detection.md](port-detection.md) §5 を参照。`/dev/uorb/sensor_lego[N]` は LUMP 経由で動く全 device で機能する (Mindstorms EV3 系含む)。

### 6.3 #44 motor driver 依存事項 (LED / motor 物理駆動)

SPIKE Color Sensor / Ultrasonic Sensor の LED 制御 (mode 3 LIGHT / mode 5 LIGHT 等) と motor の PWM 駆動は **#44 H-bridge motor driver (TIM1/3/4)** に依存:

- pybricks 参考実装 (`pbio/drv/legodev/legodev_pup_uart.c:894-900`) では SYNC 完了後に H-bridge を `-MAX_DUTY` (Pin 1 supply) または `+MAX_DUTY` (Pin 2 supply) に駆動して LED 電源を供給する。`pbdrv_legodev_spec_basic_flags()` で `SPIKE_COLOR_SENSOR / SPIKE_ULTRASONIC_SENSOR = NEEDS_SUPPLY_PIN1`、`TECHNIC_COLOR_LIGHT_MATRIX = NEEDS_SUPPLY_PIN2` の対応を持つ
- 現バージョン (#45 単独) では H-bridge 未実装のため `LEGOSENSOR_SEND` の wire-level 送信は成功する (TX byte 増加で確認可能) が物理 LED は点灯しない
- motor の `SEND POWER` も同様に PWM 駆動なしでは回転しない
- `LEGOSENSOR_SEND` で frame は LUMP 経由で確実に送出される — 現在の制限は単に supply 電源が無いだけ。#44 完了後は同 ioctl で LED / motor 駆動が動作する想定

## 7. 設計選択

### 7.1 chardev (`/dev/legosensor[N]`) ではなく uORB primary

Codex 2 ラウンド review で `/dev/uorb/sensor_lego[N]` (uORB primary) が確定:

- IMU (`/dev/uorb/sensor_imu0`) と一貫した sensor framework 採用 → 同 `poll()` loop で fusion 可能
- multi-subscriber が natural (制御 app + monitor app の併存)
- ring buffer / batch / poll(2) を NuttX framework が提供
- LUMP の動的 mode + 動的 data shape は `SENSOR_TYPE_CUSTOM` の固定 envelope + C union で吸収可能

### 7.2 nbuffer = 16 (instance 全体)

- LUMP は ~10..100 Hz、subscriber は 100 ms read loop が普通
- 100 Hz × 100 ms loop で **欠落しない** 余裕として 16 frame
- per-instance の circbuf (per-subscriber ではない)、user 増加で RAM 線形増加なし
- RAM 計上: heap allocation `16 × 56 byte × 6 port = 5.4 KB` + upper-half/user 管理領域

### 7.3 `O_WROK` 拒否

`sensor_ops_s::open` で `(filep->f_oflags & O_WROK) != 0` の時 `-EACCES` を返す。userspace が `sensor_write()` で偽 sample を inject するのを防ぐ (LUMP transport をバイパスさせない)。

### 7.4 #44 motor driver との分離 (将来の Issue)

BOOST/Technic motor は **encoder telemetry を sensor として読みつつ PWM で同時駆動** が pybricks 標準ユースケース。よって以下を採用:

- **#43 LUMP は port に 1 engine + 複数 publisher attach 可能**
- **#45 は telemetry publisher** (boot 時に attach、physical disconnect で剥がれない)
- **#44 は SELECT/SEND の writer/control owner** (chardev 経由で PWM 駆動 + LUMP control)

#44 設計時に `lump_acquire_control(port, owner_token) / lump_release_control(port, owner_token)` を #43 に追加し、SELECT/SEND を token base に変更する想定。**#45 は boot 時に LUMP control owner を取得しない** (CLAIM ioctl 時に user fd が初めて取る)、legacy `lump_select_mode/send_data` (token なし) は将来 owner == NULL の時のみ許可 (NULL bypass にしない)。

## 8. tuning 定数

| 定数 | 値 | 説明 |
|---|---|---|
| `LEGOSENSOR_NUM_PORTS` | 6 | `/dev/uorb/sensor_lego0..5` |
| `LEGOSENSOR_MAX_DATA_BYTES` | 32 | LUMP frame payload 上限 |
| `LEGOSENSOR_NBUFFER` | 16 | upper-half circbuf depth |
| `LEGOSENSOR_PENDING_TIMEOUT_MS` | 500 | SELECT 確定待ち上限 |

## 9. 参照

- 設計詳細: [lump-protocol.md](lump-protocol.md), [port-detection.md](port-detection.md)
- 公開 ABI: `boards/spike-prime-hub/include/board_legosensor.h`
- driver 実装: `boards/spike-prime-hub/src/legosensor_uorb.c`
- CLI: `apps/legosensor/legosensor_main.c`
- NuttX sensor framework: `nuttx/include/nuttx/sensors/sensor.h`, `nuttx/drivers/sensors/sensor.c`
