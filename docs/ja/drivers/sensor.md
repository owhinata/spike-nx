# LEGO センサー uORB ドライバ

## 1. 概要

LUMP UART プロトコルエンジン (Issue #43) の上に乗る **NuttX uORB sensor** ドライバ (Issue #79)。**デバイスクラス単位** に topic を持ち、`/dev/uorb/sensor_color` を開けば必ずカラーセンサーが、`/dev/uorb/sensor_motor_r` を開けば必ず右輪モーターが流れる。

| Topic | LPF2 type_id | 接続条件 |
|---|---|---|
| `/dev/uorb/sensor_color` | 61 | 任意 port |
| `/dev/uorb/sensor_ultrasonic` | 62 | 任意 port |
| `/dev/uorb/sensor_force` | 63 | 任意 port |
| `/dev/uorb/sensor_motor_m` | 49 (SPIKE Large Motor) | 任意 port |
| `/dev/uorb/sensor_motor_r` | 48 (SPIKE Medium Motor) | port 0 / 2 / 4 (A / C / E) |
| `/dev/uorb/sensor_motor_l` | 48 (SPIKE Medium Motor) | port 1 / 3 / 5 (B / D / F) |

接尾辞: `_m` = アーム/マニピュレーター用 (type 49 = Large、高トルク)、`_r` / `_l` = 右輪/左輪 (driving wheel、type 48 = Medium を port パリティで分離)。未マップ type_id (38 / 46 / 47 / 65 / 75 / 76 等) は frame drop。

mode 切替・writable mode TX (LED / 書込 mode 送信) は `sensor_lowerhalf_s::ops->control()` 経由の custom ioctl で行う。multi-subscriber + 単一 control owner の arbitration は `LEGOSENSOR_CLAIM/RELEASE` で管理。

## 2. アーキテクチャ

```
[user]   apps/legosensor/legosensor_main.c (CLI)
   │ open(/dev/uorb/sensor_<class>) + read() + ioctl()
   │
[kernel uORB upper-half]  nuttx/drivers/sensors/sensor.c
   │ push_event() → ring buffer (nbuffer=16) → poll() wake
   │
[kernel lower-half]  boards/spike-prime-hub/src/legosensor_uorb.c
   │   class topic 6 個 (静的) ─┬─ classifier(type_id, port)
   │   port binding 6 個       ─┘   ↓
   │                                 g_legosensor_class_state[class].bound_port
   │ ↑ on_sync / on_data / on_error per port (LUMP kthread context)
   │
[kernel] LUMP engine (#43)  stm32_legoport_lump.c
   │
[kernel] DCM (#42)
```

### 2.1 register 方針

- **boot 時 static register**: 6 class topic 分 `sensor_custom_register("/dev/uorb/sensor_<class>", sizeof(struct lump_sample_s))` を一斉に実行
- 接続なし port は何も publish しない (subscriber は `poll()` で待つ)
- `lump_attach` は per port (6 ports)、登録順序は (1) 6 class topic → (2) 6 port `lump_attach`。`lump_attach` は engine が SYNCED なら同期で `on_sync` を呼ぶため push 先の lower-half が先に揃っている必要がある

### 2.2 1 topic = 1 port ルール

各 class topic は **同時に最大 1 つのポート** を bind する。複数 port が同 class に分類された場合は **port 番号が最も小さいもの** が勝ち、残りは frame drop。

- `on_sync(port, info)` が classifier(type_id, port) で class X を返す
- class-global の `g_bind_lock` 下で:
  - X が未 bind → その port を bind
  - X が他 port (番号 > 自分) を bind 済み → **bind 交代** (新 port 勝ち、旧 port は frame drop に降格 + disconnect sentinel push)
  - X が他 port (番号 < 自分) を bind 済み → 自分が frame drop
- `on_data(port, mode, ...)`: bind 済み port のみ class lower-half に push、未 bind なら drop
- `on_error(port)`: bind 中なら disconnect sentinel push + bind 解除 (該当 class が空く)

> **自動 rebind なし**: class が空いた直後に「未 bind の同種 port を自動 rebind」処理は入れない。port 抜去/再挿入で `on_sync` が再発火するまでその port は bound されない (簡素化)。

### 2.3 callback コンテキスト・lock 順序

- `on_sync` / `on_data` / `on_error` は LUMP **per-port kthread context** から呼ばれる
- `g_bind_lock` (class-global) と per-port lock は **同時保持しない**。一方を取って読んで離してから他方を取る
- `push_event()` は両方の lock を release してから呼ぶ (upper-half の `nxrmutex_t` との循環回避)

### 2.4 複数同種 sensor を扱いたい場合

本リファクタは「同 class topic に複数 port を集約する」ユースケースを **意図的にサポートしない**。複数の同種 sensor を扱う場合:

- **LUMP 直通**: `/dev/legoport[N]` chardev (Issue #43) の `LEGOPORT_LUMP_*` ioctl で port 単位操作
- subscriber 側で 2 つの class topic を pair 化 (例: drivebase は `sensor_motor_l` / `sensor_motor_r` を別々に read。Issue #77)

## 3. Sample envelope

```c
struct lump_sample_s
{
  uint64_t timestamp;       /* μs (sensor framework 慣習) */
  uint32_t seq;             /* per-port monotonic counter */
  uint32_t generation;      /* SYNC / SELECT 確定 / disconnect で +1 */
  uint8_t  port;            /* 0..5 (現 bound port、A..F) */
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
| disconnect sentinel | `type_id == 0 && len == 0` | port 抜去 (`on_error`) もしくは bind 交代で旧 port が外れた場合 |

### 3.2 generation の意味論

| イベント | generation |
|---|---|
| 初回 SYNC 完了 | +1 (sync sentinel として publish) |
| `LEGOSENSOR_SELECT` 成功確定 | +1 (engine が新 mode を観測した次回 `on_data` で bump) |
| SELECT 失敗 (timeout 500 ms) | bump せず (subscriber は valid sample を捨てない) |
| disconnect / bind 交代 | +1 (disconnect sentinel として publish) |

Subscriber は SELECT 直前の generation を記録しておけば、generation 跳ね + `mode_id` 変化のタイミングで新 mode samples を識別可能。`port` フィールドが変わった場合は bind 交代を検知できる。

## 4. Custom ioctl

| cmd | arg | 用途 | claim |
|---|---|---|---|
| `LEGOSENSOR_GET_INFO` | `legosensor_info_arg_s *` | bound port + per-mode schema 一覧 | -- |
| `LEGOSENSOR_GET_STATUS` | `lump_status_full_s *` | engine 状態 (state/baud/RX/TX 等) | -- |
| `LEGOSENSOR_CLAIM` | (なし) | 制御権を取得 (filep を owner に) | -- |
| `LEGOSENSOR_RELEASE` | (なし) | 制御権解放 | owner only |
| `LEGOSENSOR_SELECT` | `legosensor_select_arg_s *` | mode 切替 (`lump_select_mode`) | required |
| `LEGOSENSOR_SEND` | `legosensor_send_arg_s *` | writable mode TX (`lump_send_data`) | required |
| `LEGOSENSOR_SET_PWM` | `legosensor_pwm_arg_s *` | LED / motor PWM 制御 (class 別ルーティング) | required |

class-specific ioctl 番地レンジ (将来予約、本 plan では `-ENOTTY`):

| Class | レンジ |
|---|---|
| COLOR | `_SNIOC(0x0100..0x010f)` |
| ULTRASONIC | `_SNIOC(0x0110..0x011f)` |
| FORCE | `_SNIOC(0x0120..0x012f)` |
| MOTOR_M | `_SNIOC(0x0130..0x013f)` |
| MOTOR_R | `_SNIOC(0x0140..0x014f)` |
| MOTOR_L | `_SNIOC(0x0150..0x015f)` |

### 4.1 errno 仕様

| ioctl | エラー | 条件 |
|---|---|---|
| CLAIM | `-ENODEV` | topic に bind 中の port なし |
|  | `-EBUSY` | 別 fd が claim 中 (同 fd の再 CLAIM は冪等 OK) |
| RELEASE | `-EACCES` | claim 未保持 / claim が stale |
| SELECT / SEND | `-EACCES` | 未 claim |
|  | `-ENODEV` | claim 中に port 抜去 / bind 交代 (再 CLAIM 要) |
| SET_PWM | `-EACCES` | 未 claim |
|  | `-ENODEV` | 同上 |
|  | `-ENOTSUP` | force / motor_* (本リリース段階) / firmware に LIGHT mode なし / LIGHT mode の shape (3×INT8 for color、4×INT8 for ultrasonic) と一致しない |
|  | `-EINVAL` | num_channels が class 期待値と不一致、channels[i] が範囲外 (LED 0..10000、motor -10000..10000、LED に負値は EINVAL) |
| GET_INFO / GET_STATUS | `-ENODEV` | bind 中 port なし |

### 4.2 CLAIM / RELEASE 排他モデル

- read (publish 受信) と GET_INFO/GET_STATUS は誰でも可、SELECT/SEND/SET_PWM だけが claim 必須 (multi-subscriber + 単一 control owner)
- 既に他 fd が CLAIM 中 → `-EBUSY`
- 同 filep からの再 CLAIM は idempotent (no-op、OK)
- write 系を CLAIM していない fd から発行 → `-EACCES`
- **stale claim**: CLAIM 取得後に bind 交代 / port 抜去が発生した場合、claim は無効化 (内部の `bind_generation` を bump)。次回 write 系 ioctl で `-ENODEV` を返す → subscriber は再 CLAIM が必要
- **auto-release**: per-fd `close()` で `sensor_ops_s::close(lower, filep)` が呼ばれ、driver が `claim_owner == filep` なら NULL クリア。明示 RELEASE 忘れも安全

### 4.3 SELECT の API contract

`LEGOSENSOR_SELECT` ioctl は LUMP engine への "送信成功" を返すだけで、**実際の mode 確定は subscriber が `seq` / `generation` / `mode_id` を観測して判定する**。理由:

- `lump_select_mode` は engine の TX queue に積むだけ
- デバイスが mode を切替えるのに ~10 ms かかる
- 切替確認は次回 `on_data` で `mode_id == requested_mode` を見るしかない

driver 側は per-port mutex 下で `pending_select_mode` + `pending_select_deadline = now + 500 ms` を保持し、`on_data` で `frame->mode == pending` のとき generation++ + pending クリア。期限切れの場合は silently drop。

### 4.4 SET_PWM (LED / motor PWM)

| Class | 実装 | バックエンド | values 意味 |
|---|---|---|---|
| COLOR | ✅ | LUMP writable mode "LIGHT" (mode index は info_cache から `name == "LIGHT"` を検索してキャッシュ) | channels[0..2] = LED 0..2 brightness 0..10000 (.01 % 単位) |
| ULTRASONIC | ✅ | 同上 | channels[0..3] = eye LED 0..3 brightness 0..10000 |
| FORCE | `-ENOTSUP` (恒久) | — | アクチュエータなし |
| MOTOR_M / R / L | `-ENOTSUP` (本リリース段階) | (LUMP ではなく STM32 TIM PWM、Issue #80 で実装) | channels[0] = signed duty -10000..10000 |

LIGHT mode 解決と payload 変換:

- `on_sync` 時に `info_cache.modes[i].name == "LIGHT"` を走査して `light_mode_idx` にキャッシュ。同時に該当 mode の shape を検証 (color = INT8 × 3、ultrasonic = INT8 × 4)。一致しない場合は `light_mode_idx = -1`、SET_PWM は `-ENOTSUP`
- **payload 変換**: `channels[i]` (0..10000) を percent (0..100, INT8) に量子化 — `(channels[i] * 100 + 5000) / 10000` (中点四捨五入)
- `lump_send_data(port, light_mode_idx, payload, num_channels)` で送信 (SELECT は呼ばない)
- LED 制御は **active SELECT mode から独立** (LUMP 仕様: writable mode への SEND は active SELECT に影響しない) → 測定継続中も LED 制御可能

> **物理 LED 点灯は H-bridge supply pin に依存 (Issue #80 で実装)**
>
> SPIKE Color Sensor (`NEEDS_SUPPLY_PIN1`) と Ultrasonic Sensor (`NEEDS_SUPPLY_PIN1`) は SYNC 完了後に H-bridge を `-MAX_DUTY` で駆動して LED に電源を供給する必要がある (pybricks 参考実装: `pbio/drv/legodev/legodev_pup_uart.c:894-900`、capability map: `legodev_spec.c:201-208`)。本リリース (#79) では H-bridge driver が未実装のため、`LEGOSENSOR_SET_PWM` は LUMP LIGHT frame を wire 上に送出する (`tx_bytes` で確認可能) が **物理 LED は電源供給がないため点灯しない**。Issue #80 の H-bridge driver が landed すると、SYNC 後に該当 port の supply pin が自動駆動され、本 ioctl の brightness 設定が物理的にも反映される想定。

### 4.5 kernel ↔ userspace の責務分離

| 層 | 責務 | 例 |
|---|---|---|
| **kernel driver (本実装)** | mechanism のみ。SELECT / SEND / SET_PWM をそのまま装置に渡す。**mode に応じた LED 自動制御等の policy は持たない** | SELECT(0) を呼んだら LUMP CMD SELECT を発行するだけ、LED 状態には触れない |
| **userspace ヘルパライブラリ (#78、別 Issue)** | policy を持つ。例: COLOR/REFLT mode で LED ON、AMBI mode で LED OFF を `legolib_color_set_mode(fd, mode)` の中で SELECT + SET_PWM を atomic に発行 | mode → LED 対応表は #78 内のテーブルで管理、firmware 仕様変更時もここ 1 箇所修正 |

NuttX 流の "mechanism in kernel, policy in userspace" を踏襲。subscriber が直接 SELECT / SET_PWM を叩くのも引き続き OK (ライブラリは convenience で、enforce ではない)。

## 5. CLI (`legosensor`)

```
legosensor                                 全 class topic の状態一覧
legosensor list                            同上
legosensor <class>                         1 class topic の status
legosensor <class> info                    bound port + per-mode schema
legosensor <class> status                  engine + traffic counters
legosensor <class> watch [ms]              poll → read decoded samples (default 1000)
legosensor <class> select <mode>           open → CLAIM → SELECT → close (auto-RELEASE)
legosensor <class> send <mode> <hex>...    open → CLAIM → SEND → close
legosensor <class> pwm <ch0> [ch1 ch2 ch3] open → CLAIM → SET_PWM → close
```

`<class>` は `color | ultrasonic | force | motor_m | motor_r | motor_l`。

各書き込みコマンドは **1 プロセスで完結**: open → CLAIM → 操作 → close で auto-RELEASE。プロセス境界をまたいで claim を持続する必要がある場合は別途 daemon 化 (Issue #77 の drivebase を参照)。

### 5.1 例

```sh
# Color sensor を port A に挿す
legosensor color info                    # bound port=A, modes 一覧
legosensor color watch                   # 1 秒間 sample を decode 表示
legosensor color select 1                # REFLT mode へ (mode 1)
legosensor color pwm 5000 0 0            # LED0 を 50% 点灯
legosensor color pwm 0 0 0               # 全 LED off

# Ultrasonic を別 port に挿す
legosensor ultrasonic pwm 5000 5000 0 0  # eye LED 上 2 個点灯

# Force / motor の SET_PWM は本リリースでは未対応
legosensor force pwm 0                   # -ENOTSUP
legosensor motor_m pwm 0                 # -ENOTSUP (#80 で実装)
```

### 5.2 `dd` / `sensortest` が使えない理由

- **`dd`**: poll(2) を使わず単一 `read()` を発行する。upper-half の non-fetch read path は新 sample がない時に `-ENODATA` を即返すため、push_event との race で `Unknown error 61` (ENODATA) を返す
- **`sensortest`**: `g_sensor_info[]` table の標準 sensor type 名 (accel0, gyro0 等) しか受け付けない。`sensor_<class>` は custom path なので reject

`legosensor watch` が正しい end-to-end 検証ツール。

## 6. デバイス対応

### 6.1 検証済 (HW で SYNCED 確認)

| Type | 名称 | num_modes | デフォルト mode | 対応 class topic |
|---|---|---|---|---|
| 48 | SPIKE Medium Motor | 6 | POWER (mode 0) | `sensor_motor_r` (port A/C/E) / `sensor_motor_l` (port B/D/F) |
| 49 | SPIKE Large Motor | 6 | POWER (mode 0) | `sensor_motor_m` |
| 61 | SPIKE Color Sensor | 8 | COLOR (mode 0) | `sensor_color` |
| 62 | SPIKE Ultrasonic Sensor | 8 | DISTL (mode 0) | `sensor_ultrasonic` |
| 63 | SPIKE Force Sensor | 7 | FORCE (mode 0) | `sensor_force` |

### 6.2 未マップ type_id

38 / 46 / 47 / 65 / 75 / 76 等の Powered Up デバイスは本リリースの classifier では **frame drop**。class topic への push なし。LUMP 直通 (`/dev/legoport[N]` chardev) で port 単位の操作は可能。

### 6.3 Force / Motor デバイスの DATA 開始制約 (LUMP engine 側、別 Issue 化予定)

Force Sensor (type 63) と SPIKE Medium / Large Motor (type 48 / 49) は **LUMP SYNC 後にホスト側から SELECT を発行しないと DATA frame を吐かない**。本実装の LUMP engine (`stm32_legoport_lump.c`) は SYNC 完了直後の auto-SELECT を行わないため、これらのデバイスは挿しても 600 ms (no-DATA timeout) で session が切れ、`legosensor force info` 等は `-ENODEV` を返す。

dmesg で観測される pattern:

```
lump: port A: type_id=63
lump: port A: SYNCED type=63 modes=7 baud=115200
lump: port A: no DATA for 600 ms, disconnecting
lump: port A session ended ret=-110 step=N sleep=...ms
```

Color (type 61) / Ultrasonic (type 62) は default mode で auto-stream するため SELECT 不要。pybricks 参考実装 (`pbio/drv/legodev/legodev_pup_uart.c:903`) では SYNC 完了後に必ず `pbdrv_legodev_request_mode(default_mode)` を発行している。

**本リリース (#79) の対処**: Force / Motor は実機検証から除外し、上記制約を明文化するに留める。auto-SELECT-after-SYNC の実装は **#43 LUMP engine への follow-up Issue として別途切り出す** (本 plan のスコープ外)。

## 7. 後続 Issue

| Issue | 範囲 |
|---|---|
| #80 | モーター PWM H-bridge ドライバ (STM32 TIM 使用)。MOTOR_M / R / L の SET_PWM が `-ENOTSUP` から実装に切り替わる |
| #78 | userspace ヘルパライブラリ `apps/legolib/` (per-class policy: mode → LED 自動制御 等) |
| #77 | drivebase userspace daemon (`apps/drivebase/`、左右輪 motor をペアで制御。#78 + #80 に依存) |

## 8. tuning 定数

| 定数 | 値 | 説明 |
|---|---|---|
| `LEGOSENSOR_NUM_PORTS` | 6 | 物理ポート数 (A..F) |
| `LEGOSENSOR_CLASS_NUM` | 6 | class topic 数 |
| `LEGOSENSOR_MAX_DATA_BYTES` | 32 | LUMP frame payload 上限 |
| `LEGOSENSOR_NBUFFER` | 16 | upper-half circbuf depth (per class topic) |
| `LEGOSENSOR_PENDING_TIMEOUT_MS` | 500 | SELECT 確定待ち上限 |

## 9. 参照

- 設計詳細: [lump-protocol.md](lump-protocol.md), [port-detection.md](port-detection.md)
- 公開 ABI: `boards/spike-prime-hub/include/board_legosensor.h`
- driver 実装: `boards/spike-prime-hub/src/legosensor_uorb.c`
- CLI: `apps/legosensor/legosensor_main.c`
- LUMP API: `boards/spike-prime-hub/include/board_lump.h`
- NuttX sensor framework: `nuttx/include/nuttx/sensors/sensor.h`, `nuttx/drivers/sensors/sensor.c`
