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
[user]   apps/sensor/sensor_main.c (CLI)
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

| Class | 実装 | 代替手段 / バックエンド | values 意味 |
|---|---|---|---|
| COLOR | `-ENOTSUP` (Issue #92) | `LEGOSENSOR_SEND mode=3` (LIGHT、3×INT8 PCT) を直接使う | — |
| ULTRASONIC | `-ENOTSUP` (Issue #92) | `LEGOSENSOR_SEND mode=5` (LIGHT、4×INT8 PCT) を直接使う | — |
| FORCE | `-ENOTSUP` (恒久) | — | アクチュエータなし |
| MOTOR_M / R / L | ✅ | `stm32_legoport_pwm_set_duty()` (STM32 TIM PWM、Issue #80) を直叩き | channels[0] = signed duty -10000..10000 (.01 % 単位) |

**SET_PWM の意味付け**: H-bridge ベースの物理 PWM 専用 (motor 系)。LIGHT 系 (COLOR / ULTRASONIC) は LUMP writable mode への WRITE であって PWM ではないので SET_PWM 経路から外し、SEND を直に叩く形に統一した (Issue #92)。LIGHT mode の brightness は INT8 PCT (0..100) なので、`LEGOSENSOR_SEND` の `data[0..N]` にそれぞれ 0..100 を入れる。

LIGHT 書込み (`LEGOSENSOR_SEND` 経由、Issue #92):

- `legosensor_send_arg_s` に `mode = 3` (COLOR) または `mode = 5` (ULTRASONIC) と、LED 1 個あたり INT8 PCT (0..100) を 1 byte ずつ `data[0..N-1]` に積んで `LEGOSENSOR_SEND` ioctl を発行する。kernel 側でのスケーリングは無し
- engine entry は `lump_send_data(port, mode, data, len)`。`current_mode != mode` のときに `CMD SELECT mode` を **同じドレインパスで先送り** してから DATA を流す (Issue #92)。これにより `current_mode` も LIGHT 側に揃う — 古い実装は SELECT を呼ばず "writable mode SEND は active SELECT 独立" を利用していたが、SPIKE Color Sensor の firmware 自動 LED 制御の都合で SELECT 経由にした方が pybricks 流儀と合致する
- (旧仕様メモ) LUMP プロトコル仕様としては writable mode への SEND は active SELECT mode から独立しており、SELECT を打たずに LED brightness を書くことも理論上は可能。ただし Color / Ultrasonic の firmware は SELECT 通知に対して自動 LED 制御を被せてくるため、SELECT を経由しないと LIGHT 値が即上書きされて意味を成さない

> **物理 LED 点灯は H-bridge supply pin が前提 (Issue #80 で実装済み)**
>
> SPIKE Color Sensor (`NEEDS_SUPPLY_PIN1`) と Ultrasonic Sensor (`NEEDS_SUPPLY_PIN1`) は SYNC 完了後に H-bridge を `-MAX_DUTY` で駆動して LED に電源を供給する必要がある (pybricks 参考実装: `pbio/drv/legodev/legodev_pup_uart.c:894-900`、capability map: `legodev_spec.c:201-208`)。Issue #80 で `lump_pin_supply_on_sync()` 経由で auto-supply されるようになり、SYNCED 直後には既に H-bridge が supply rail に pin 済み。状態は `port pwm <P> status` (PINNED フラグ) で確認可能。

### 4.5 SPIKE Color Sensor の自動 LED 制御 (実機観測)

> LEGO 公開仕様には記述なし。pybricks の暗黙の前提として実装されており、本プロジェクトでも **2026-05-01 に実機 (Hub firmware = main, sensor 45605) で再現確認済**。

SPIKE Color Sensor (type_id=61) は SELECT を受け取るたびに firmware 側で **mode ごとに LED 3 灯を自律的に ON/OFF** する。Hub 側のドライバ (`stm32_legoport_lump.c` / `legosensor_uorb.c`) には mode → LED の対応表は **存在せず**、SELECT を CMD として転送しているだけ。

| mode | 名称 | 観測される LED 既定動作 | 物理的意味 |
|---|---|---|---|
| 0 | COLOR | ON | 反射光から色を当てる → LED 必須 |
| 1 | REFLT | ON | 反射率測定 |
| 2 | AMBI  | OFF | 環境光測定。LED が点くと測れない |
| 3 | LIGHT | OFF (writable で上書き可) | LED 制御 mode。ユーザの SET_PWM/WRITE で点ける |
| 4 | RREFL | ON | raw 反射 |
| 5 | RGB I | ON | RGB+I (反射光のスペクトル分解) |
| 6 | HSV   | ON | LED ON 状態の HSV (pybricks の `surface=True` 経路) |
| 7 | SHSV  | OFF | 周囲光下の HSV (pybricks の `surface=False`、ambient 相当) |

**LIGHT mode (3) で書き込んだ brightness は mode 3 に滞在中だけ有効** (実機検証で確定した仮説 A):

- mode 3 SELECT 中に `LEGOSENSOR_SET_PWM` / `LUMP_SEND` で書いた値 → LED に即反映
- そこから `port lump <P> select 5` 等で他モードへ抜けると → firmware が「反射計測用デフォ輝度」で LED を上書きする (ユーザ値は捨てられる)
- 戻って `select 3` しても、再度 WRITE しない限り LED は OFF (mode 3 既定)

つまり SET_PWM は「カラーセンサーを純粋な LED アレイとして使う」用途では使えるが、「RGB_I 測定中の LED 輝度を絞る」用途には使えない (ファームに上書きされる)。

### 4.6 kernel ↔ userspace の責務分離

| 層 | 責務 | 例 |
|---|---|---|
| **sensor firmware (LEGO 製)** | per-mode の LED ON/OFF は firmware が自律的に持つ。Hub 側からは触れない | mode 5 SELECT で LED 自動 ON、mode 7 SELECT で自動 OFF (4.5 表) |
| **kernel driver (本実装)** | mechanism のみ。SELECT / SEND / SET_PWM を CMD として装置に渡す。**mode → LED policy は持たない** | SELECT(0) は LUMP CMD SELECT を 1 発出すだけで LED 状態には触れない (実際の ON/OFF は firmware 側で起きる) |
| **userspace ヘルパライブラリ (#78、別 Issue)** | firmware の既定 policy を**上書き**したい場合の被せ層。例: 「mode 0 でも LED を消したい」「mode 3 で常時 50% で点けっぱなしにする」等を `legolib_color_*` 系 API でカプセル化 | firmware が既に "正しい" LED 制御をしているので、#78 はあくまで override 用途。デフォで十分なら #78 を経由しなくて良い |

NuttX 流の "mechanism in kernel, policy in userspace" を踏襲。subscriber が直接 SELECT / SET_PWM を叩くのも引き続き OK (ライブラリは convenience で、enforce ではない)。

### 4.7 SPIKE Ultrasonic Sensor modes (45604)

LEGO 公式 spec PDF (`techspecs_techniccolorsensor.pdf` の姉妹版) では 100 Hz の sample rate を謳っているが、LUMP info にこのレートは encode されておらず、firmware の実出力を観測すると mode によってバラつく (動きあり ~110 fps、静止 ~67 fps、Issue #92 で計測)。

| mode | 名称 | サイズ | pybricks API | 用途 |
|---|---|---|---|---|
| 0 | DISTL | 1×INT16 | `distance` | 距離 (cm)、long range |
| 1 | DISTS | 1×INT16 | — | 距離 short range (pybricks 未使用) |
| 2 | SINGL | 1×INT16 | — | single-ping 距離 (pybricks 未使用) |
| 3 | LISTN | 1×INT8 | `presence` | 他超音波信号の検知 (0/1) |
| 4 | TRAW | 1×INT32 | — | raw time-of-flight (pybricks 未使用) |
| 5 | LIGHT | 4×INT8 (writable) | `lights` | eye-LED 4 灯の輝度 (0..100 %) |
| 6 | PING | 1×INT8 (writable, raw 0..1) | — | pybricks は "??" (`legodev.h:333`)、用途不明だが LUMP info で **writable + raw 0..1 + unit PCT** を実機確認 (`sensor ultrasonic info` 2026-05-02)。ほぼ間違いなく "ping を 1 発撃つトリガ" (write 1=fire / 0=idle)。pybricks 未実装で副作用も未検証なので勝手に呼ばない方針 |
| 7 | ADRAW | 1×INT16 | — | raw ADC 値 (pybricks 未使用) |
| 8 | CALIB | 7×INT16 | — | キャリブレーション (pybricks 未使用) |

ポート給電: COLOR と同じく `NEEDS_SUPPLY_PIN1`、SYNC 後に H-bridge を `-MAX_DUTY` に振って LED 系を通電する必要あり (Issue #80)。LIGHT mode の brightness は COLOR と同じく **mode 5 滞在中に書いた値だけ有効** と仮定 (実機検証は未だ — COLOR の §4.5 仮説 A と同パターンが想定される)。

### 4.8 SPIKE Motor modes (Medium=48, Large=49) と LUMP-write 無効性 (実機観測)

SPIKE Medium (`motor_r` / `motor_l`、type 48 = relative encoder) と Large (`motor_m`、type 49 = absolute encoder) の per-mode 仕様 (pybricks `legodev.h:359` と LUMP info から):

| mode | 名称 | サイズ | データ型 | 公開 API | 実機での意味 |
|---|---|---|---|---|---|
| 0 | POWER | 1×INT8 (writable, -100..100) | LUMP-internal commanded power | (pybricks 未使用) | **常に 0** (Hub が H-bridge 直叩きで動かしているため firmware の commanded power は 0 のまま) |
| 1 | SPEED | 1×INT8 (writable, -100..100) | encoder 由来 | (pybricks 未使用 publish) | encoder 実測速度。~1000 fps publish、driving 状況がそのまま反映 (例: pwm 3000 → SPEED 21、すべり込み) |
| 2 | POS | 1×INT32 (writable, deg) | encoder 由来 | (pybricks 未使用 publish) | 相対累積角度。~1000 fps、回転を単調にカウント |
| 3 | APOS | 1×INT16 (writable、Large のみ -180..179) | absolute encoder | (pybricks 未使用 publish) | 絶対角度 |
| 4 | CALIB | 2×INT16 | — | (pybricks 未使用) | キャリブレーション |
| 5 | STATS | 14×INT16 | — | (pybricks 未使用) | 内部統計 |

**LUMP-write 経路 (`sensor motor_* send 0/1/2/3 ...`) は完全に no-op (実機検証 2026-05-02)**:

- `pwm 0` (BRAKE) 状態で `sensor motor_r send 0 50` (POWER に 80 を LUMP-write) → 1 秒待っても **`port pwm A status` は BRAKE/duty=0 のまま**、motor も動かない
- `send 0 50` 後に POWER (mode 0) を read しても **0 のまま** (echo もされない)
- LUMP wire frame は出ている (motor MCU は受信しているはず) が、**firmware が完全に無視**

つまり SPIKE Hub 上での motor 駆動は **H-bridge 1 経路のみ** (`sensor motor_* pwm <duty>` / `port pwm <P> set <duty>`) が実用解。pybricks が LUMP-write を一切使わず H-bridge 直叩きに統一しているのは、この実機制約に対する正しい設計判断と確認できる。

副作用: mode 0 POWER は telemetry としても無価値 (常に 0)。**速度を読みたいなら mode 1 SPEED**、**角度を読みたいなら mode 2 POS / 3 APOS** を select する。

## 5. CLI (`sensor`)

```
sensor                                 全 class topic の状態一覧
sensor list                            同上
sensor <class>                         1 class topic の status
sensor <class> info                    bound port + per-mode schema
sensor <class> status                  engine + traffic counters
sensor <class> watch [ms]              poll → read decoded samples (default 1000)
sensor <class> fps [ms]                rate-only count, no per-sample print
sensor <class> select <mode>           open → CLAIM → SELECT → close (auto-RELEASE)
sensor <class> send <mode> <hex>...    open → CLAIM → SEND → close
sensor <class> pwm <ch0> [ch1 ch2 ch3] open → CLAIM → SET_PWM → close
```

`<class>` は `color | ultrasonic | force | motor_m | motor_r | motor_l`。

各書き込みコマンドは **1 プロセスで完結**: open → CLAIM → 操作 → close で auto-RELEASE。プロセス境界をまたいで claim を持続する必要がある場合は別途 daemon 化 (Issue #77 の drivebase を参照)。

### 5.1 例

```sh
# Color sensor を port A に挿す
sensor color info                    # bound port=A, modes 一覧
sensor color watch                   # 1 秒間 sample を decode 表示
sensor color select 1                # REFLT mode へ (mode 1)
sensor color send 3 32 00 00         # LIGHT (mode 3) — LED0 を 50% 点灯
sensor color send 3 00 00 00         # LIGHT — 全 LED off

# Ultrasonic を別 port に挿す
sensor ultrasonic send 5 32 32 00 00 # LIGHT (mode 5) — eye LED 上 2 個 50%

# Motor — H-bridge PWM (Issue #80 backend)
sensor motor_m pwm 5000              # 前進 50%
sensor motor_m pwm -5000             # 後進 50%
sensor motor_m pwm 0                 # ブレーキ (pybricks 互換、§4.4 参照)

# Color / Ultrasonic / Force の PWM は不可 (SEND を使うか -ENOTSUP)
sensor color pwm 0 0 0               # -ENOTSUP (SEND mode 3 を使う)
sensor ultrasonic pwm 0 0 0 0        # -ENOTSUP (SEND mode 5 を使う)
sensor force pwm 0                   # -ENOTSUP (アクチュエータなし)
```

### 5.2 `dd` / `sensortest` が使えない理由

- **`dd`**: poll(2) を使わず単一 `read()` を発行する。upper-half の non-fetch read path は新 sample がない時に `-ENODATA` を即返すため、push_event との race で `Unknown error 61` (ENODATA) を返す
- **`sensortest`**: `g_sensor_info[]` table の標準 sensor type 名 (accel0, gyro0 等) しか受け付けない。`sensor_<class>` は custom path なので reject

`sensor watch` が正しい end-to-end 検証ツール。

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

## 7. 後続 Issue

| Issue | 範囲 |
|---|---|
| #80 | (CLOSED 2026-05-01) モーター PWM H-bridge ドライバ + LUMP supply auto-pin。MOTOR_M / R / L の SET_PWM は #92 でこれを直叩きする形に接続済み |
| #78 | userspace ヘルパライブラリ `apps/legolib/` (per-class policy: mode → LED 自動制御 等) |
| #77 | drivebase userspace daemon (`apps/drivebase/`、左右輪 motor をペアで制御。#78 に依存) |

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
- CLI: `apps/sensor/sensor_main.c`
- LUMP API: `boards/spike-prime-hub/include/board_lump.h`
- NuttX sensor framework: `nuttx/include/nuttx/sensors/sensor.h`, `nuttx/drivers/sensors/sensor.c`
