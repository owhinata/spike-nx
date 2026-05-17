# drivebase デーモン (Issue #77)

SPIKE Prime Hub の 2 モーター駆動ベース (drivebase) を、5 ms 周期の閉ループ制御として userspace daemon で動かす実装。pybricks の drivebase 機能 (drive_straight / turn / forever / stop / on_completion 等) を **機能パリティ**で移植している (pbio の C ソースは持ち込まず、SPIKE 専用 / NuttX-native に書き直し)。

## 1. 構成

### 1.1 レイヤ

```
ユーザー app             kernel /dev/drivebase chardev          userspace daemon
─────────                ─────────────────────────────          ────────────────
ioctl(fd, DRIVEBASE_*)   cmd_ring (MPSC, 8 段)        ────►    DAEMON_PICKUP_CMD
                         state_db (double-buffer)     ◄────    DAEMON_PUBLISH_STATE
                         status (seqlock)             ◄────    DAEMON_PUBLISH_STATUS
                         emergency_stop_cb (kernel)
                                                              RT pthread (SCHED_FIFO 220, 5 ms)
                                                              motor encoder drain (LEGOSENSOR)
                                                              IMU drain (LSM6DSL uORB)
                                                              observer + trajectory + PID
                                                              motor SET_PWM / COAST / BRAKE
```

ユーザ向け ioctl と daemon 内部 ioctl は別ヘッダに分離している (FUSE 移植容易性、§9 参照):

| ヘッダ | 公開度 | 内容 |
|---|---|---|
| `boards/spike-prime-hub/include/board_drivebase.h` | 公開 | DRIVEBASE_DRIVE_STRAIGHT / TURN / FOREVER / STOP / GET_STATE / GET_STATUS / JITTER_DUMP 等のユーザー向け ABI |
| `boards/spike-prime-hub/include/board_drivebase_internal.h` | 内部 | DRIVEBASE_DAEMON_ATTACH / DETACH / PICKUP_CMD / PUBLISH_STATE / PUBLISH_STATUS — daemon ↔ kernel chardev のみ使用 |

### 1.2 chardev 登録

`/dev/drivebase` は `boards/spike-prime-hub/src/stm32_drivebase_chardev.c` で kernel-side に登録される。daemon が居なくてもデバイスファイルは存在し、`open()` は成功する (走行系 ioctl は -ENOTCONN を返す)。

NuttX BUILD_PROTECTED では `register_driver()` が userspace から呼べないため、必然的に kernel-side 登録となる。userspace daemon は通常 ioctl (DAEMON_ATTACH 等) で kernel chardev に「attach」して使用する。

## 2. ファイル構成

```
boards/spike-prime-hub/
├── include/
│   ├── board_drivebase.h            user-facing ABI
│   └── board_drivebase_internal.h   daemon-internal ABI
└── src/
    └── stm32_drivebase_chardev.c    kernel chardev shim (cmd_ring / state_db / watchdog)

apps/drivebase/
├── Kconfig                          APP_DRIVEBASE + RT priority / stack / gain default
├── Makefile                         CLI builtin (PROGNAME=drivebase, STACKSIZE=4096)
├── drivebase_main.c                 NSH CLI: start/stop/status/config/straight/...
├── drivebase_daemon.{c,h}           lifecycle FSM + idle loop + stall watchdog
├── drivebase_chardev_handler.{c,h}  DAEMON_ATTACH/PICKUP/PUBLISH wrapper
├── drivebase_rt.{c,h}               SCHED_FIFO 220 + clock_nanosleep 5 ms tick + jitter ring
├── drivebase_motor.{c,h}            sensor_motor_l/r encoder drain + SET_PWM/COAST/BRAKE
├── drivebase_imu.{c,h}              sensor_imu0 drain + Z-gyro heading 1D
├── drivebase_drivebase.{c,h}        L/R 集約 (drive_straight / turn / curve / arc / forever)
├── drivebase_servo.{c,h}            per-motor 閉ループ
├── drivebase_observer.{c,h}         sliding-window slope (default 30 ms)
├── drivebase_control.{c,h}          PID + anti-windup + on_completion 種別処理
├── drivebase_trajectory.{c,h}       台形プロファイル (accel / cruise / decel + triangular fallback)
├── drivebase_settings.{c,h}         gain table + drive_speed default + completion thresholds
├── drivebase_angle.h                int64 mdeg + deg ↔ mm 変換 (π = 355/113)
└── drivebase_internal.h             enum db_side_e 等の共通型
```

## 3. ABI

### 3.1 ioctl 番号

`board_drivebase.h` で定義 (group `_DBASEBASE = 0x4900`)。

ユーザー向け:

```c
DRIVEBASE_CONFIG               _DBASEIOC(0x01)  /* drivebase_config_s */
DRIVEBASE_RESET                _DBASEIOC(0x02)
DRIVEBASE_DRIVE_STRAIGHT       _DBASEIOC(0x10)
DRIVEBASE_DRIVE_CURVE          _DBASEIOC(0x11)
DRIVEBASE_DRIVE_ARC_ANGLE      _DBASEIOC(0x12)
DRIVEBASE_DRIVE_ARC_DISTANCE   _DBASEIOC(0x13)
DRIVEBASE_DRIVE_FOREVER        _DBASEIOC(0x14)
DRIVEBASE_TURN                 _DBASEIOC(0x15)
DRIVEBASE_STOP                 _DBASEIOC(0x16)
DRIVEBASE_SPIKE_DRIVE_FOREVER  _DBASEIOC(0x20)
DRIVEBASE_SPIKE_DRIVE_TIME     _DBASEIOC(0x21)
DRIVEBASE_SPIKE_DRIVE_ANGLE    _DBASEIOC(0x22)
DRIVEBASE_GET_DRIVE_SETTINGS   _DBASEIOC(0x30)
DRIVEBASE_SET_DRIVE_SETTINGS   _DBASEIOC(0x31)
DRIVEBASE_SET_USE_GYRO         _DBASEIOC(0x32)
DRIVEBASE_GET_STATE            _DBASEIOC(0x33)
DRIVEBASE_GET_HEADING          _DBASEIOC(0x34)
DRIVEBASE_JITTER_DUMP          _DBASEIOC(0x40)
DRIVEBASE_GET_STATUS           _DBASEIOC(0x41)
```

daemon-internal:

```c
DRIVEBASE_DAEMON_ATTACH         _DBASEIOC(0x80)
DRIVEBASE_DAEMON_DETACH         _DBASEIOC(0x81)
DRIVEBASE_DAEMON_PICKUP_CMD     _DBASEIOC(0x82)
DRIVEBASE_DAEMON_PUBLISH_STATE  _DBASEIOC(0x83)
DRIVEBASE_DAEMON_PUBLISH_STATUS _DBASEIOC(0x84)
DRIVEBASE_DAEMON_PUBLISH_JITTER _DBASEIOC(0x85)
```

### 3.2 主要 struct

全 struct は固定幅型のみで構成され、`_Static_assert(sizeof(...) == N)` でサイズロック済 (Linux FUSE への移植時に 32/64 bit 互換)。

```c
enum drivebase_on_completion_e {
  DRIVEBASE_ON_COMPLETION_COAST       = 0,  /* 完了→自由回転 */
  DRIVEBASE_ON_COMPLETION_BRAKE       = 1,  /* 完了→短絡制動 */
  DRIVEBASE_ON_COMPLETION_HOLD        = 2,  /* 完了→位置 PID で停止保持 */
  DRIVEBASE_ON_COMPLETION_CONTINUE    = 3,  /* 完了→等速継続 (停止しない) */
  DRIVEBASE_ON_COMPLETION_COAST_SMART = 4,  /* SMART: 完了後 ~100 ms hold → coast */
  DRIVEBASE_ON_COMPLETION_BRAKE_SMART = 5,  /* SMART: 完了後 ~100 ms hold → brake */
};

struct drivebase_config_s          { uint32_t wheel_d_um; uint32_t axle_t_um; uint8_t r[8]; };
struct drivebase_drive_straight_s  { int32_t distance_mm; uint8_t on_completion; uint8_t r[7]; };
struct drivebase_turn_s            { int32_t angle_deg; uint8_t on_completion; uint8_t r[3]; };
struct drivebase_drive_forever_s   { int32_t speed_mmps; int32_t turn_rate_dps; };
struct drivebase_stop_s            { uint8_t on_completion; uint8_t r[7]; };

struct drivebase_state_s
{
  int32_t  distance_mm;       /* drivebase 中心の累積前進距離 */
  int32_t  drive_speed_mmps;
  int32_t  angle_mdeg;        /* heading × 1000 (encoder + 任意で gyro 上書き) */
  int32_t  turn_rate_dps;
  uint32_t tick_seq;
  uint8_t  is_done;
  uint8_t  is_stalled;
  uint8_t  active_command;
  uint8_t  reserved;
};

struct drivebase_status_s
{
  uint8_t  configured, motor_l_bound, motor_r_bound, imu_present, use_gyro;
  uint8_t  daemon_attached;
  uint8_t  reserved[2];
  uint32_t tick_count, tick_overrun_count, tick_max_lag_us;
  uint32_t cmd_ring_depth;
  uint32_t cmd_drop_count;
  uint32_t last_cmd_seq;
  uint32_t last_pickup_us;
  uint32_t last_publish_us;
  uint32_t attach_generation;
  uint32_t encoder_drop_count;
};
```

`drivebase_jitter_dump_s` は §6.3 で詳述。

### 3.3 cmd_ring の push policy

複数 user fd が producer になりうるので MPSC。kernel 側は `producer_lock` (nxmutex) で短時間直列化。

| 状況 | policy |
|---|---|
| 通常コマンドで ring 満杯 | `-EBUSY` で即時 return (caller を待たせない、priority inversion 回避) |
| STOP コマンドで ring 末尾が STOP | 上書き (coalesce) |
| STOP コマンドで ring 満杯 | 最古の non-STOP envelope を drop して push (STOP は必ず受け付ける) |

これにより `DRIVEBASE_STOP` は ring full 状態でも必ず受領される。

### 3.4 エラーコード (走行系 ioctl)

| 状況 | 戻り値 |
|---|---|
| daemon 未起動 | `-ENOTCONN` |
| `DRIVEBASE_CONFIG` 未実行で走行系 ioctl | `-ENOTCONN` |
| daemon 動作中に重複 attach | `-EBUSY` |
| モーター片方未接続 (type_id != 48) | `-ENODEV` |
| LEGOSENSOR_CLAIM 競合 | `-EBUSY` |
| stall watchdog 発火後 | `-EIO` |
| reserved 領域 != 0 / range out | `-EINVAL` |

### 3.5 緊急 STOP 経路

`DRIVEBASE_STOP` は ioctl context で完結する fast path:

1. `output_epoch` を atomic increment
2. `default_on_completion` に応じて `stm32_legoport_pwm_coast()` または `stm32_legoport_pwm_brake()` を kernel 内 static 関数として直接呼ぶ
3. STOP envelope を cmd_ring に push し、daemon に trajectory sync を依頼

daemon round-trip 不要、ロック不要 (atomic + lock-free ring) → `< 100 µs latency` を kernel context のみで達成。

emergency_stop は **user 関数ポインタを kernel に登録しない**。daemon segfault による dangling pointer / kernel HardFault を避けるため、daemon は ATTACH 時に「左右モーターの legoport port idx (0..5)」だけを kernel に渡し、kernel は kernel-resident な `stm32_legoport_pwm_*` を port idx で叩く。

## 4. CLI 仕様

PROGNAME = `drivebase` (NSH builtin)。stack size は 4096 (§7 参照)。

| サブコマンド | 動作 |
|---|---|
| `drivebase` (引数なし) | usage を表示 |
| `drivebase status` | DRIVEBASE_GET_STATUS スナップショットを表示 (daemon 未起動でも動作) |
| `drivebase start [wheel_mm [axle_mm [tick_ms]]]` | daemon 起動。`wheel_mm` / `axle_mm` は小数指定可 (例 `17.6 56.5`)。`tick_ms` は RT 制御周期 (Issue #120, 範囲 1–20 ms)。default 56 / 112 / 2。**ボード起動時に rcS から default 引数で自動実行される (Issue #120)** |
| `drivebase stop` | daemon 停止 (グレースフル teardown、~2 秒以内) |
| `drivebase config <wheel_mm> <axle_mm>` | DRIVEBASE_CONFIG。小数 OK (内部 ABI が μm 単位なので sub-mm 精度を保持)。現状は daemon の起動時 default を使うので通常不要 |
| `drivebase reset [distance_mm] [angle_deg]` | DRIVEBASE_RESET — daemon の publish する distance/heading を指定値 (default 0/0) に再 anchor。`drivebase start` 時にも自動的に 0/0 に reset されるので、本 verb は主にセッション途中で再ゼロ化したいとき用 |
| `drivebase straight <mm> [<mmps>] [coast\|brake\|hold]` | DRIVE_STRAIGHT。`mmps` 省略 (or `0`) で `db_settings` の default |
| `drivebase turn <deg> [<dps>] [coast\|brake\|hold]` | TURN (CCW positive)。`dps` 省略で default |
| `drivebase curve <radius_mm> <angle_deg> [coast\|brake\|hold]` | DRIVE_CURVE (Issue [#138](https://github.com/owhinata/spike-nx/issues/138)、半径 `R` の円弧を `angle_deg` 描く)。`R == 0` は `turn` にフォールバック。Phase 4 (C) trajectory_stretch (§8.2.1) で distance と heading が同時完了 |
| `drivebase arc <radius_mm> <distance_mm> [coast\|brake\|hold]` | DRIVE_ARC_DISTANCE (Issue [#138](https://github.com/owhinata/spike-nx/issues/138)、半径 `R` の円弧を距離 `distance_mm` 進む)。`R == 0` は `straight` にフォールバック |
| `drivebase forever <mmps> <dps>` | DRIVE_FOREVER (停止しない、distance + heading 同時) |
| `drivebase stop-motion <coast\|brake\|hold>` | DRIVEBASE_STOP (緊急停止 fast path) |
| `drivebase get-state [duration_ms [interval_ms]]` | DRIVEBASE_GET_STATE。引数なしは 1 行のみ。`duration_ms` 指定時は `interval_ms` 毎 (default 100ms) にサンプリングして 1 行ずつ表形式で出力 |
| `drivebase set-gyro <none\|1d\|3d>` | DRIVEBASE_SET_USE_GYRO (heading 上書き) |
| `drivebase jitter [reset]` | DRIVEBASE_JITTER_DUMP (RT loop の wake latency 統計) |

開発専用の隠し verb: `_motor`, `_alg`, `_servo`, `_drive`, `_rt`, `_daemon`, `_imu`。lifecycle FSM の段階的検証用で、daemon 動作中には使用してはならない (g_daemon と BSS を共有する static 構造体を破壊する)。

## 5. lifecycle FSM

`drivebase_daemon.c` の `daemon_task_main`:

```
DAEMON_STOPPED → drivebase_daemon_start() → DAEMON_INITIALIZING
   1. drivebase_motor_init                 (open + CLAIM /dev/uorb/sensor_motor_l/r、LEGOSENSOR_GET_INFO で type=48 検証)
   2. drivebase_motor_select_mode(L/R, 2)  (LUMP mode 2 = POS = signed int32 deg、~30 ms 待機)
   3. db_drivebase_init + reset            (geometry + servo[L/R] + observer + control)
   4. db_chardev_handler_attach            (open /dev/drivebase + DAEMON_ATTACH)
   5. db_imu_open                          (best-effort、失敗しても encoder-only で継続)
   6. db_rt_init + db_rt_start             (RT pthread spawn、SCHED_FIFO 220、4 KB stack)
DAEMON_INITIALIZING → DAEMON_RUNNING
   idle loop: usleep(50 ms) + DAEMON_PUBLISH_STATUS 1 発
DAEMON_RUNNING → drivebase_daemon_stop() → DAEMON_TEARDOWN
   1. atomic_store(running = false)
   2. db_rt_stop                           (RT pthread を join、最長 1 tick 待機)
   3. drivebase_motor_coast(L/R)           (明示的に両モーター coast)
   4. db_imu_close
   5. db_chardev_handler_detach            (DAEMON_DETACH + close fd → kernel が自動 coast)
   6. drivebase_motor_deinit               (LEGOSENSOR fd close → legoport chardev が auto-coast 二重保険)
DAEMON_TEARDOWN → sem_post(teardown_done) → DAEMON_STOPPED
```

stop は `drivebase_daemon_stop(timeout_ms)` (default 2000 ms) で teardown_done を待つ。

### 5.1 stall watchdog (daemon 側)

RT tick callback (`rt_tick_cb`) が `db_rt_s.deadline_miss_count` を監視:

- 5 連続 deadline miss (lag > 1 ms) で `drivebase_motor_coast(L/R)` を緊急発火
- `atomic_store(running = false)` で daemon を teardown 経路に乗せる

### 5.2 stale daemon watchdog (kernel 側)

`stm32_drivebase_chardev.c` の LPWORK item (25 ms 周期 poll) が `last_publish_ticks` を監視:

- daemon が PUBLISH_STATE を 50 ms 以上止めたら stale と判定
- `db_emergency_actuate(default_on_completion)` で kernel 直叩きで motor 停止
- `db_detach_locked()` で attach state を解除 → 以後の走行系 ioctl は -ENOTCONN

これで daemon が livelock した場合も motor 安全停止を kernel が保証する。

### 5.3 ATTACH fd の close cleanup

kernel chardev の `close()` fop で:

```c
if (dev->attached && dev->attach_filep == filep) {
    /* ATTACH した fd が close された (daemon 終了 or segfault) */
    db_detach_locked(dev, true);   /* emergency_stop = true → motor coast */
}
```

`attach_filep` ポインタ比較で「ATTACH した fd の close」だけを検出する (PID 比較は thread/fd 複製で誤判定するため使わない)。`attach_generation` は `DRIVEBASE_GET_STATUS` で公開され、再 ATTACH 世代を user-space から追跡可能。

## 6. RT 制御ループ

### 6.1 tick 機構: 絶対時刻 nanosleep

専用 HW timer IRQ は使用しない (TIM6 が Sound DAC TRGO 占有、TIM13/14 は defconfig 未有効、NVIC 0x80–0xF0 が満杯。ハードウェア台帳 `docs/ja/hardware/dma-irq.md` 参照)。

```c
struct timespec next;
clock_gettime(CLOCK_MONOTONIC, &next);
while (atomic_load(&rt->running)) {
    ts_add_ns(&next, 5000000);  /* +5 ms 絶対時刻 */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

    /* jitter 計測 — task wake 直後の clock_gettime と絶対 deadline の差 */
    struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t lag_us = ts_to_us(&now) - ts_to_us(&next);
    record_jitter(rt, lag_us);

    /* tick callback (rt_tick_cb) を実行 */
    if (cr = rt->tick_cb(now_us, arg) < 0) break;
}
```

絶対時刻方式で drift しない。CONFIG_USEC_PER_TICK=10 + tickless TIM9 + SCHED_FIFO 220 によりアイドル時 jitter < 50 µs (max_lag 80 µs を実測)。

### 6.2 RT thread 制約

RT thread 内で禁止: `printf` / `syslog` / `malloc` / blocking mutex / シリアル系 I/O。
許容: `clock_nanosleep` / `clock_gettime` / `read`/`ioctl(motor_fd|imu_fd|chardev_fd)` (CLAIM 済 + O_NONBLOCK / kernel chardev 内部 lock-free) / drivebase_* 内部関数 (単一 thread)。

### 6.3 jitter ring + バケット

`drivebase_jitter_dump_s.hist_us[8]` は wake latency の累積 histogram:

| bucket | 範囲 |
|---|---|
| 0 | < 50 µs |
| 1 | 50–100 µs |
| 2 | 100–200 µs |
| 3 | 200–500 µs |
| 4 | 500–1000 µs |
| 5 | 1–2 ms |
| 6 | 2–5 ms |
| 7 | 5 ms+ |

`deadline_miss_count` は lag ≥ DB_RT_DEADLINE_US (= 1000 µs) のサンプル数。`drivebase jitter reset` で 0 化可能。

### 6.4 RT thread 内の per-tick 処理

`rt_tick_cb` が呼ぶ:

1. `db_chardev_handler_tick` — DAEMON_PICKUP_CMD で cmd_ring を空になるまでドレイン → 各 envelope を dispatch
2. `db_drivebase_update` — encoder drain (両 servo) → observer (sliding-window slope 30 ms) → trajectory(t) reference → cascade PID → SET_PWM/COAST/BRAKE
3. `db_imu_drain_and_update` (use_gyro != NONE のとき) — sensor_imu0 read O_NONBLOCK → Z-gyro 積分 → bias estimator
4. `DAEMON_PUBLISH_STATE` — state_db inactive slot に書込 + atomic swap
5. stall watchdog 判定 (§5.1)

## 7. user task stack と heap (重要 — TLS_ALIGNED)

NuttX BUILD_PROTECTED の **デフォルト**である `CONFIG_TLS_ALIGNED=y` + `TLS_LOG2_MAXSTACK=13` (8 KB アラインメント) は、本 daemon と相性が悪い。defconfig で **無効化** している:

```
# CONFIG_TLS_ALIGNED is not set
```

### 7.1 何が問題か

TLS_ALIGNED=y では、全 user task stack が「8 KB-aligned 8 KB スロット」内に置かれることを要求される (TLS lookup を `sp & ~0x1FFF` だけで行うため)。daemon が起動すると:

- daemon main task stack 8 KB → 8 KB-aligned スロット
- RT pthread stack 4 KB → 別の 8 KB-aligned スロット (上半分は未使用で wasted)

結果、`free` で観測すると user heap (Umem) の `maxfree` が 53240 → 20472 B に劇的に減少 (used は +13 KB しか増えていないのに maxfree が -32 KB)。

その状態で NSH が `drivebase status` の CLI task (4 KB stack) を `task_spawn` しようとすると、別の 8 KB-aligned スロットを要求するが空きが見つからず `-ENOMEM` を返す。`exec_builtin` が失敗、NSH は `cmd_unrecognized` 経路に落ちて **「nsh: drivebase: command not found」** を出す (CONFIG_NSH_FILE_APPS / CONFIG_LIBC_EXECFUNCS が無効なので errno を visible にする救済 path がない)。

### 7.2 修正後

TLS_ALIGNED=n で alignment 強制が消え、`maxfree` の減少は実 stack 消費分 (~13 KB) だけになる。CLI 4 KB stack の確保は安定して成功し、daemon 動作中も `drivebase status / get-state / stop` 全てが round-trip する。

NuttX の Kconfig 自体が "In other builds, the unaligned stack implementation is usually superior" と明記している。BUILD_PROTECTED の TLS_ALIGNED デフォルトは upstream の歴史的経緯で、本ボードで採用するメリットはない。

### 7.3 実測値 (TLS_ALIGNED=y → n)

| metric | TLS_ALIGNED=y | TLS_ALIGNED=n |
|---|---|---|
| Umem free post-start | 45952 B | 45952 B |
| Umem **maxfree** post-start | **20472 B** | **45736 B** |
| `nfree` (free chunk 数) | 4 | 2 |
| `drivebase status` (daemon alive) | ✗ "command not found" | ✓ round-trip OK |

## 8. 制御アルゴリズム

pybricks pbio の C ソースは持ち込まず、SPIKE 専用 / NuttX-native に書き直し。SPIKE Medium Motor (LPF2 type 48) ×2 + SPIKE driving base (wheel 56 mm / axle 112 mm) を前提に最適化。

### 8.1 観測器 (`drivebase_observer.c`)

sliding-window slope estimator (per-sample IIR LP より高 SNR):

- `DB_OBSERVER_RING_DEPTH=64` の ring buffer に `(t_us, x_mdeg)` を蓄積
- 速度推定: window (default 30 ms) 内の最古サンプルとのスロープ
- stall 検出: 期待速度と推定速度の乖離が `stall_low_speed_mdegps` 以下 + duty が `stall_min_duty` 以上 → `stall_window_ms` 連続で `stalled = true`

### 8.2 軌道 (`drivebase_trajectory.c`)

台形プロファイル (accel / cruise / decel)。短い move では triangular fallback (cruise=0) で v_peak を再計算 (整数 sqrt 使用)。`triangle_peak()` は `k = 2·a·d/(a+d)` を先に圧縮してから `v² = d_total · k` を計算する形で uint64 overflow を回避している (Issue [#144](https://github.com/owhinata/spike-nx/issues/144) Step 0)。

`db_trajectory_init_position(t0, x0, x1, v_peak, accel, decel)` で plan、`db_trajectory_get_reference(tr, t)` で `(x_mdeg, v_mdegps, a_mdegps2, done)` を引く。

#### 8.2.1 trajectory_stretch (Phase 4 C、Issue [#144](https://github.com/owhinata/spike-nx/issues/144))

`db_drivebase_drive_curve` は distance と heading の trajectory を独立に planning するため、半径と角度の組み合わせによっては `total_dt_us` が大きくずれる。短い軸 (通常 heading) が先に完了すると、残った軸 (distance) が arc から外れた直線 tangent を走り続けて幾何が崩れる (post-#142 bench: R=300×90 で ~1.5 秒の tangent)。

`db_trajectory_stretch_to_total(tr, T_target_us)` は finite trajectory を slower trapezoid に再 plan して `total_dt_us` を厳密に `T_target_us` へ合わせる:

- `|x1 - x0|` (displacement) は保持
- accel/decel 比は維持 (`accel_mdegps2` / `decel_mdegps2` 不変)
- 二分探索で v_peak ∈ [1, current_v_peak] を求める。各 candidate で `compute_total_dt(v)` を計算、best candidate を保持しながら closest を選ぶ。最後に ±32 の近傍 probe で整数 floor 由来の plateau を吸収
- `cruise_dt = T_target - accel_dt - decel_dt` で時間を厳密一致させ、displacement の微小残差 (≤ 100 mdeg = 0.1°) は `db_trajectory_get_reference()` の最終時刻 clamp (`x = x1` at `t >= total_dt`) で吸収

エラー時は `tr` を mutate しない設計 (`-EINVAL` precondition violation, `-ERANGE` 数値解 fail or residual 100 mdeg 超過)。`db_drivebase_drive_curve` 側は **tmp copy で stretch → 成功時のみ commit** pattern で呼び、失敗時は `syslog(LOG_WARNING)` を出して unstretched fallback (curve 自体は実行され、tangent が出るが user command を abort しない)。

stretch 適用には duration 比のゲート (`DB_TRAJECTORY_STRETCH_RATIO_THRESHOLD_MIL = 1500` = 1.5×) を設けている。2 軸の総 dt 比がこれ未満なら stretch をスキップし、Phase 2 までと同じ独立完了挙動に戻す。理由: 比が小さい (近似的に同期している) curve で stretch を掛けると、heading 軸が slower trajectory を追従する間に `ki_pos=15` の i_acc が積算され、ramp 終端で overshoot が増える regression を実機で確認。実機 bench (post-#142):

| Curve | Duration ratio | Stretch | 結果 |
|---|---|---|---|
| `curve 150 90` | 1.25× | スキップ | baseline 通り (~0.3 秒の軽微な desync のみ) |
| `curve 200 360` | 1.73× | 適用 | tangent 消失 |
| `curve 300 90` | 2.26× | 適用 | tangent 消失 |

閾値は将来 Phase 6 (feed-forward) で再評価。

pybricks reference: `lib/pbio/src/trajectory.c:pbio_trajectory_stretch`。pybricks は時刻ベース parameterization (t1/t2/t3) で連立方程式を解析的に解くが、spike-nx は dt セグメント + 距離ベース parameterization なので数値解 (二分探索) を採用している。

dev verb: `drivebase _alg stretch <f_x1> <f_v> <f_a> <f_d> <l_x1> <l_v> <l_a> <l_d>` で BEFORE/AFTER + 0/25/50/75/100% timeline サンプル + residual bound check (≤100 mdeg threshold) をハードウェア無しで実行可能。

注意: Phase 5 (#142) の reference-time pause は per-axis 実装。片軸だけ pause すると stretch で同期させた pair が崩れる可能性があるが、これは #142 既存仕様として許容 (drivebase-level shared pause は別 Issue 候補)。

### 8.3 PID + 完了判定 (`drivebase_control.c`)

cascade PID (位置 P+I+D + 速度 P+I)。anti-windup は二段構え:

1. **I 項凍結** (`should_freeze_integrator`) — error が deadband 内 (±3000 mdeg = ±3°) / output が windup 方向に saturate / controller paused のとき積算停止
2. **Reference-time pause** (Issue [#142](https://github.com/owhinata/spike-nx/issues/142) Phase 5、`drivebase_aggregate.c`) — P 項と同方向に出力が rail clamp され、かつ ref が逆方向に減速していないとき trajectory 時刻自体を凍結。state が catch-up するまで ref を進めず、加速 phase で pos_err が育って i_acc が暴走する経路を構造的に断つ

両者は per-axis (distance / heading) で独立して発火。pybricks `pbio_position_integrator` (lib/pbio/src/integrator.c L142-218) の移植。

完了判定:
- `|position_error| < pos_tolerance` && `|speed_error| < speed_tolerance (30 dps)` を `done_window_ms (50 ms)` 連続維持 → `is_done = true`
- `pos_tolerance` は kp_pos から物理導出 (Issue [#140](https://github.com/owhinata/spike-nx/issues/140) Phase 1 F): `max(1000, 400 × 1000 / kp_pos)` mdeg = kp=50 で 8000 mdeg (≈ 4 mm wheel)

`on_completion` 種別ごとの終端動作:

| 種別 | is_done 後の動作 |
|---|---|
| COAST | motor coast、reference freeze、controller pause |
| BRAKE | motor brake、reference freeze、controller pause |
| HOLD | duty=0 でなく PID で位置維持 (active hold) |
| CONTINUE | trajectory 等速継続 (停止しない) |
| COAST_SMART | 完了時点から `smart_passive_hold_time (~100 ms)` torque 維持 → coast へ degrade |
| BRAKE_SMART | 同上で brake へ degrade |

SMART 完了は次の relative move 開始判定 (`|prev_endpoint - current_position| < pos_tolerance × 2` で前回 endpoint から trajectory を組む = pybricks "continue from endpoint") の前提でもある。

### 8.4 L/R 集約 (`drivebase_drivebase.c`)

左右 servo の actual position から:
- `distance = (l_pos + r_pos) / 2 × wheel_circumference / 360`
- `heading_rad = (r_pos - l_pos) × wheel_circumference / (2 × axle_track × π / 360)`

distance + heading の 2 系 PID 出力 (mm/s, deg/s) を逆変換して L/R 各 servo の reference speed に分配。gyro が ON なら heading actual を gyro 推定 heading で上書き (encoder の slip 影響を排除)。

### 8.5 設定 (`drivebase_settings.c`)

SPIKE Medium Motor 用 default gain (per-axis、Issue #141 Phase 2 で集約 PID 化):

| 軸 | kp_pos | ki_pos | kd_pos | kp_speed | out_max |
|---|---|---|---|---|---|
| distance | 50 | 15 | 0 | 5 | ±10000 (±100% duty) |
| heading | 50 | 15 | 0 | 5 | ±8000 (±80% duty、L/R compose 余裕) |

ki_pos=15 は SPIKE duty 直駆動 + feed-forward なし環境で静摩擦 (~12-18% breakaway) を超えるための値。pybricks の "drivebase 集約 PID は ki=0" は torque-output 前提なので採用不可。Issue #142 Phase 5 Step B で ki=10 を試したが短距離で 5 mm undershoot、長距離で done 立たずの regression が出たため ki=15 に確定 (Phase 6 で feed-forward 導入後に再評価)。

drive_speed default は wheel_diameter から算出 (`v_max_mdegps`、`accel = v_max × 4 / 1` = 1/4 sec で max speed)。`drivebase _alg settings` でダンプ可能。

### 8.6 ランタイム設定上書き (`/mnt/flash/drivebase.cfg`, Issue [#143](https://github.com/owhinata/spike-nx/issues/143))

`drivebase start` 時に外部 W25Q256 NOR (LittleFS `/mnt/flash`) 上の text file から PID gain / completion / stall / wheel/axle/tick を読み込み、ビルド済みバイナリの default を rebuild なしで上書きできる。bench tuning iteration 短縮が目的。

**形式**: properties (key=value)、`#` でコメント (行頭または value 中の `=` の右側、最初の `#` 以降を value から削除)、空行と前後 whitespace は無視。LF / CRLF 両対応、先頭 BOM 許容。行長上限 256 bytes (超過は warn して skip)。未知 key / parse error は LOG_WARNING の上で **その行だけ skip**、他 key は適用。

**精度ガード**:
- ファイル無し (`ENOENT`) → silent fallback (dmesg 無し)
- mount 失敗 / 他 IO エラー → `LOG_WARNING`
- 1 key 以上適用 → `LOG_INFO`: `drivebase: loaded N config key(s) from /mnt/flash/drivebase.cfg`
- ファイルあり + 0 key 適用 → `LOG_WARNING` (typo 検出を可視化)

**Race 防止**: `db_settings_freeze()` を `db_chardev_handler_attach()` 直前で呼び、RT thread 起動後の write は `-EBUSY` で構造的に拒否。read race は発生しない。

**サポート key 一覧** (規定 default を括弧内に併記):

| グループ | key | 型 | 範囲 / 意味 |
|---|---|---|---|
| PID dist | `pid_dist_kp_pos` (50) | int32 | (0, 10000] |
| | `pid_dist_ki_pos` (15) | int32 | [0, 10000] |
| | `pid_dist_kd_pos` (0) | int32 | [0, 10000] |
| | `pid_dist_kp_speed` (5) | int32 | [0, 10000] |
| | `pid_dist_ki_speed` (0) | int32 | [0, 10000] |
| | `pid_dist_deadband_mdeg` (3000) | int32 | [0, 100000] mdeg |
| | `pid_dist_out_max` (10000) | int32 | (0, 10000]、out_min は対称 |
| PID head | `pid_head_*` | 同上、`out_max` default 8000 |
| Completion | `comp_dist_pos_tol_mdeg` (-1) | int32 | -1 で kp_pos 由来導出 (#140)、または [1000, 60000] |
| | `comp_dist_speed_tol_mdegps` (30000) | int32 | [0, 1000000] |
| | `comp_dist_smart_continue_mdeg` (-1) | int32 | -1 で default 6000、または [0, 60000] |
| | `comp_dist_done_window_ms` (50) | uint32 | [0, 10000] |
| | `comp_dist_smart_passive_hold_ms` (100) | uint32 | [0, 10000] |
| | `comp_head_*` | 同上 |
| Stall | `stall_speed_mdegps` (30000) | int32 | [0, 1000000] |
| | `stall_duty_min` (6000) | int32 | [0, 10000] |
| | `stall_window_ms` (200) | uint32 | [0, 10000] |
| Start | `wheel_d_um` (56000) | uint32 | > 0、`drivebase start` CLI で wheel 省略時のみ採用 |
| | `axle_t_um` (112000) | uint32 | > 0、CLI で axle 省略時のみ採用 |
| | `tick_us` (DB_RT_TICK_US_DEFAULT) | uint32 | > 0、CLI で tick 省略時のみ採用 |

**Wheel / axle / tick の precedence**: `drivebase start [wheel_mm] [axle_mm] [tick_ms]` で argc により明示判定。明示時は CLI 優先、省略時は config を採用、両方無しでコード default。

**編集手段** (on-device): defconfig で `CONFIG_SYSTEM_VI=y` + `CONFIG_SYSTEM_TERMCURSES=y` 有効化済み。NSH 上で:

```
nsh> vi /mnt/flash/drivebase.cfg
```

最小書き換えなら `cat` + redirect でも可:

```
nsh> echo "pid_dist_ki_pos = 12" >> /mnt/flash/drivebase.cfg
nsh> drivebase stop
nsh> drivebase start
nsh> drivebase _alg settings   # 適用値の確認
```

**サンプル** (`/mnt/flash/drivebase.cfg`):

```
# Phase 5 Step B 再 sweep 用に ki を一時的に下げる
pid_dist_ki_pos = 10
pid_head_ki_pos = 10

# 物理車両が SPIKE デフォルトと違う場合
wheel_d_um = 62000
axle_t_um = 145000
```

**完全テンプレート**: 全 30 key + コメントを `apps/drivebase/drivebase.cfg.sample` にコミットしてある。リポジトリから zmodem 等で `/mnt/flash/` にコピーするか、内容を `vi` で貼り付ければ「全 key を明示指定」の状態から start できる。tuning 中は使う key だけ残して他を `#` でコメントアウトする運用が見通しが良い。

## 9. Linux への移植性 (FUSE)

本 daemon は kernel chardev + IPC 設計のため、将来の Linux ポートで FUSE (`/dev/fuse` + libfuse) に置き換え可能:

- **NuttX 版**: kernel chardev が VFS 受け口、cmd_ring + sem_post で daemon 起こし
- **Linux 版**: FUSE filesystem 内で同じ ioctl 番号 + struct を `FUSE_IOCTL` callback で受ける。daemon が直接 callback として呼ばれる
- **差分は IPC レイヤだけ**: cmd_ring + state_db の bridge code (~250 行 kernel + DAEMON_ATTACH/PICKUP/PUBLISH ioctl) が消え、daemon 内の chardev_handler ロジックは無改変で動く
- ABI struct + user-facing ioctl 番号は両環境で完全一致

これを実現するための ABI 規約:
- 固定幅型のみ (`uint32_t` / `int32_t` / `uint64_t` / `uint8_t`)、`long` / pointer / `time_t` / `bool` を struct field に入れない
- `_Static_assert(sizeof(struct ...) == N)` + `_Static_assert(offsetof(...) == K)` で全 struct 検証
- 可変長 / pointer 内包 struct は禁止

**STOP latency 保証は環境依存:**
- NuttX 版: kernel context emergency_cb で < 100 µs typical
- Linux/FUSE 版: daemon の userspace scheduler 依存。同等保証は不可 (FUSE callback 経由のため)。Linux ポート時は別経路 (cgroup priority / RT scheduler / kernel module bypass 等) を要する

## 10. トラブルシューティング

### 10.1 `drivebase status` が "command not found" を返す

§7 を参照。`CONFIG_TLS_ALIGNED=y` が原因。defconfig で `# CONFIG_TLS_ALIGNED is not set` を追加 + `make nuttx-distclean && make` で解消。

### 10.2 daemon が起動直後に sem_post せず teardown_done で詰まる

motor が両ポート (odd + even) に挿さっていない可能性。`drivebase_motor_init` が `LEGOSENSOR_GET_INFO` で type_id != 48 を検出すると `-ENODEV` で失敗し、`fail` ラベルに jump して teardown_done を post せずに return。

実機 dmesg で `lump: port X: SYNCED type=48` が両 port で出ていることを確認。

### 10.3 IMU 機能 (`set-gyro 1d`) が効かない

`/dev/uorb/sensor_imu0` が boot 時の LSM6DSL init 失敗で存在しない可能性。dmesg で `ERROR: Failed to initialize LSM6DSL: -110` が出ていれば I2C2 配線 / pull-up / boot 順序を確認。daemon は best-effort なので IMU 失敗でも encoder-only で動く (`drivebase status` の `imu_present=0`)。

### 10.4 stall watchdog が誤発火する

deadline_miss が 5 連続で coast 経路に乗る。`drivebase jitter` で wake latency の histogram を確認:
- bucket 5+ (≥ 1 ms) が継続的に出る → BTstack / sound DMA / flash I/O との競合の可能性
- `apps/btsensor/btsensor stop` で BT を止めて再現確認
- `apps/sound/sound stop` で sound DMA を止めて再現確認

### 10.5 multiple start/stop の後で daemon が立たない

`attach_generation` が GET_STATUS で正常に増加しているか確認。kernel chardev の close cleanup が走らずに `attached=true` のままになっていると、次の ATTACH が `-EBUSY` で蹴られる。`reboot` でクリーン状態に戻る。

## 11. 参照

- ハードウェア台帳: `docs/ja/hardware/dma-irq.md` (TIM/DMA/NVIC 占有状況)
- LUMP プロトコル: `docs/ja/drivers/lump-protocol.md`
- LEGOSENSOR ABI: `docs/ja/drivers/sensor.md` (motor_l/m/r class topic、SET_PWM / COAST / BRAKE)
- IMU drain パターン: `docs/ja/drivers/imu.md` (sensor_imu0 uORB)
- アルゴリズム参照 (コピー禁止): `pybricks/lib/pbio/src/{drivebase,servo,trajectory,observer,control,integrator,control_settings}.c`
