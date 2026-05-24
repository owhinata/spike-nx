# drivebase daemon (Issue #77)

A 5 ms closed-loop controller that runs the SPIKE Prime Hub's two-motor drivebase from a userspace daemon.  Ports the pybricks drivebase feature set (drive_straight / turn / forever / stop / on_completion variants, etc.) at the **behaviour-parity** level — pbio's C sources are not vendored; everything is rewritten as SPIKE-specific, NuttX-native code.

## 1. Architecture

### 1.1 Layers

```
user app                  kernel /dev/drivebase chardev          userspace daemon
─────────                 ─────────────────────────────          ────────────────
ioctl(fd, DRIVEBASE_*)    cmd_ring (MPSC, depth 8)     ────►    DAEMON_PICKUP_CMD
                          state_db (double-buffer)     ◄────    DAEMON_PUBLISH_STATE
                          status (seqlock)             ◄────    DAEMON_PUBLISH_STATUS
                          emergency_stop_cb (kernel)
                                                                RT pthread (SCHED_FIFO 220, 5 ms)
                                                                motor encoder drain (LEGOSENSOR)
                                                                IMU drain (LSM6DSL uORB)
                                                                observer + trajectory + PID
                                                                motor SET_PWM / COAST / BRAKE
```

User-facing ioctls and daemon-internal ioctls live in separate headers (FUSE portability — see §9):

| header | visibility | contents |
|---|---|---|
| `boards/spike-prime-hub/include/board_drivebase.h` | public | DRIVEBASE_DRIVE_STRAIGHT / TURN / FOREVER / STOP / GET_STATE / GET_STATUS / JITTER_DUMP — the user-facing ABI |
| `boards/spike-prime-hub/include/board_drivebase_internal.h` | internal | DRIVEBASE_DAEMON_ATTACH / DETACH / PICKUP_CMD / PUBLISH_STATE / PUBLISH_STATUS — daemon ↔ kernel chardev only |

### 1.2 chardev registration

`/dev/drivebase` is registered kernel-side from `boards/spike-prime-hub/src/stm32_drivebase_chardev.c`.  The device file exists even when no daemon is running; `open()` always succeeds (drive verbs return `-ENOTCONN` until a daemon attaches).

NuttX BUILD_PROTECTED does not export `register_driver()` to userspace, so kernel-side registration is the only viable path.  The userspace daemon "attaches" via the regular ioctl channel (DAEMON_ATTACH).

## 2. File layout

```
boards/spike-prime-hub/
├── include/
│   ├── board_drivebase.h            user-facing ABI
│   └── board_drivebase_internal.h   daemon-internal ABI
└── src/
    └── stm32_drivebase_chardev.c    kernel chardev shim (cmd_ring / state_db / watchdog)

apps/drivebase/
├── Kconfig                          APP_DRIVEBASE + RT priority / stack / gain defaults
├── Makefile                         CLI builtin (PROGNAME=drivebase, STACKSIZE=4096)
├── drivebase_main.c                 NSH CLI: start/stop/status/config/straight/...
├── drivebase_daemon.{c,h}           lifecycle FSM + idle loop + stall watchdog
├── drivebase_chardev_handler.{c,h}  DAEMON_ATTACH/PICKUP/PUBLISH wrappers
├── drivebase_rt.{c,h}               SCHED_FIFO 220 + clock_nanosleep 5 ms tick + jitter ring
├── drivebase_motor.{c,h}            sensor_motor_l/r encoder drain + SET_PWM/COAST/BRAKE
├── drivebase_imu.{c,h}              sensor_imu0 drain + Z-gyro 1D heading
├── drivebase_drivebase.{c,h}        L/R aggregation (drive_straight / turn / curve / arc / forever)
├── drivebase_servo.{c,h}            per-motor closed loop
├── drivebase_observer.{c,h}         sliding-window slope (default 30 ms)
├── drivebase_control.{c,h}          PID + anti-windup + on_completion handling
├── drivebase_trajectory.{c,h}       trapezoidal profile (accel / cruise / decel + triangular fallback)
├── drivebase_settings.{c,h}         gain table + drive_speed defaults + completion thresholds
├── drivebase_angle.h                int64 mdeg + deg ↔ mm conversion (π = 355/113)
└── drivebase_internal.h             enum db_side_e and shared types
```

## 3. ABI

### 3.1 ioctl numbers

Defined in `board_drivebase.h` (group `_DBASEBASE = 0x4900`).

User-facing:

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

### 3.2 Key structs

All structs use fixed-width types only and carry `_Static_assert(sizeof(...) == N)` size locks (32/64-bit compatible for the eventual Linux FUSE port).

```c
enum drivebase_on_completion_e {
  DRIVEBASE_ON_COMPLETION_COAST       = 0,  /* coast on completion */
  DRIVEBASE_ON_COMPLETION_BRAKE       = 1,  /* short-circuit brake */
  DRIVEBASE_ON_COMPLETION_HOLD        = 2,  /* active position hold via PID */
  DRIVEBASE_ON_COMPLETION_CONTINUE    = 3,  /* keep cruising (do not stop) */
  DRIVEBASE_ON_COMPLETION_COAST_SMART = 4,  /* SMART: hold ~100 ms then coast */
  DRIVEBASE_ON_COMPLETION_BRAKE_SMART = 5,  /* SMART: hold ~100 ms then brake */
};

struct drivebase_config_s          { uint32_t wheel_d_um; uint32_t axle_t_um; uint8_t r[8]; };
struct drivebase_drive_straight_s  { int32_t distance_mm; uint8_t on_completion; uint8_t r[7]; };
struct drivebase_turn_s            { int32_t angle_deg; uint8_t on_completion; uint8_t r[3]; };
struct drivebase_drive_forever_s   { int32_t speed_mmps; int32_t turn_rate_dps; };
struct drivebase_stop_s            { uint8_t on_completion; uint8_t r[7]; };

struct drivebase_state_s
{
  int32_t  distance_mm;       /* cumulative forward distance at the chassis centre */
  int32_t  drive_speed_mmps;
  int32_t  angle_mdeg;        /* heading × 1000 (encoder, optionally overridden by gyro) */
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

`drivebase_jitter_dump_s` is described in §6.3.

### 3.3 cmd_ring push policy

Multiple user fds may produce, so the ring is MPSC.  The kernel side serialises producers briefly through `producer_lock` (nxmutex).

| situation | policy |
|---|---|
| ring full for a regular command | return `-EBUSY` immediately (no caller blocking, avoids priority inversion) |
| STOP command and ring tail is already STOP | overwrite in place (coalesce) |
| STOP command and ring full | drop the oldest non-STOP envelope to make room (STOP must always land) |

This guarantees `DRIVEBASE_STOP` is accepted even when the ring is saturated.

### 3.4 Drive-verb error codes

| situation | return |
|---|---|
| daemon not running | `-ENOTCONN` |
| drive verb before `DRIVEBASE_CONFIG` | `-ENOTCONN` |
| second ATTACH while daemon already attached | `-EBUSY` |
| one motor unplugged (type_id != 48) | `-ENODEV` |
| LEGOSENSOR_CLAIM contention | `-EBUSY` |
| stall watchdog has fired | `-EIO` |
| reserved bytes != 0 / out-of-range arg | `-EINVAL` |

### 3.5 Emergency-stop fast path

`DRIVEBASE_STOP` is fully resolved inside the ioctl:

1. atomic increment `output_epoch`
2. Based on `default_on_completion`, call the kernel-resident static helper `stm32_legoport_pwm_coast()` or `stm32_legoport_pwm_brake()` directly.
3. Push a STOP envelope into cmd_ring so the daemon can sync its trajectory state.

No daemon round-trip, no locks (atomics + lock-free ring) → `< 100 µs latency` resolved entirely in kernel context.

emergency_stop **never registers a user-space function pointer with the kernel**.  Doing so would dangle when the daemon segfaults and risk a kernel HardFault.  Instead the daemon hands the kernel only the `legoport port idx (0..5)` for left/right at ATTACH time, and the kernel calls the resident `stm32_legoport_pwm_*` by index.

## 4. CLI

PROGNAME = `drivebase` (NSH builtin).  STACKSIZE = 4096 (see §7).

| sub-command | behaviour |
|---|---|
| `drivebase` (no args) | print usage |
| `drivebase status` | dump DRIVEBASE_GET_STATUS snapshot (works without a daemon) |
| `drivebase start [wheel_mm [axle_mm [tick_ms]]]` | spawn the daemon.  `wheel_mm` / `axle_mm` accept decimals (e.g. `17.6 56.5`); `tick_ms` sets the RT control period (Issue #120, range 1–20 ms).  Defaults: 56 / 112 / 2.  **Auto-launched at board boot from rcS with the default arguments (Issue #120)** |
| `drivebase stop` | graceful daemon teardown (returns within ~2 s) |
| `drivebase config <wheel_mm> <axle_mm>` | DRIVEBASE_CONFIG.  Decimals OK; the wire ABI carries micrometers so sub-mm precision survives (currently optional — daemon already takes wheel/axle from `start`) |
| `drivebase reset [distance_mm] [angle_deg]` | DRIVEBASE_RESET — re-anchors the published distance/heading at the requested baseline (default 0/0).  Daemon also auto-resets on `drivebase start`, so this verb is mainly for re-zeroing mid-session |
| `drivebase straight <mm> [<mmps>] [coast\|brake\|hold]` | DRIVE_STRAIGHT.  Omit `mmps` (or pass `0`) to use the `db_settings` default |
| `drivebase turn <deg> [<dps>] [coast\|brake\|hold]` | TURN (CCW positive).  Omit `dps` for the default |
| `drivebase curve <radius_mm> <angle_deg> [coast\|brake\|hold]` | DRIVE_CURVE (Issue [#138](https://github.com/owhinata/spike-nx/issues/138)) — sweep an arc of radius `R` over `angle_deg`.  `R == 0` falls back to `turn`.  Phase 4 (C) `trajectory_stretch` (§8.2.1) keeps distance and heading completing simultaneously |
| `drivebase arc <radius_mm> <distance_mm> [coast\|brake\|hold]` | DRIVE_ARC_DISTANCE (Issue [#138](https://github.com/owhinata/spike-nx/issues/138)) — travel `distance_mm` along an arc of radius `R`.  `R == 0` falls back to `straight` |
| `drivebase forever <mmps> <dps>` | DRIVE_FOREVER (no completion, distance + heading concurrently) |
| `drivebase stop-motion <coast\|brake\|hold>` | DRIVEBASE_STOP (emergency fast path) |
| `drivebase get-state [duration_ms [interval_ms]]` | DRIVEBASE_GET_STATE.  Default = single-row table; with `duration_ms` polls every `interval_ms` (default 100 ms) and prints one row per sample |
| `drivebase set-gyro <none\|1d\|3d>` | DRIVEBASE_SET_USE_GYRO (override heading source) |
| `drivebase jitter [reset]` | DRIVEBASE_JITTER_DUMP (RT loop wake-latency stats) |

Hidden development verbs: `_motor`, `_alg`, `_servo`, `_drive`, `_rt`, `_daemon`, `_imu`.  These existed for the staged commit-by-commit verification of the lifecycle FSM and **must not be invoked while the daemon is running** — they share BSS-resident `static` structs with `g_daemon` and would trample its state.

## 5. Lifecycle FSM

`daemon_task_main` in `drivebase_daemon.c`:

```
DAEMON_STOPPED → drivebase_daemon_start() → DAEMON_INITIALIZING
   1. drivebase_motor_init                 (open + CLAIM /dev/uorb/sensor_motor_l/r,
                                            verify type=48 via LEGOSENSOR_GET_INFO)
   2. drivebase_motor_select_mode(L/R, 2)  (LUMP mode 2 = POS = signed int32 deg, ~30 ms warm-up)
   3. db_drivebase_init + reset            (geometry + servo[L/R] + observer + control)
   4. db_chardev_handler_attach            (open /dev/drivebase + DAEMON_ATTACH)
   5. db_imu_open                          (best-effort; encoder-only continuation if it fails)
   6. db_rt_init + db_rt_start             (spawn RT pthread, SCHED_FIFO 220, 4 KB stack)
DAEMON_INITIALIZING → DAEMON_RUNNING
   idle loop: usleep(50 ms) + DAEMON_PUBLISH_STATUS once per cycle
DAEMON_RUNNING → drivebase_daemon_stop() → DAEMON_TEARDOWN
   1. atomic_store(running = false)
   2. db_rt_stop                           (join the RT pthread, ≤ 1 tick wait)
   3. drivebase_motor_coast(L/R)           (explicit coast on both motors)
   4. db_imu_close
   5. db_chardev_handler_detach            (DAEMON_DETACH + close fd → kernel auto-coasts)
   6. drivebase_motor_deinit               (close LEGOSENSOR fds → legoport chardev auto-coasts as a safety net)
DAEMON_TEARDOWN → sem_post(teardown_done) → DAEMON_STOPPED
```

`drivebase_daemon_stop(timeout_ms)` (default 2000 ms) waits on `teardown_done`.

### 5.1 Daemon-side stall watchdog

The RT tick callback (`rt_tick_cb`) tracks `db_rt_s.deadline_miss_count`:

- Five consecutive deadline misses (lag > 1 ms) trigger an emergency `drivebase_motor_coast(L/R)`
- `atomic_store(running = false)` takes the daemon down the teardown path

### 5.2 Kernel-side stale-daemon watchdog

`stm32_drivebase_chardev.c` arms an LPWORK item polling every 25 ms that watches `last_publish_ticks`:

- If the daemon hasn't called PUBLISH_STATE for ≥ 50 ms it is considered stale
- `db_emergency_actuate(default_on_completion)` directly stops the motors from kernel context
- `db_detach_locked()` clears the attach state — drive verbs return `-ENOTCONN` afterwards

Together with §5.1, this guarantees motor safety even if the daemon livelocks.

### 5.3 ATTACH-fd close cleanup

The chardev `close()` fop:

```c
if (dev->attached && dev->attach_filep == filep) {
    /* The fd that ATTACHed was just closed (daemon exit or segfault) */
    db_detach_locked(dev, true);   /* emergency_stop = true → coast motors */
}
```

Comparing `attach_filep` by pointer detects "the ATTACH fd was closed" precisely (PID comparison would mis-fire under thread / fd duplication).  `attach_generation` is exposed via `DRIVEBASE_GET_STATUS` so user space can track re-attach generations.

## 6. RT control loop

### 6.1 Tick mechanism: absolute-time nanosleep

A dedicated HW timer IRQ is **not** used.  TIM6 is owned by the Sound DAC TRGO, TIM13/14 are not enabled in the defconfig, and NVIC priorities 0x80–0xF0 are full (see the hardware ledger `docs/en/hardware/dma-irq.md`).

```c
struct timespec next;
clock_gettime(CLOCK_MONOTONIC, &next);
while (atomic_load(&rt->running)) {
    ts_add_ns(&next, 5000000);  /* +5 ms absolute */
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

    /* Sample jitter as the gap between the absolute deadline and post-wake clock_gettime */
    struct timespec now; clock_gettime(CLOCK_MONOTONIC, &now);
    int64_t lag_us = ts_to_us(&now) - ts_to_us(&next);
    record_jitter(rt, lag_us);

    /* Run the user-supplied tick callback (rt_tick_cb) */
    if ((cr = rt->tick_cb(now_us, arg)) < 0) break;
}
```

Absolute-time scheduling is drift-free.  With `CONFIG_USEC_PER_TICK=10` + tickless TIM9 + SCHED_FIFO 220, idle-state jitter measures < 50 µs (max_lag 80 µs observed).

### 6.2 RT-thread constraints

Forbidden inside the RT thread: `printf`, `syslog`, `malloc`, blocking mutexes, serial I/O.  Allowed: `clock_nanosleep`, `clock_gettime`, `read`/`ioctl(motor_fd|imu_fd|chardev_fd)` (CLAIM-held + O_NONBLOCK / kernel chardev is internally lock-free), and the daemon's own single-threaded internals.

### 6.3 Jitter ring + buckets

`drivebase_jitter_dump_s.hist_us[8]` is a cumulative histogram of wake latencies:

| bucket | range |
|---|---|
| 0 | < 50 µs |
| 1 | 50–100 µs |
| 2 | 100–200 µs |
| 3 | 200–500 µs |
| 4 | 500–1000 µs |
| 5 | 1–2 ms |
| 6 | 2–5 ms |
| 7 | 5 ms+ |

`deadline_miss_count` counts samples with lag ≥ DB_RT_DEADLINE_US (= 1000 µs).  `drivebase jitter reset` zeros the counters.

### 6.4 Per-tick work

`rt_tick_cb` runs:

1. `db_chardev_handler_tick` — DAEMON_PICKUP_CMD drains cmd_ring to empty, dispatching each envelope
2. `db_drivebase_update` — encoder drain (both servos) → observer (sliding-window slope, 30 ms) → trajectory(t) reference → cascade PID → SET_PWM/COAST/BRAKE
3. `db_imu_drain_and_update` (when use_gyro != NONE) — non-blocking read of sensor_imu0 → Z-gyro integration → bias estimator
4. `DAEMON_PUBLISH_STATE` — write into the inactive state_db slot, atomic-swap active
5. Stall watchdog evaluation (§5.1)

## 7. User-task stack and heap (important — TLS_ALIGNED)

The NuttX BUILD_PROTECTED **default** of `CONFIG_TLS_ALIGNED=y` + `TLS_LOG2_MAXSTACK=13` (8 KB alignment) interacts badly with this daemon.  The defconfig **disables** it:

```
# CONFIG_TLS_ALIGNED is not set
```

### 7.1 What goes wrong with the default

With `CONFIG_TLS_ALIGNED=y`, every user-task stack must live inside an "8 KB-aligned 8 KB slot" (so TLS lookup can be done with `sp & ~0x1FFF` alone).  When the daemon comes up:

- daemon main task stack 8 KB → one 8 KB-aligned slot
- RT pthread stack 4 KB → another 8 KB-aligned slot (the upper half is wasted)

Observed via `free`: the user heap (`Umem`) `maxfree` collapses from 53240 B to 20472 B even though `used` only grew by 13 KB — the missing 32 KB is fragmentation.

When NSH then tries to spawn the `drivebase status` CLI task (4 KB stack), it requests yet another 8 KB-aligned slot; allocation fails with `-ENOMEM`.  `exec_builtin` gives up and NSH falls through to `cmd_unrecognized`, surfacing the failure as **"nsh: drivebase: command not found"** because `CONFIG_NSH_FILE_APPS` and `CONFIG_LIBC_EXECFUNCS` are off — there's no rescue path that exposes the real errno.

### 7.2 With the fix

`CONFIG_TLS_ALIGNED=n` removes the alignment requirement, so `maxfree` only loses the actual stack consumption (~13 KB).  The 4 KB CLI stack allocates reliably and `drivebase status / get-state / stop` all round-trip cleanly while the daemon is alive.

The Kconfig itself flags this: *"In other builds, the unaligned stack implementation is usually superior."*  TLS_ALIGNED's BUILD_PROTECTED default is upstream history — there's no benefit to keeping it on for this board.

### 7.3 Measured impact (TLS_ALIGNED y → n)

| metric | TLS_ALIGNED=y | TLS_ALIGNED=n |
|---|---|---|
| Umem free post-start | 45952 B | 45952 B |
| Umem **maxfree** post-start | **20472 B** | **45736 B** |
| `nfree` (free-chunk count) | 4 | 2 |
| `drivebase status` (daemon alive) | ✗ "command not found" | ✓ round-trips |

## 8. Control algorithm

pybricks pbio is read for behavioural reference only — none of its C is vendored.  Everything is rewritten as SPIKE-specific, NuttX-native code that assumes SPIKE Medium Motor (LPF2 type 48) ×2 + the SPIKE driving base (wheel 56 mm / axle 112 mm).

### 8.1 Observer (`drivebase_observer.c`)

Sliding-window slope estimator (better SNR than per-sample IIR):

- `DB_OBSERVER_RING_DEPTH = 64` ring buffer of `(t_us, x_mdeg)` samples
- Velocity estimate = slope between the newest sample and the oldest sample within the window (default 30 ms)
- Stall detection: actual velocity below `stall_low_speed_mdegps` while applied duty stays above `stall_min_duty` for `stall_window_ms` consecutive ticks → `stalled = true`

### 8.2 Trajectory (`drivebase_trajectory.c`)

Trapezoidal profile (accel / cruise / decel).  Short moves degenerate to a triangular fallback (cruise=0) with `v_peak` recomputed via integer sqrt.  `triangle_peak()` factorises `k = 2·a·d/(a+d)` before multiplying by `d_total` to keep the intermediate product inside `uint64` at realistic upper bounds (Issue [#144](https://github.com/owhinata/spike-nx/issues/144) Step 0).

`db_trajectory_init_position(t0, x0, x1, v_peak, accel, decel)` plans the move; `db_trajectory_get_reference(tr, t)` returns `(x_mdeg, v_mdegps, a_mdegps2, done)`.

#### 8.2.1 trajectory_stretch (Phase 4 C, Issue [#144](https://github.com/owhinata/spike-nx/issues/144))

`db_drivebase_drive_curve` plans the distance and heading trajectories independently, so the two `total_dt_us` values can diverge depending on (radius, angle).  When the shorter axis (usually heading) finishes first, the longer axis (usually distance) continues, producing a visible straight-line tangent at the tail of the arc (post-#142 bench: ~1.5 s tangent at R=300×90).

`db_trajectory_stretch_to_total(tr, T_target_us)` re-plans a finite trajectory as a slower trapezoid that takes exactly `T_target_us`:

- `|x1 - x0|` (displacement) preserved
- accel/decel ratio preserved (`accel_mdegps2` / `decel_mdegps2` unchanged)
- binary search picks `v_peak ∈ [1, current_v_peak]` minimising `|compute_total_dt(v) - T_target|`, keeping a best candidate during the descent, then probes ±32 around the converged value to absorb integer-floor plateaus
- `cruise_dt = T_target - accel_dt - decel_dt` forces exact `total_dt`; the displacement residual (≤ 100 mdeg = 0.1°) is absorbed by `db_trajectory_get_reference()`'s end-time clamp (`x = x1` at `t >= total_dt`)

On failure (precondition `-EINVAL`, numerical solver `-ERANGE`, or residual gate trip > 100 mdeg) the helper leaves `tr` untouched.  `db_drivebase_drive_curve` uses a **tmp-copy commit pattern**: stretch a local copy, only overwrite the live trajectory on success.  Failure emits a `syslog(LOG_WARNING)` and falls back to the unstretched curve so the user command is not aborted (the tangent regression is observable instead).

Stretch is gated by a duration-ratio threshold (`DB_TRAJECTORY_STRETCH_RATIO_THRESHOLD_MIL = 1500`, i.e. 1.5×).  When the two axes' total_dt ratio is below the threshold the trajectories are similar enough that the tangent is barely visible; stretching the shorter one would make the heading PID accumulate i_acc (`ki_pos = 15`) over a needlessly longer trajectory and overshoot at the ramp end (regression observed on bench at ratio 1.25×).  Above the threshold the tangent regression dominates and stretching wins.  Empirical bench (post-#142):

| Curve | Duration ratio | Stretch | Outcome |
|---|---|---|---|
| `curve 150 90` | 1.25× | skipped | clean baseline preserved (~0.3 s mild desync) |
| `curve 200 360` | 1.73× | applied | tangent eliminated |
| `curve 300 90` | 2.26× | applied | tangent eliminated |

Tuning this knob downwards is a Phase 6 (feed-forward) follow-up.

pybricks reference: `lib/pbio/src/trajectory.c:pbio_trajectory_stretch`.  pybricks parametrises trajectories by absolute timestamps (t1/t2/t3) and solves the trapezoid analytically; spike-nx uses dt segments and a displacement-based parametrisation, so we solve numerically with binary search.

Dev verb: `drivebase _alg stretch <f_x1> <f_v> <f_a> <f_d> <l_x1> <l_v> <l_a> <l_d>` exercises the helper without hardware — prints BEFORE/AFTER state, a 0/25/50/75/100% timeline sample of both trajectories, and a residual-bound PASS/FAIL gate (≤ 100 mdeg).

Caveat: the reference-time pause from #142 is per-axis.  If only one axis pauses mid-curve the stretched pair desynchronises — accepted as the existing #142 design (a drivebase-level shared pause is a follow-up Issue candidate).

### 8.3 PID + completion (`drivebase_control.c`)

Cascade PID (position P+I+D feeding velocity P+I).  Anti-windup is two-tiered:

1. **I-term freeze** (`should_freeze_integrator`) — stops accumulation when error is inside the deadband (±3000 mdeg = ±3°), when output is saturated in the windup direction, or when the controller is paused.
2. **Reference-time pause** (Issue [#142](https://github.com/owhinata/spike-nx/issues/142) Phase 5, `drivebase_aggregate.c`) — freezes trajectory time itself when the P term is rail-clamped in its own direction AND the reference is not decelerating away.  The reference cannot run ahead of the state during prolonged saturation, structurally cutting the path where `pos_err` grows during the accel phase and inflates `i_acc`.

Both tiers fire per-axis (distance / heading) independently.  Ported from pybricks `pbio_position_integrator` (lib/pbio/src/integrator.c L142-218).

Completion gate:
- `|position_error| < pos_tolerance` && `|speed_error| < speed_tolerance (30 dps)` held for `done_window_ms (50 ms)` consecutively → `is_done = true`
- `pos_tolerance` is derived from kp_pos (Issue [#140](https://github.com/owhinata/spike-nx/issues/140) Phase 1 F): `max(1000, 400 × 1000 / kp_pos)` mdeg = 8000 mdeg (≈ 4 mm wheel) at kp=50

Per `on_completion` final action:

| variant | post-`is_done` behaviour |
|---|---|
| COAST | motor coast, freeze reference, controller pause |
| BRAKE | motor brake, freeze reference, controller pause |
| HOLD | active hold via PID (not duty=0) |
| CONTINUE | trajectory keeps cruising (no stop) |
| COAST_SMART | hold torque for `smart_passive_hold_time (~100 ms)` then degrade to coast |
| BRAKE_SMART | same with brake |

SMART completion also sets up the next relative-move start judgement (`|prev_endpoint - current_position| < pos_tolerance × 2` → start the next trajectory from `prev_endpoint` = pybricks "continue from endpoint").

### 8.4 L/R aggregation (`drivebase_drivebase.c`)

From the per-servo actual positions:

- `distance = (l_pos + r_pos) / 2 × wheel_circumference / 360`
- `heading_rad = (r_pos - l_pos) × wheel_circumference / (2 × axle_track × π / 360)`

Two PIDs (distance and heading) produce mm/s and deg/s outputs, then the inverse map distributes them into reference speeds for the L/R servos.

**Phase 3b heading PID injection from the IMU (Issue [#148](https://github.com/owhinata/spike-nx/issues/148))**:

`drivebase set-gyro 1d` swaps the heading PID's *state input* from the encoder-derived `(sR-sL)/2` to the Madgwick-fused world-vertical heading.  This is what closes the loop on wheel slip, gear backlash, and motor-to-motor mismatch.  Two-stage latching keeps a race-free guarantee:

- **`status.use_gyro_requested`** — the user-asked mode (`set-gyro` ioctl, or boot-time `use_gyro_plus1` cfg)
- **`status.use_gyro_latched`** — only set at command-start (`db_drivebase_capture_start_heading(do_latch=true)`), and only when `gyro_origin_capturable()` (IMU attached, calibrated, not stale, requested != NONE) is true.  Frozen during the in-flight motion; transient IMU staleness mid-motion just bounces the per-tick injection to encoder fallback without flipping the latched intent.

Mid-motion `set-gyro` returns `-EBUSY` (`status.last_set_gyro_rc=-16`); `set-gyro 3d` returns `-ENOSYS` (`-38`, deferred to Phase 7); invalid values return `-EINVAL` (`-22`).  The kernel chardev envelope queue swallows the daemon's return code, so the propagation channels are the published `last_set_gyro_rc` field and a one-shot `syslog(LOG_WARNING)` line — not the user-facing ioctl rc.

Only the **position state** (`state.heading_x_mdeg`) flips to gyro; the D-term input (`state.heading_v_mdegps`) stays encoder-derived to dodge LSM6DSL quantisation noise.  Matches the pybricks pattern.

The origin baseline `gyro_origin_mdeg` is captured by `set_use_gyro` / `set_origin` / `reset` / start-of-motion latch as `db_imu_get_heading_mdeg() - angle_mdeg`, and the PID input + publish path both subtract it.  So `drivebase reset 0 90` lands `get-state.angle_mdeg ≈ 90000`, and an encoder-mode `turn 90` followed by `set-gyro 1d` keeps the published angle at ≈ 90000 instead of snapping back to 0.

Because the publish path subtracts the same baseline, the PID state and the user-visible heading share a single source of truth (SSOT) across both in-flight motion and idle-rotate scenarios.

### 8.5 Settings (`drivebase_settings.c`)

Defaults targeted at SPIKE Medium Motor (per-axis, aggregated since Issue #141 Phase 2):

| Axis | kp_pos | ki_pos | kd_pos | kp_speed | out_max |
|---|---|---|---|---|---|
| distance | 50 | 15 | 0 | 5 | ±10000 (±100% duty) |
| heading | 50 | 15 | 0 | 5 | ±8000 (±80% duty — leaves L/R compose headroom) |

`ki_pos = 15` is what overcomes the SPIKE Medium Motor's static-friction floor (~12-18% breakaway) under direct duty drive without feed-forward.  The pybricks rule "drivebase aggregate PID uses ki=0" assumes torque output and cannot be adopted here.  Issue #142 Phase 5 Step B confirmed empirically: `ki=10` short-undershoots by ~5 mm on `straight 50` and prevents `turn 180 coast` from ever asserting `done`, so `ki=15` stays optimal until feed-forward arrives in Phase 6.

drive_speed defaults are derived from `wheel_diameter` (`v_max_mdegps`, `accel = v_max × 4 / 1` = reach max speed in 1/4 s).  `drivebase _alg settings` dumps the live values.

#### 8.5.1 Feed-forward (Issue [#152](https://github.com/owhinata/spike-nx/issues/152) Phase 6 Step 6.1)

A linear feed-forward `duty_ff = kV·v_ref + kA·a_ref` is summed into the aggregate per-axis PID (distance / heading).  Port of the WPILib `SimpleMotorFeedforward` shape to duty output, integer-only (FPU stays off-limits outside Madgwick).

- **Injection point**: inside `db_pid_update()`, BEFORE the saturation clamp: `out_unsat += duty_ff`.  A single clamp makes the anti-windup `sat_high/sat_low` flags reflect the FF-inclusive output, so I never accumulates while FF alone is pinning the rail.
- **Unit scaling**: trajectory ref is `mdeg/s` / `mdeg/s²`, gains are `.01% per (deg/s)` / `.01% per (deg/s²)`.  To structurally keep `__aeabi_ldivmod` out of the RT path, the `/1000` conversion runs in int32 BEFORE the multiplication.  With kV/kA bounded to ±1000, `kV × v_dps` peaks at 1.11e6 and `kA × a_dps2` at 8e5 — comfortably inside int32.
- **Per-motor kS friction**: added in Step 6.2 (see below).  L/R compose is linear for kV/kA (per-axis is equivalent to per-motor), but `sign(v) × kS` is non-linear (pivot makes per-axis and per-motor disagree), so kS must apply after the compose.
- **Defaults**: kV=0 / kA=0 = behavioural no-op.  Bench with `ff_dist_kV = 9` reduced `straight 300 brake` settling from 4.6 s to 1.8 s and eliminated the post-target backward swing (anti-windup `i_acc` release artefact) — see #152 bench notes.
- **SysId measurements (Step 6.4)**: `ff_dist_kV ≈ 6` (lower than the plan's seed of 9 — the real motor needs less back-EMF compensation), `ff_dist_kA ≈ 1`.  See §8.5.4 below for the bench data.
- **kA contribution formula**: `peak_accel × kA / 1000` in .01% duty.  With the default distance-axis trajectory accel = 1833 deg/s², `ff_dist_kA=1` adds **18.3% duty during accel/decel phases** (zero in cruise).  The heading-axis default accel is 916 deg/s² so `ff_head_kA=1` adds **9.2%** instead.
- **Per-axis routing**: `ff_dist_*` only affects the distance component of straight / drive_forever / curve commands; `ff_head_*` only affects the heading component of turn / curve.
- **Heading-axis FF stays at 0 in production**: the heading kV has never been bench-validated as beneficial.  Phase 6.1 only added dist kV; turn 90 baseline runs (head kV=0) reached target cleanly.  Step 6.5 tried `head kV=6 + head kA=1` together and saw regression, but cannot separate which gain caused it — kV's independent contribution is unverified, so the safer default is to leave `ff_head_*` at 0 until a dedicated heading-axis SysId / bench run confirms each gain independently.
- **Step 6.5 evaluation — DO NOT enable kA in production**: The SysId-measured `kA=1` looked promising (contribution well above plan OQ1's 2% threshold), but bench on both axes showed regression:
    `straight 300 brake` @ kA=1: dist = 289-295 mm (target 300), `done=0` for the full 3 s observation window.
    `turn 90 brake` @ kA=1: angle = 84-87° (target 90°), `done` late or never.
  Root cause: kA is symmetric across the trajectory accel/decel phases, so the negative `kA × a_ref` during decel subtracts from PID output and stops the motor short of target.  Once stopped, static friction (~25%) traps it; PID alone (ki=10) ramps too slowly to break out before the observation ends.  The Phase 6.2 baseline (kA=0) reaches target cleanly in 1.5–1.8 s.  Per plan OQ1 spirit ("ship a contribution only if the bench confirms it helps"), kA stays at default 0 in production cfg, the verb stays for forward-compat, and Phase 6.6 will revisit kA together with ki / kS so the decel/PID balance can be tuned as one set.

cfg keys (Step 6.1):

| key | default | range | meaning |
|---|---|---|---|
| `ff_dist_kV` | 0 | [-1000, +1000] | distance-axis velocity FF (.01% per deg/s) |
| `ff_dist_kA` | 0 | [-1000, +1000] | distance-axis acceleration FF (.01% per deg/s²) |
| `ff_head_kV` | 0 | [-1000, +1000] | heading-axis velocity FF |
| `ff_head_kA` | 0 | [-1000, +1000] | heading-axis acceleration FF |

#### 8.5.2 Per-motor friction FF (Step 6.2)

The kS friction term applies **per-motor after the L/R compose**.  From the per-axis ref-velocity exports (Step 6.1) we form `vL_ref = D - H` and `vR_ref = D + H`, then add `sign(v) × kS / 2` to each side before the per-side clamp (Plan D1).

- **Why per-motor**: on a pivot `vD = vH > 0`, per-axis distribution would yield `left = kS - kS = 0`, `right = kS + kS = 2 × kS` — double the intended correction on one wheel.  Per-motor evaluation of `sign(vL_ref) / sign(vR_ref)` gives the correct `left = 0, right = kS` instead (Codex Round 1 BLOCKING, captured in the plan).
- **`/2` attenuation**: mirrors pybricks `lib/pbio/src/observer.c:250` (`torque_friction / 2 * sign(rate_ref)`).  During final decel, `v_ref` may stay above the hysteresis `enter` threshold for a while; the half-strength kS keeps the resulting duty step small enough that anti-windup behaves and the move does not overshoot.
- **Sign hysteresis**: a naive `sign(v_ref) × kS` flips by `±2 × kS` (~14% duty for `kS=700`) whenever `v_ref` crosses 0.  Hysteresis state machine:

| condition (top-down) | new sign |
|---|---|
| `v_ref >=  enter` | +1 (commits, overrides held — reverse-direction follow-through) |
| `v_ref <= -enter` | -1 (same) |
| `|v_ref| <  exit` | 0 (kS off, FB's I fills in if needed) |
| else (`exit <= |v_ref| < enter`) | prev (hold) |

Defaults: `enter = 5000 mdeg/s (5 dps)`, `exit = 1000 mdeg/s (1 dps)`.  `db_ff_state_s::sign_v_held` lives per-side on `db_drivebase_s::ff_state_left/right`; `drivebase_drivebase.c`'s compose path updates it each tick.

cfg keys (Step 6.2):

| key | default | range | meaning |
|---|---|---|---|
| `ff_motor_kS` | 0 | [0, 1000] | .01% duty friction (left = right common, `/2` applied) |
| `ff_v_hyst_enter_mdegps` | 5000 | [0, 100000] | hysteresis enter (mdeg/s) |
| `ff_v_hyst_exit_mdegps` | 1000 | [0, 100000] | hysteresis exit (mdeg/s) |

Convention is `enter > exit`, but the setters do not enforce a cross-key invariant (that would make cfg load order load-bearing).  The hysteresis function tolerates any ordering deterministically.

**Sizing kS (important)**: do NOT copy the `kS_nominal` printed by `_sysid ramp-ks` (~2952 = ~30% duty on the reference drivebase) into `ff_motor_kS`.  SysId measures the absolute static-friction duty — the minimum needed to make a stationary motor move on its own.  As an `ff_motor_kS`, that value applies a 15% per-side bias (after `/2` attenuation) every tick, including cruise, shifting the PID operating point and inviting overshoot.  The right `ff_motor_kS` is a SMALL assist that helps `PID + kV + ki` tip past breakaway without disturbing steady-state:
- **`ff_motor_kS = 700`** (Phase 6.2 plan seed, bench-confirmed): 3.5 % per-side rail-step.  Lets `straight 50 brake` hit target exactly with `ki=10 + kV=9`.  This is the production-recommended value.
- **`ff_motor_kS = 2952`** (SysId result): calibration reference only — do NOT set this in cfg.

`drivebase _alg settings` prints `motor ff : kS=… v_hyst=[exit,enter]` for the live values.

#### 8.5.3 Battery sag correction (Step 6.3)

SPIKE Prime Hub's 6S Li-Ion pack sags from ~8.3 V (full) to ~6.0 V (empty), giving the same commanded duty up to a 28 % mechanical output spread.  Phase 6 Step 6.3 corrects for this with a per-motor voltage model:

```
duty_corrected = duty * battery_nominal_mv / max(vbat, battery_min_mv)
```

- **Application point**: post-compose (after PID + kV + kA + kS + 1st clamp) → battery correction → 2nd clamp ±10000 → `db_servo_apply()`.  The stall observer reads `last_applied_duty` set inside `db_servo_apply`, so it automatically sees the post-correction value — no explicit hook change required for the Plan D3 "stall uses corrected duty" guarantee.
- **Polling design**:
  - **Owner**: daemon idle thread (existing 50 ms wake), `(poll_tick & 0x3) == 0` triggers the BATIOC_VOLTAGE ioctl every 4th wake = 200 ms (5 Hz).  RT thread never issues the ioctl.
  - **Transfer**: a single `_Atomic int32_t vbat_mv`.  Cortex-M4 32-bit aligned loads are HW atomic; `memory_order_relaxed` is sufficient (one scalar, no inter-field consistency).
  - **EMA**: daemon-local `prev_ema_mv` with `prev = (prev*7 + now)/8` (τ ≈ 1.6 s).  The atomic is store-only, never RMW.
- **Cold start**: `db_battery_init(nominal_mv)` runs right after `db_settings_freeze()` and before `db_rt_start()`, so the first few RT ticks (before any poll completes) see ×1 correction.
- **Low-V cap**: `battery_min_mv = 6000` clamps the divisor at 6 V → 1.2× maximum boost.  Protects against gauge under-reads producing explosive correction.
- **RT-path 32-bit divide**: `duty * nominal_mv` peaks at `10000 * 12000 = 1.2e8` (int32-safe); the `/ vbat` becomes a hardware 32-bit divide (`__aeabi_uidiv` ~12 cycles), not the 64-bit `__aeabi_ldivmod`.

cfg keys (Step 6.3):

| key | default | range | meaning |
|---|---|---|---|
| `battery_nominal_mv` | 7200 | [1, 12000] mV | reference voltage (vbat at which gains were tuned) |
| `battery_min_mv` | 6000 | [1, 12000] mV | divisor floor (default 1.2× cap) |

Convention is `min_mv < nominal_mv` but setters do not cross-validate (cfg load order would otherwise become load-bearing).

`drivebase _alg settings` prints `battery : vbat=… nominal=… min=…` for the live snapshot.

#### 8.5.4 SysId CLI (`drivebase _sysid`, Step 6.4)

Open-loop measurement of the Phase 6 FF gains (kS, kV, kA) without the closed-loop PID.  Each verb bypasses `db_aggregate` / `db_servo_apply` and drives motors directly via `drivebase_motor_set_duty()`, reading speed from a side-local `db_observer` instance built from raw encoder samples drained on the CLI task.

Every verb queries `DRIVEBASE_GET_STATUS` at entry and **rejects with `-EBUSY`** if `daemon_attached==1` (the production RT thread would otherwise overwrite our duty commands every 2 ms).  Every exit path coasts both motors.

**Ground-only contract (Plan D8, repeated in the help string)**:
- The drivebase must be on a flat surface with ~2 m of clear straight space.  Free-spin (wheels-up) measurements under-fit because they don't include floor friction or chassis inertia; kA cannot be measured at all.
- Keep Ctrl-C ready as a manual stop.
- **If your terminal swallows Ctrl-C (e.g. `picocom`)**: run the verb in the background with `&` and stop it via `kill -2 <pid>` — `-2` is **SIGINT**, which is what the handler is registered for.  Plain `kill <pid>` defaults to SIGKILL on most NuttX builds, which bypasses the handler entirely and leaves motors at whatever duty was last commanded.  SIGTERM is not caught either, so always specify `-2`.
- Re-run at different battery voltages to confirm the nominal-normalised gains converge.

**Verb order is fixed**: `ramp-ks → ramp-kv → ramp-ka`.  ramp-kv subtracts the kS measured by ramp-ks; ramp-ka uses the kV measured by ramp-kv.  Running them out of order produces biased gains.

| verb | arguments | output |
|---|---|---|
| `_sysid ramp-ks <step> [max] [hold_ms]` | step duty in .01%, max (default 2500 = 25%), hold (default 200 ms) | per-step vL/vR CSV + breakaway duty `kS_L / kS_R / max` + vbat-normalised `kS_nominal` + cfg-line summary |
| `_sysid ramp-kv <max> <duration_ms> <kS_subtract>` | peak duty, total time for both directions, kS from ramp-ks | 22 samples (forward 11 + reverse 11) CSV + per-side kV fit + average + nominal + cfg-line summary |
| `_sysid ramp-ka <duty> <duration_ms> <kV>` | step duty, observation window, kV from ramp-kv | 25 ms-spaced v_avg trace + `v_steady_avg` + 63% rise `τ_ms` + `kA = kV*τ/1000` + cfg-line summary |
| `_sysid vbat` | (none) | one-line `vbat=… mV nominal=… mV ratio=…/1000` |

**vbat normalisation direction (Plan D8 Codex Round 2 BLOCKING)**:
A measurement at low vbat OVERESTIMATES the gain (more duty was needed to hit the same physical effect).  To rebase to the nominal voltage:
```
gain_nominal = gain_measured * vbat_mv / battery_nominal_mv
```
i.e. MULTIPLY by `vbat / nominal` (a number < 1 when below nominal).  Both raw and normalised values are printed alongside vbat.  Rounding is symmetric to avoid systematic truncation bias.

**ramp-kv fit**:
- `kS_subtract` argument removes the friction term: `u_eff = duty - sign(v_mdegps) * kS_subtract`
- Samples below breakaway (`|v_mdegps| < 30000`) are excluded from the fit
- Least-squares: `kV = Σ(u_eff * v_dps) / Σ(v_dps²)` with `v_dps = v_mdegps / 1000`
- Forward + reverse pooled into one fit per side, then L/R averaged
- int64 accumulators absorb worst-case sums while keeping the final result int32

**ramp-ka fit**:
- The step duty runs for `duration_ms`, with v_avg sampled every 25 ms
- `v_steady` = average of the final 25 % of samples
- τ = time when v_avg first reaches 63.2 % of v_steady (matches the first-order plant model the FF implicitly assumes)
- `kA = kV * τ_ms / 1000` (ms → s), result in `.01% per (deg/s²)`

**Output protocol**: print-only; the operator copies the suggested cfg line into `/mnt/flash/drivebase.cfg`.  An auto-write surface is a separate Issue's responsibility.

**No RT-path impact**: SysId runs only while the daemon is stopped, so its math never executes from the RT tick.  `drivebase_sysid.o` does contain 64-bit divides in the fit helpers, but it is outside the RT path and excluded from the CoreMark gate (Plan note).  RT-path object 64-bit divide counts are unchanged from Step 6.3 (aggregate.o=0, control.o=5, drivebase.o=13, rt.o=3, battery.o=0).

### 8.6 Runtime config override (`/mnt/flash/drivebase.cfg`, Issue [#143](https://github.com/owhinata/spike-nx/issues/143))

At `drivebase start` the daemon reads a text file on the external W25Q256 NOR (LittleFS at `/mnt/flash`) and applies any recognised key=value pairs to PID gains / completion / stall / wheel/axle/tick before launching the RT thread.  This shortens bench-tuning iteration: no rebuild required.

**Format**: properties (key=value).  A `#` starts a comment, either at the beginning of a line or trailing on a value line (everything from the first `#` on the right-hand side of `=` is dropped before integer parsing).  Blank lines and leading/trailing whitespace are stripped.  Accepts both LF and CRLF line endings and an optional leading UTF-8 BOM.  Line length cap 256 bytes (overlong lines warn and skip).  Unknown keys / parse errors log `LOG_WARNING` and **skip that one line**; remaining keys apply.

**Reliability**:
- File missing (`ENOENT`) → silent fallback (no dmesg entry)
- Mount failure / other I/O error → `LOG_WARNING`
- At least one key applied → `LOG_INFO`: `drivebase: loaded N config key(s) from /mnt/flash/drivebase.cfg`
- File present but zero keys applied → `LOG_WARNING` (surfaces typos)

**Race protection**: `db_settings_freeze()` is called right before `db_chardev_handler_attach()`, so any write attempt after the RT thread starts returns `-EBUSY`.  Reads from the RT tick are race-free by construction.

**Supported keys** (compiled defaults shown in parentheses):

| Group | Key | Type | Range / meaning |
|---|---|---|---|
| PID dist | `pid_dist_kp_pos` (50) | int32 | (0, 10000] |
| | `pid_dist_ki_pos` (15) | int32 | [0, 10000] |
| | `pid_dist_kd_pos` (0) | int32 | [0, 10000] |
| | `pid_dist_kp_speed` (5) | int32 | [0, 10000] |
| | `pid_dist_ki_speed` (0) | int32 | [0, 10000] |
| | `pid_dist_deadband_mdeg` (3000) | int32 | [0, 100000] mdeg |
| | `pid_dist_out_max` (10000) | int32 | (0, 10000]; out_min mirrors |
| PID head | `pid_head_*` | as above, `out_max` default 8000 |
| Completion | `comp_dist_pos_tol_mdeg` (-1) | int32 | -1 = derive from kp_pos (#140), else [1000, 60000] |
| | `comp_dist_speed_tol_mdegps` (30000) | int32 | [0, 1000000] |
| | `comp_dist_smart_continue_mdeg` (-1) | int32 | -1 = default 6000, else [0, 60000] |
| | `comp_dist_done_window_ms` (50) | uint32 | [0, 10000] |
| | `comp_dist_smart_passive_hold_ms` (100) | uint32 | [0, 10000] |
| | `comp_head_*` | mirrors |
| FF | `ff_dist_kV` (0) | int32 | [-1000, +1000] .01% per (deg/s), distance axis (Step 6.1) |
| | `ff_dist_kA` (0) | int32 | [-1000, +1000] .01% per (deg/s²), distance axis |
| | `ff_head_kV` (0) | int32 | [-1000, +1000] .01% per (deg/s), heading axis |
| | `ff_head_kA` (0) | int32 | [-1000, +1000] .01% per (deg/s²), heading axis |
| | `ff_motor_kS` (0) | int32 | [0, 1000] .01% duty friction, left=right (Step 6.2) |
| | `ff_v_hyst_enter_mdegps` (5000) | int32 | [0, 100000] hysteresis enter |
| | `ff_v_hyst_exit_mdegps` (1000) | int32 | [0, 100000] hysteresis exit (convention enter > exit) |
| Battery | `battery_nominal_mv` (7200) | int32 | [1, 12000] mV — correction reference voltage (Step 6.3) |
| | `battery_min_mv` (6000) | int32 | [1, 12000] mV — divisor floor (default 1.2× cap, convention min < nominal) |
| Stall | `stall_speed_mdegps` (30000) | int32 | [0, 1000000] |
| | `stall_duty_min` (6000) | int32 | [0, 10000] |
| | `stall_window_ms` (200) | uint32 | [0, 10000] |
| Start | `wheel_d_um` (56000) | uint32 | > 0, used only when CLI omits wheel |
| | `axle_t_um` (112000) | uint32 | > 0, used only when CLI omits axle |
| | `tick_us` (DB_RT_TICK_US_DEFAULT) | uint32 | > 0, used only when CLI omits tick |
| | `use_gyro_plus1` (0 = unset) | uint8 | Phase 3b (#148): 0 = key absent ⇒ `NONE`, 1 = `NONE`, 2 = `1D`.  The daemon calls `db_drivebase_set_use_gyro` once at startup; Madgwick has not converged yet so the `latched` transition is deferred to the first command. |

**Wheel / axle / tick precedence**: `drivebase start [wheel_mm] [axle_mm] [tick_ms]` decides "explicit vs omitted" by `argc`.  Explicit CLI > config > compiled default.

**On-device editing**: defconfig enables `CONFIG_SYSTEM_VI=y` + `CONFIG_SYSTEM_TERMCURSES=y`, so:

```
nsh> vi /mnt/flash/drivebase.cfg
```

For one-off appends, `echo` + redirect also works:

```
nsh> echo "pid_dist_ki_pos = 12" >> /mnt/flash/drivebase.cfg
nsh> drivebase stop
nsh> drivebase start
nsh> drivebase _alg settings    # confirm the applied values
```

**Example** (`/mnt/flash/drivebase.cfg`):

```
# Phase 5 Step B re-sweep: temporarily lower the integrator gain
pid_dist_ki_pos = 10
pid_head_ki_pos = 10

# Non-default chassis geometry
wheel_d_um = 62000
axle_t_um = 145000
```

**Full template**: a fully annotated cfg listing all 39 supported keys (Phase 6 Step 6.1 added 4 linear FF keys; Step 6.2 added 3 per-motor friction keys; Step 6.3 added 2 battery sag keys) at their compiled defaults is checked in at `apps/drivebase/drivebase.cfg.sample`.  Either zmodem it onto the hub or paste its contents into `vi /mnt/flash/drivebase.cfg`.  When iterating, keep only the keys you are tuning and comment out the rest — that keeps `dmesg` informative about what changed.

## 9. Linux portability (FUSE)

The kernel-chardev + IPC design carries over cleanly to a Linux port using FUSE (`/dev/fuse` + libfuse):

- **NuttX build**: kernel chardev sits in front of VFS; daemon woken via cmd_ring + sem_post
- **Linux build**: a FUSE filesystem implements the same ioctl numbers + structs through `FUSE_IOCTL` callbacks; the daemon is invoked directly as a callback
- **Only the IPC layer differs**: cmd_ring + state_db bridge code (~250 lines kernel + DAEMON_ATTACH/PICKUP/PUBLISH ioctls) goes away.  The daemon's chardev_handler logic runs unmodified.
- ABI structs and user-facing ioctl numbers remain bit-identical between the two builds.

To make this work, the ABI rules are:

- Fixed-width types only (`uint32_t`, `int32_t`, `uint64_t`, `uint8_t`); `long` / pointers / `time_t` / `bool` are forbidden in struct fields
- `_Static_assert(sizeof(struct ...) == N)` and `_Static_assert(offsetof(...) == K)` lock every public struct
- No variable-length structs; no embedded pointers

**STOP latency guarantees are environment-specific:**

- NuttX build: kernel emergency_cb in < 100 µs typical
- Linux/FUSE build: bound by the userspace scheduler — equivalent guarantees are not portable (FUSE callbacks must round-trip through user space).  Linux ports needing the same SLA must arrange a separate path (cgroup priority / RT scheduler / kernel-module bypass).

## 10. Troubleshooting

### 10.1 `drivebase status` reports "command not found"

See §7.  Root cause is `CONFIG_TLS_ALIGNED=y`.  Add `# CONFIG_TLS_ALIGNED is not set` to the defconfig, then `make nuttx-distclean && make`.

### 10.2 Daemon never posts teardown_done after start

Likely both motors aren't plugged in (one odd port + one even port required).  `drivebase_motor_init` sees `LEGOSENSOR_GET_INFO` return type_id != 48 and short-circuits to `fail` without posting `teardown_done`.

Confirm `lump: port X: SYNCED type=48` for both ports in dmesg.

### 10.3 Gyro features (`set-gyro 1d`) have no effect

Triage order:

1. Check `drivebase status` `imu_present`.
   - `0` → LSM6DSL boot init failed.  Look for `ERROR: Failed to initialize LSM6DSL: -110` in dmesg — check I2C2 wiring / pull-ups / boot ordering.  The daemon is best-effort, so encoder-only operation continues.
   - `1` → IMU is healthy; continue.
2. Check `drivebase status` `last_set_gyro_rc`.
   - `0` → accepted, but `latched` may not have transitioned at command-start (see next step)
   - `-16 (EBUSY)` → `set-gyro` was issued mid-motion.  `drivebase stop-motion coast` first, then retry
   - `-38 (ENOSYS)` → `set-gyro 3d` is not supported until Phase 7
   - `-22 (EINVAL)` → unknown mode
3. `use_gyro_requested == 1` but `use_gyro_latched == 0` after issuing a motion:
   - Madgwick not converged (just after cold boot).  Confirm `drivebase _imu show` reports `madgwick.initialized=1` + `calibrated=1`.  Needs ≥ 200 ms of stillness.
   - IMU stale (`db_imu_is_stale` true).  `_imu drift 5` to confirm samples are arriving.
4. `drivebase get-state` `angle_mdeg` does not turn gyro-origin-relative:
   - `use_gyro_requested == 1` and the internal `gyro_origin_valid` must both hold.  If `set-gyro 1d` ran before IMU calibration, the origin never landed — try `drivebase reset 0 0` to re-snapshot.

### 10.4 Stall watchdog fires spuriously

Five consecutive deadline misses route through coast.  Inspect `drivebase jitter`:

- Sustained samples in bucket 5+ (≥ 1 ms) suggest contention with BTstack / sound DMA / flash I/O
- `apps/btsensor/btsensor stop` to silence Bluetooth and re-test
- `apps/sound/sound stop` to silence the DAC and re-test

### 10.5 Daemon refuses to come up after several start/stop cycles

Confirm `attach_generation` is incrementing in `GET_STATUS`.  If kernel chardev close-cleanup didn't run and `attached` got stuck at true, the next ATTACH returns `-EBUSY`.  A `reboot` resets the chardev cleanly.

## 11. References

- Hardware ledger: `docs/en/hardware/dma-irq.md` (TIM/DMA/NVIC ownership)
- LUMP protocol: `docs/en/drivers/lump-protocol.md`
- LEGOSENSOR ABI: `docs/en/drivers/sensor.md` (motor_l/m/r class topics, SET_PWM / COAST / BRAKE)
- IMU drain pattern: `docs/en/drivers/imu.md` (sensor_imu0 uORB)
- Algorithm reference (do not vendor): `pybricks/lib/pbio/src/{drivebase,servo,trajectory,observer,control,integrator,control_settings}.c`
