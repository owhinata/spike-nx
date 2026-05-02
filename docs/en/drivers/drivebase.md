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

struct drivebase_config_s          { uint32_t wheel_diameter_mm; uint32_t axle_track_mm; uint8_t r[8]; };
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
| `drivebase start [wheel_mm] [axle_mm]` | spawn the daemon.  Default wheel=56, axle=112 (SPIKE driving base) |
| `drivebase stop` | graceful daemon teardown (returns within ~2 s) |
| `drivebase config <wheel_mm> <axle_mm>` | DRIVEBASE_CONFIG (currently optional — daemon already takes wheel/axle from `start`) |
| `drivebase straight <mm> [coast\|brake\|hold]` | DRIVE_STRAIGHT |
| `drivebase turn <deg>` | TURN (CCW positive) |
| `drivebase forever <mmps> <dps>` | DRIVE_FOREVER (no completion, distance + heading concurrently) |
| `drivebase stop-motion <coast\|brake\|hold>` | DRIVEBASE_STOP (emergency fast path) |
| `drivebase get-state` | DRIVEBASE_GET_STATE (distance / speed / heading / done / stall) |
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

Trapezoidal profile (accel / cruise / decel).  Short moves degenerate to a triangular fallback (cruise=0) with `v_peak` recomputed via integer sqrt.

`db_trajectory_init_position(t0, x0, x1, v_peak, accel, decel)` plans the move; `db_trajectory_get_reference(tr, t)` returns `(x_mdeg, v_mdegps, a_mdegps2, done)`.

### 8.3 PID + completion (`drivebase_control.c`)

Cascade PID (position P+I+D feeding velocity P+I).  Anti-windup:
- Position error inside the deadband (±3000 mdeg = ±3°) → freeze the integrator
- Output saturated → clamp the integrator's accumulation direction

Completion gate:
- `|position_error| < pos_tolerance (3000 mdeg)` && `|speed_error| < speed_tolerance (30 dps)` held for `done_window_ms (50 ms)` consecutively → `is_done = true`

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

Two PIDs (distance and heading) produce mm/s and deg/s outputs, then the inverse map distributes them into reference speeds for the L/R servos.  When gyro is enabled the heading actual is overridden with the gyro estimate (rejecting wheel slip).

### 8.5 Settings (`drivebase_settings.c`)

Defaults targeted at SPIKE Medium Motor (position P=50 / I=20 / D=0, velocity P=5 / I=0).  drive_speed defaults are derived from `wheel_diameter` (`v_max_mdegps`, `accel = v_max × 4 / 1` = reach max speed in 1/4 s).  `drivebase _alg settings` dumps the live values.

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

`/dev/uorb/sensor_imu0` may not exist if LSM6DSL boot init failed.  Look for `ERROR: Failed to initialize LSM6DSL: -110` in dmesg — check the I2C2 wiring / pull-ups / boot ordering.  The daemon is best-effort, so encoder-only operation continues with `imu_present=0` reported in `drivebase status`.

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
