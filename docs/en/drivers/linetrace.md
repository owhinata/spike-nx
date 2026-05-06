# linetrace (Issue #107)

User-space NSH builtin that follows a line by issuing `DRIVEBASE_DRIVE_FOREVER` continuously while reading reflected-light samples from the LEGO color sensor.  Modeled on the canonical pybricks pattern (`pybricks/tests/motors/drivebase_line.py`): a tight `err = target - reflection(); drive(speed, err * Kp); wait(...)` loop that lives entirely in user space.

## 1. Why a separate app

pybricks does not provide a dedicated line-trace API.  Their canonical example is a 5 ms MicroPython script that reads `ColorSensor.reflection()` and calls `DriveBase.drive(speed, turn_rate)` in a loop.  spike-nx mirrors that philosophy: the C-layer drivebase daemon (Issue #77) owns the 5 ms RT control loop but stays line-trace-agnostic, and the line-trace policy lives in a thin user-space CLI on top of the existing `DRIVEBASE_DRIVE_FOREVER` ioctl.

| layer | responsibility |
|---|---|
| `apps/drivebase/` daemon (5 ms RT, prio 220) | trapezoidal velocity profile, encoder/IMU fusion, PWM output |
| kernel `/dev/drivebase` chardev | command ring, state double-buffer, emergency stop |
| **`apps/linetrace/` (this app)** | reads color sensor, computes turn rate, re-issues `DRIVE_FOREVER` at 100 Hz |

The daemon's command ring (depth 8) absorbs the rapid re-issue: each new `DRIVE_FOREVER` envelope simply overwrites the next-tick target, so the most recent value is what the 5 ms tick consumes.  No new ioctl ABI is added.

## 2. Subcommands

```
linetrace cal                                   # 3 s sample sweep
linetrace run <speed_mmps> <kp> [target] \      # main command
              [--max-turn dps] [--hz N]
linetrace                                       # usage
```

Defaults: `target=50`, `--max-turn=180`, `--hz=100`.  `kp` is fractional (e.g. `linetrace run 100 1.5`).

### 2.1 `linetrace cal`

Opens `/dev/uorb/sensor_color`, CLAIM, SELECT mode 1 (REFLT), reads samples for 3 seconds while the operator sweeps the sensor over the line and the surrounding floor.  Prints the observed min / max / midpoint:

```
nsh> linetrace cal
linetrace: sampling for 3000 ms — sweep the sensor over black/white
linetrace: cal 250 samples: black=4 white=72 midpoint=38
           suggested: linetrace run <speed_mmps> <kp> 38
```

### 2.2 `linetrace run`

1. Opens `/dev/drivebase`, calls `DRIVEBASE_GET_STATE`.  If `active_command != DRIVEBASE_ACTIVE_NONE` (some other drive verb is in flight), refuses to start.
2. Opens `/dev/uorb/sensor_color`, CLAIM, SELECT mode 1 (REFLT).
3. Installs a SIGINT handler.
4. Loop at `--hz` (default 100 Hz, `clock_nanosleep CLOCK_MONOTONIC TIMER_ABSTIME` so cumulative drift is zero):
   - Drain the color sensor non-blocking, keep the latest reflection value
   - `err = target - reflection`
   - `turn_dps = clamp(kp * err, ±max_turn)`
   - `ioctl(DRIVEBASE_DRIVE_FOREVER, {speed_mmps, turn_dps})`
5. On SIGINT: `ioctl(DRIVEBASE_STOP, {COAST})`, close fds, exit.

### 2.3 `linetrace target <N>` / `linetrace max_turn <DPS>` (Issue #119)

Single-purpose subcommands that mutate `target` / `max_turn` without engaging the drivebase.  Lets the user set the operational reflectance threshold while still idle, so `linetrace status` shows last_err against the real target during robot positioning.  `max_turn` is also usable as live-tuning during a `run`: changing it makes the next tick re-derive the anti-windup clamp from the new value.

```
linetrace target 38            # apply the cal midpoint
linetrace max_turn 120         # tighter output cap for aggressive damping
```

Ranges: `target ∈ [0, 100]`, `max_turn ∈ [1, 1000]`.  Missing / extra / non-integer arguments are usage errors; query the current values with `linetrace status`.

### 2.4 `linetrace pidstat` (Issue #118)

Observation command for tuning PID gains (Kp/Ki/Kd) on real hardware.  Mirrors drivebase `get-state` (Issue #115): a header line followed by one row per interval streaming the PID internals.

```
linetrace pidstat                       # one snapshot row
linetrace pidstat 5000                  # 5 s @ 1 Hz (≈5 rows + summary)
linetrace pidstat 5000 100              # 5 s @ 100 ms intervals (≈50 rows + summary)
```

Default `interval_ms = 1000` (1 Hz).  Shorter intervals load printf/USB-CDC onto the 100 Hz daemon and inject control jitter — keep ≥1000 ms unless you have a specific reason.  `interval_ms < 1000/hz` (below the control period) is rejected.

#### Columns (14)

```
 time_ms      iter   refl     err   err_min   err_max  err_avg    zc    d_max    d_avg      i_acc  turn_max  turn_avg    sat
```

| Column | Kind | Meaning |
|---|---|---|
| `time_ms` | snapshot | Elapsed since pidstat start (graph x-axis) |
| `iter` | snapshot | Daemon control-tick counter (uint32, wraps ≈497 d) |
| `refl` | snapshot | Color sensor reflection (0–100) |
| `err` | snapshot | `target - refl` |
| `err_min/max` | interval agg | Min/max err over preceding interval (100 Hz ticks).  **Aliasing defense** |
| `err_avg` | interval agg | **mean(\|err\|) × 10** (0.1 fixed-point).  IAE-style tracking quality |
| `zc` | interval agg | Zero-crossings of err sign in the interval |
| `d_max` | interval agg | max(\|d_term\|).  Spike amplitude under high Kd |
| `d_avg` | interval agg | mean(\|d_term\|).  `d_max/d_avg` ratio reveals noise character |
| `i_acc` | snapshot | Integrator accumulator (slow signal, snapshot suffices) |
| `turn_max/avg` | interval agg | max/mean(\|turn_dps\|).  Output amplitude |
| `sat` | cumulative delta | Saturation hits (clamp actually acted) since pidstat start |

The 7 aggregate columns (err_min/max/avg, d_max/avg, turn_max/avg) print `-` when `interval_tick_count == 0` (idle entry, no ticks yet).

#### Summary line

```
# pidstat: sat=N iter=B..E duration_ms=M reported_ticks=R expected=X
```

`reported_ticks` sums tick counts of *printed* intervals; `expected = duration_ms × hz / 1000`.  The difference distinguishes daemon-stop, dropped trailing partial interval, and execution jitter / missed ticks.  Snapshot mode (`duration_ms = 0`) suppresses the summary.

#### Operational notes

- **Do not issue `linetrace run` / `brake` while pidstat is running.**  A 1-interval engaged/idle straddle dilutes d_avg/turn_avg and is not flagged.
- Use `linetrace status` for a single-shot scalar peek; use `pidstat` for time-series tuning.  Issue #118 dropped `last_p_term`/`last_i_term`/`last_d_term`/`last_turn_dps` from `status` and added `last_i_acc` instead.

## 3. Tuning

PID gains are tuned by reading the `linetrace pidstat` columns in real time:

| Symptom | Watch | Action |
|---|---|---|
| Oscillation (Kp too high) | `zc` rising + wide `err_min/err_max` | Lower Kp |
| Integral wind-up (Ki too high) | `i_acc` pinned at the anti-windup clamp | Lower Ki |
| Derivative noise (Kd too high) | `d_max / d_avg` ratio large (burst noise) | Lower Kd or add LPF |
| Output saturation (max_turn too low) | `sat` rising + `turn_max == max_turn` | Increase `--max-turn` |
| Tracking-quality compare | `err_avg` (IAE-like) | Smaller is better between gain sets |

Host-side log + plot example (skip the `#` summary line):

```bash
picocom -t '!' /dev/ttyACM0 | tee pidstat.log
# In another shell:
awk '!/^#/ && NR>1 {print $1, $4}' pidstat.log | gnuplot -p -e \
    "plot '<cat' using 1:2 with lines title 'err'"
# Column 4=err, 5=err_min, 6=err_max, 7=err_avg(×10), 8=zc,
# 9=d_max, 10=d_avg, 13=turn_avg, 14=sat
```

### 3.1 Core knobs

- `target`: midpoint reflection between line (black) and floor (white).  Use `linetrace cal` first.
- `kp`: start at 1.0 and double until the robot oscillates, then back off ~30 %.  Typical values 1.5–4.0.
- `--max-turn`: caps the turn rate so a brief lost-line spike (high `err`) cannot fling the robot.  Default 180 dps is safe for a 56 mm-wheel chassis.
- `--hz`: the LPF2 color sensor publishes at ≈100 Hz; loops faster than that just re-issue the same sample.  Slower (e.g. 50 Hz) reduces CPU but adds latency.

## 4. Wiring

- Color sensor: any port (LPF2 type 61 auto-detected).  Mounted forward-facing about 10 mm above the floor.
- Motors: standard SPIKE Prime drivebase (motor on port A or C as L, B or D as R).  Configure once with `drivebase config <wheel_diameter_mm> <axle_track_mm>` before `linetrace run`.

## 5. Out of scope (future Issues)

- D term — current loop is P-only.  Add if oscillation / overshoot becomes problematic at higher speeds.
- Line-loss detection / search — the loop blindly drives at the most recent reflection forever.  pybricks does not handle this either; add a watchdog (no fresh sample within N ms → coast) when needed.
- Gain scheduling — at higher speeds `kp` should fall to keep the same closed-loop bandwidth; current CLI is constant-gain.
- Junction recognition (T / cross intersections) — needs color classification + a state machine.
- IMU heading correction — would conflict with color-driven steering; this app uses color alone.

## 6. References

- pybricks canonical pattern: `pybricks/tests/motors/drivebase_line.py`
- drivebase ABI: [drivebase.md](drivebase.md), `boards/spike-prime-hub/include/board_drivebase.h`
- color sensor (LPF2 type 61, REFLT mode): [sensor.md](sensor.md), `boards/spike-prime-hub/include/board_legosensor.h`
