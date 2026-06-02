# linetrace (Issue #107)

User-space NSH builtin that follows a line by issuing `DRIVEBASE_DRIVE_FOREVER` continuously while reading the LEGO color sensor's RGB+I intensity channel (mode 5, INT16 channel 4, 0..1024).  Modeled on the canonical pybricks pattern (`pybricks/tests/motors/drivebase_line.py`): a tight `err = target - intensity(); drive(speed, err * Kp); wait(...)` loop that lives entirely in user space.  Issue #125 swapped the input source from mode 1 (REFLT, INT8 0..100) to mode 5 because the higher-resolution intensity channel removes the 1-LSB jitter Reflection produced near the line edge.

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
linetrace run [speed_mmps] [kp] [target] \      # main command
              [--kp K] [--ki K] [--kd K] [--hz N] \
              [--v-min mm/s] [--v-alpha A] [--v-beta B] \
              [--edge left|right]
linetrace edge left|right                       # select followed edge
linetrace                                       # usage
```

Defaults: `target=512`, `--hz=100` (or whatever `/mnt/flash/linetrace.cfg` overrides — see §3.3).  `speed_mmps`/`kp`/`target` are **optional positionals** (Issue #181): an omitted one inherits the current value (the config overlay applied at `start`, or the prior run).  A bare `linetrace run` therefore re-engages at the persisted/last tuning; with nothing configured it resolves to `speed=0 kp=0`, the same active-hold as the long-standing `run 0 0` idiom.  Because a leading `-0.3` parses as an option rather than a positional, a **negative** gain must be passed via `--kp` (symmetric with `--ki`/`--kd`); a positional `kp` and `--kp` together let the flag win.  `kp/ki/kd` are fractional (e.g. `linetrace run 100 0.3`).  Intensity-era kp values are roughly **1/10 of the reflection-era values** because err magnitudes are ~10× larger (range 0..1024 vs 0..100); start at ~0.15 instead of 1.5.

The `--max-turn` flag and `linetrace max_turn N` subcommand were **retired in Issue #126**.  `max_turn` is now a per-tick derived value that tracks `speed_apply` (Issue #121's empirical `max_turn ≈ v` rule baked into the controller).

`--v-min/--v-alpha/--v-beta` enable dynamic speed control (Issue #126).  These flags are an opt-in safety switch and reset on every `run` — to the `linetrace.cfg` value if one is set (§3.3), otherwise to "dynamic OFF" (`v_min := speed`, `α := β := 0`).  See §3.2 for details.

### 2.1 `linetrace cal`

Opens `/dev/uorb/sensor_color`, CLAIM, SELECT mode 5 (RGB+I; the loop reads the 4th INT16 channel = intensity, 0..1024), reads samples for 3 seconds while the operator sweeps the sensor over the line and the surrounding floor.  Prints the observed min / max / midpoint:

```
nsh> linetrace cal
linetrace: sampling for 3000 ms — sweep the sensor over dark/bright
linetrace: cal 250 samples: dark=42 bright=982 midpoint=512
           suggested: linetrace run <speed_mmps> <kp> 512
```

### 2.2 `linetrace run`

1. Opens `/dev/drivebase`, calls `DRIVEBASE_GET_STATE`.  If `active_command != DRIVEBASE_ACTIVE_NONE` (some other drive verb is in flight), refuses to start.
2. Opens `/dev/uorb/sensor_color`, CLAIM, SELECT mode 5 (RGB+I).  Sample reads gate on `mode_id == 5 && data_type == LUMP_DATA_INT16 && num_values >= 4 && len >= 8` so a short DATA frame cannot leak a zero-filled `i16[3]` as a valid intensity reading.
3. Installs a SIGINT handler.
4. Loop at `--hz` (default 100 Hz, `clock_nanosleep CLOCK_MONOTONIC TIMER_ABSTIME` so cumulative drift is zero):
   - Drain the color sensor non-blocking, keep the latest intensity value (`s.data.i16[3]`, clamped to `[0, 1024]`)
   - `err = target - intensity`
   - `turn_dps = clamp(edge_sign * (p + i + d), ±max_turn)` where `edge_sign = +1` for the LEFT edge (default), `-1` for RIGHT — see §2.4
   - `ioctl(DRIVEBASE_DRIVE_FOREVER, {speed_mmps, turn_dps})`
5. On SIGINT: `ioctl(DRIVEBASE_STOP, {COAST})`, close fds, exit.

### 2.3 `linetrace target <N>` (Issue #119)

Single-purpose subcommand that mutates `target` without engaging the drivebase.  Lets the user set the operational intensity threshold while still idle, so `linetrace status` shows last_err against the real target during robot positioning.

```
linetrace target 512           # apply the cal midpoint
```

Range: `target ∈ [0, 1024]`.  Missing / extra / non-integer arguments are usage errors; query the current value with `linetrace status`.

> The `linetrace max_turn N` mutator that originally shipped with #119 was retired in #126: `max_turn` is now derived per tick from `speed_apply` and no longer needs a CLI knob.

### 2.4 `linetrace edge left|right` (Issue #180)

Selects which edge of the line the robot follows.  The optical intensity slope `c` has the opposite sign on the two edges, so the steering polarity must flip between them.

- **`left`** is the default and the legacy behavior — the robot travels on the **left** side of the line.  `right` negates the steering output.
- The sign is applied only to the controller **output**: `turn_dps = clamp(edge_sign * (p + i + d), ±max_turn)`.  `err = target - intensity`, the integrator `i_acc`, and all `pidstat` aggregates stay in the pure measurement domain, so the same gains and the same tuning columns mean the same thing on either edge — only the commanded turn direction reverses.
- Set it two ways, mirroring `target`:

```
linetrace edge right                       # mutate without engaging
linetrace run 100 0.3 512 --edge right     # set as part of a run
```

- **Sticky**: the edge inherits across `run` invocations (like `kp/ki/kd/target/hz`) and resets to `left` only on `linetrace start`.  Query it with `linetrace status` (`edge: left|right`).
- **Live switch caveat**: `linetrace edge` does not reset the PID state.  Flipping the edge mid-run instantly reverses the integrated steering authority (`i_acc`), producing a brief counter-steer transient before the loop re-settles.  This is intended; switch edges while stopped if you want a clean transition.

Invalid tokens (anything other than `left`/`right`) and wrong argument counts are usage errors.

> Intersection branching — where the selected edge decides which way to go at a junction — is **not** implemented here; #180 only establishes the edge parameter as the foundation.  Persisting the edge (and the other tuning) across reboots is implemented in #181 — see §3.3.

### 2.5 `linetrace pidstat` (Issue #118)

Observation command for tuning PID gains (Kp/Ki/Kd) on real hardware.  Mirrors drivebase `get-state` (Issue #115): a header line followed by one row per interval streaming the PID internals.

```
linetrace pidstat                       # one snapshot row
linetrace pidstat 5000                  # 5 s @ 1 Hz (≈5 rows + summary)
linetrace pidstat 5000 100              # 5 s @ 100 ms intervals (≈50 rows + summary)
```

Default `interval_ms = 1000` (1 Hz).  Shorter intervals load printf/USB-CDC onto the 100 Hz daemon and inject control jitter — keep ≥1000 ms unless you have a specific reason.  `interval_ms < 1000/hz` (below the control period) is rejected.

#### Columns (17, with v_max/v_avg/v_min added in Issue #126)

```
 time_ms      iter intens     err   err_min   err_max  err_avg    zc    d_max    d_avg      i_acc  turn_max  turn_avg   v_max   v_avg   v_min    sat
```

| Column | Kind | Meaning |
|---|---|---|
| `time_ms` | snapshot | Elapsed since pidstat start (graph x-axis) |
| `iter` | snapshot | Daemon control-tick counter (uint32, wraps ≈497 d) |
| `intens` | snapshot | Color sensor RGB+I intensity channel (0–1024); short for "intensity", header width matches the `%6d` data column |
| `err` | snapshot | `target - intens` |
| `err_min/max` | interval agg | Min/max err over preceding interval (100 Hz ticks).  **Aliasing defense** |
| `err_avg` | interval agg | **mean(\|err\|) × 10** (0.1 fixed-point).  IAE-style tracking quality |
| `zc` | interval agg | Zero-crossings of err sign in the interval |
| `d_max` | interval agg | max(\|d_term\|).  Spike amplitude under high Kd |
| `d_avg` | interval agg | mean(\|d_term\|).  `d_max/d_avg` ratio reveals noise character |
| `i_acc` | snapshot | Integrator accumulator (slow signal, snapshot suffices) |
| `turn_max/avg` | interval agg | max/mean(\|turn_dps\|).  Output amplitude |
| `v_max/avg/min` | interval agg | max/mean/min applied speed (mm/s) over the interval.  Surfaces dynamic-speed behavior (#126) |
| `sat` | cumulative delta | Saturation hits (clamp actually acted) since pidstat start |

The 10 aggregate columns (err_min/max/avg, d_max/avg, turn_max/avg, v_max/avg/min) print `-` when `interval_tick_count == 0` (idle entry, no ticks yet).

All `err`/`d_term` columns are **edge-independent** (`err = target - intens` regardless of the selected edge, #180); the edge only flips the sign applied to `turn_dps` downstream, so `turn_max/avg` magnitudes are unaffected too.

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
| Output saturation (turn authority too low) | `sat` rising + `turn_max == v_max` | Raise `speed` (max_turn tracks speed since #126) |
| Lost line in curves | `v_avg ≈ speed` even on tight curves | Engage dynamic speed via `--v-min` + `--v-alpha`/`--v-beta` (#126) |
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

- `target`: midpoint intensity between line (dark) and floor (bright).  Use `linetrace cal` first.
- `kp`: start at 0.1 and double until the robot oscillates, then back off ~30 %.  Typical values 0.15–0.4.  These are ~1/10 of the reflection-era kp because the err absolute value is now ~10× larger (intensity range 0–1024 vs reflection 0–100).
- `--hz`: the LPF2 color sensor publishes at ≈100 Hz; loops faster than that just re-issue the same sample.  Slower (e.g. 50 Hz) reduces CPU but adds latency.  When using dynamic speed, keep `v/hz ≈ 1.5 mm/tick`; for `speed > 200 mm/s` raise `--hz` to 150+ to preserve spatial resolution.

### 3.2 Dynamic speed (Issue #126)

`--v-min/--v-alpha/--v-beta` make speed vary in `[v_min, speed]` based on controller demand.  The harder the curve (large `|err|` or `|derr|`), the more the robot slows down.

| Flag | Meaning | Default | Range |
|---|---|---|---|
| `--v-min mm/s` | Floor speed during sharp curves | speed (= dynamic OFF) | `[1, speed]` |
| `--v-alpha A` | `\|err\|` coefficient | 0 (= dynamic OFF) | `[0.00, 100.00]` |
| `--v-beta B` | `\|derr\|` coefficient | 0 (= dynamic OFF) | `[0.00, 100.00]` |

Formula:
```
denom = 1 + α·|err|/100 + β·|derr|/30
v = clamp(speed / denom, v_min, speed)
max_turn = v   (always tracks)
```

`ERR_FLOOR=100` and `DERR_FLOOR=30` are hardcoded in source.  At the intensity scale (Issue #125, target=512, err range 0..1024) `|err|=100` is "moderate curve" territory.  If observed peak `|err|/|derr|` deviate substantially, compensate via α/β.

**Starting points** (Issue #126 hardware-validated, gains `kp=0.36 ki=0.15 kd=0.01`):

| α | β | v at hypothetical peak (`|err|=100, |derr|=30`, base=300) | Use |
|---|---|---|---|
| **1.5** | **0.3** | =100 → **clamp** 150 | **Validated baseline** (v=550, sat 0.07 %) |
| 1.0 | 0.5 | =150 (clamp) | Conservative starting point |
| 0.5 | 0.5 | ≈200 (67 %) | Mild slowdown |

Example (validated tuning):
```
linetrace run 300 0.36 512 --hz 200 --kd 0.01 --ki 0.15 \
              --v-min 150 --v-alpha 1.5 --v-beta 0.3
```

Hardware sat-% comparison (Issue #126, v_min=250 fixed):

| v | α=1.0 β=0.5 | α=1.5 β=0.3 |
|---|---|---|
| 350 | 0.30 % | **0.00 %** |
| 450 | 0.36 % | **0.033 %** |
| 550 | 0.36 % | **0.067 %** |

Raising α from 1.0 to 1.5 makes the controller decelerate earlier on curve entry, dramatically reducing turn_dps cap hits.  Lowering β from 0.5 to 0.3 cuts d-term-driven jitter so v_avg is more stable.

Verify with `pidstat`: `v_max≈300` on straights, `v_min` floor on curves indicates the loop is engaging.  The `turn_max ≤ v_max` invariant (max_turn tracking speed_apply) holds automatically.

**Important**: `--v-*` flags reset on every `run` (unlike kp/ki/kd/target/hz/edge which inherit prior values).  This avoids accidentally carrying a stale dynamic profile from a previous tuning session.  The reset target is the `linetrace.cfg` value when one is set (an explicit operator choice, not a silent carryover — see §3.3), otherwise "dynamic OFF" (`v_min := speed`, `α := β := 0`).  A config-derived `v_min` is clamped down to the current run's `speed` so a persisted high-speed floor never breaks a deliberately slow run; an explicit `--v-min` is still range-checked `[1, speed]`.

`edge` is one of the inherited parameters: a `run` that omits `--edge` keeps the edge selected by the previous `run` or `linetrace edge`.  `linetrace start` resets it to the configured value (`linetrace.cfg` `edge`, else `left`).  This is intentional — the followed edge is a course-level choice that should persist across speed/gain retuning within a session (see §2.4).

### 3.3 Persisted configuration (Issue #181)

PID tuning and sensor settings can be persisted to `/mnt/flash/linetrace.cfg` so they survive a reboot without rebuilding the firmware.  This mirrors `drivebase.cfg` (Issue #143).

linetrace is **not** auto-started by rcS (only `btsensor`/`drivebase` are), so the file is read each time you manually run `linetrace start`: `do_start` seeds the compiled defaults, then overlays the config keys, then spawns the daemon.  Loading happens before the daemon thread exists, so it is race-free without any settings-freeze machinery.

**Precedence: CLI args to `linetrace run` > `linetrace.cfg` > compiled defaults.**  A bare `linetrace run` runs at the persisted values; any positional or flag on `run` overrides them for that session.

Wire format (identical to `drivebase.cfg`): one `key=value` per line, `#` comments (full-line or trailing), whitespace trimmed, CRLF/LF and UTF-8 BOM tolerated.  A missing file is silent; unknown keys, parse errors, and out-of-range values are logged via `dmesg` and skipped per-line (the rest of the file still applies).  A successful load logs `linetrace: loaded N config key(s) from /mnt/flash/linetrace.cfg (M rejected)`.

All values are integers; gains are x100 scaled (matching the `run` CLI range):

| key | g_params | range | meaning |
|---|---|---|---|
| `kp_x100` | kp | `[-10000, 10000]` | proportional gain ×100 (`36` ⇒ 0.36) |
| `ki_x100` | ki | `[-10000, 10000]` | integral gain ×100 |
| `kd_x100` | kd | `[-10000, 10000]` | derivative gain ×100 |
| `target` | target | `[0, 1024]` | target intensity |
| `hz` | hz | `[10, 200]` | control loop frequency |
| `speed_mmps` | speed | `>= 0` | base forward speed (mm/s) |
| `v_min_mmps` | v_min | `[1, INT_MAX]` | dynamic-speed floor (clamped to run speed) |
| `v_alpha_x100` | v_alpha | `[0, 10000]` | `\|err\|` weight ×100 |
| `v_beta_x100` | v_beta | `[0, 10000]` | `\|derr\|` weight ×100 |
| `edge` | edge | `1` or `2` | followed edge: **1 = left** (default), **2 = right** |

Edit with `vi /mnt/flash/linetrace.cfg`, then `linetrace stop; linetrace start` to apply.  A template listing every key with its range and unit lives at `apps/linetrace/linetrace.cfg.sample` (values are the compiled defaults, except `v_min_mmps` whose floor is 1 since 0 is not a valid key value).  To change only the target on a running daemon without rewriting the file, use `linetrace target <N>` (the positional `run` resolution treats a single number as `speed`, not `target`).

`sensor-tilt-angle` (sensor servo, Issue #165) is intentionally not yet a key — it will be added when #165 lands.

## 4. Wiring

- Color sensor: any port (LPF2 type 61 auto-detected).  Mounted forward-facing about 10 mm above the floor.
- Motors: standard SPIKE Prime drivebase (motor on port A or C as L, B or D as R).  Configure once with `drivebase config <wheel_diameter_mm> <axle_track_mm>` before `linetrace run`.

## 5. Out of scope (future Issues)

- D term — current loop is P-only.  Add if oscillation / overshoot becomes problematic at higher speeds.
- Line-loss detection / search — the loop blindly drives at the most recent intensity reading forever.  pybricks does not handle this either; add a watchdog (no fresh sample within N ms → coast) when needed.
- Gain scheduling — at higher speeds `kp` should fall to keep the same closed-loop bandwidth; current CLI is constant-gain.
- Junction recognition (T / cross intersections) — needs color classification + a state machine.
- IMU heading correction — would conflict with color-driven steering; this app uses color alone.

## 6. References

- pybricks canonical pattern: `pybricks/tests/motors/drivebase_line.py`
- drivebase ABI: [drivebase.md](drivebase.md), `boards/spike-prime-hub/include/board_drivebase.h`
- color sensor (LPF2 type 61, RGB+I mode 5): [sensor.md](sensor.md), `boards/spike-prime-hub/include/board_legosensor.h`
