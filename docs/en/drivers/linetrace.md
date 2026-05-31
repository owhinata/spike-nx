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
linetrace run <speed_mmps> <kp> [target] \      # main command
              [--ki K] [--kd K] [--hz N] \
              [--v-min mm/s] [--v-alpha A] [--v-beta B]
linetrace                                       # usage
```

Defaults: `target=512`, `--hz=100`.  `kp/ki/kd` are fractional (e.g. `linetrace run 100 0.3`).  Intensity-era kp values are roughly **1/10 of the reflection-era values** because err magnitudes are ~10× larger (range 0..1024 vs 0..100); start at ~0.15 instead of 1.5.

The `--max-turn` flag and `linetrace max_turn N` subcommand were **retired in Issue #126**.  `max_turn` is now a per-tick derived value that tracks `speed_apply` (Issue #121's empirical `max_turn ≈ v` rule baked into the controller).

`--v-min/--v-alpha/--v-beta` enable dynamic speed control (Issue #126).  These flags are an opt-in safety switch and reset to "dynamic OFF" (`v_min := speed`, `α := β := 0`) on every `run`.  See §2.5 for details.

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
   - `turn_dps = clamp(kp * err, ±max_turn)`
   - `ioctl(DRIVEBASE_DRIVE_FOREVER, {speed_mmps, turn_dps})`
5. On SIGINT: `ioctl(DRIVEBASE_STOP, {COAST})`, close fds, exit.

### 2.3 `linetrace target <N>` (Issue #119)

Single-purpose subcommand that mutates `target` without engaging the drivebase.  Lets the user set the operational intensity threshold while still idle, so `linetrace status` shows last_err against the real target during robot positioning.

```
linetrace target 512           # apply the cal midpoint
```

Range: `target ∈ [0, 1024]`.  Missing / extra / non-integer arguments are usage errors; query the current value with `linetrace status`.

> The `linetrace max_turn N` mutator that originally shipped with #119 was retired in #126: `max_turn` is now derived per tick from `speed_apply` and no longer needs a CLI knob.

### 2.4 `linetrace pidstat` (Issue #118)

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

**Important**: `--v-*` flags reset to "dynamic OFF" (`v_min := speed`, `α := β := 0`) on every `run` (unlike kp/ki/kd/target/hz which inherit prior values).  This avoids accidentally carrying a stale dynamic profile from a previous tuning session.

## 3.5 LQG controller (Issue #169 / P1a)

The `lqg` verb engages an observer-based **Tier-3 LQG** line follower that **coexists** with the PID. The default stays PID (`run`); LQG is opt-in until a later phase flips the default. The investigation behind it lives in `tools/lqr_linetrace/` (oracle `lqr_sim.py`).

### 3.5.1 The 2-state model

The line follower is the outer kinematic loop emitting a turn-rate command:

```
state x = [e_y, e_theta]    e_y     cross-track error [mm]
                            e_theta heading error     [rad]
  dot e_y     = v * sin(e_theta)
  dot e_theta = omega                 (omega = turn-rate command)
```

The single reflected-intensity reading sits a look-ahead distance `L` ahead of the axle, so it measures a blend of position and heading:

```
err = target - intensity = -c * (e_y + L * sin e_theta)     (linear band)
```

with measurement row `C = [-c, -c·L]`. `c` [counts/mm] is the intensity slope across the line edge (calibrate); `L` [mm] is the look-ahead (measure). The `--edge left|right` flag picks the steer polarity: RIGHT edge bakes `+c` into the Kf table, LEFT bakes `-c`, so the LEFT Kf is the sign-flipped RIGHT Kf and the estimate stays physical signed mm/rad for both edges.

### 3.5.2 Closed-form state feedback K(v)

```
K1 = sqrt(q1r)                  gain on e_y      (speed-independent)
K2 = sqrt(2*v*K1 + q2)          gain on e_theta  (grows ~sqrt(v))
omega = -K1*e_y - K2*e_theta
```

With `q2 = 0` this yields damping `zeta = 0.7071` at **every** speed (a fixed-gain PID can only hit 0.707 at one speed). The feedback is edge-agnostic — `e_y`/`e_theta` are physical signed states; the edge sign lives only in the measurement row.

### 3.5.3 Kalman observer

A 2-state steady-state Kalman observer reconstructs `(e_y, e_theta)` from the look-ahead intensity and the previously-applied turn-rate command (predict ZOH on `prev_w`, then a scalar-innovation correct against `C`). Unlike a naive PD it never differentiates `err`, so it is immune to the `L·omega` corruption that breaks a Tier-1 PD at large look-ahead. The Riccati equation is solved **once per `lqg` engage** (CLI thread) into a 24-point speed-scheduled gain table (`LQG_V_LO=50` .. `LQG_V_HI=400` mm/s); the 100 Hz loop only linear-interpolates and runs one `sinf`+`sqrtf` per tick.

The whole config (params + Kf table) is published as one immutable double-buffered slot selected by an `atomic_uint` generation counter (release-store on publish, acquire-load per tick), so a hot LQG→LQG reconfigure can never pair new params with an old table. The observer re-seed is coupled to the generation (not a shared flag), so it can never be lost.

### 3.5.4 Saturation / LOST state machine (Issue #169 HW-verify fix)

Beyond the linear band the innovation lies, so the observer must not integrate garbage. The first P1a HW run **diverged off the line in both edge polarities** — the root cause was a measurement model that holds only inside the ±`sat_band` band combined with a saturation detector that keyed off the **hard** 0/1024 rails (so a near-white 975 was wrongly corrected on) and a LOST trigger that counted **consecutive** SAT ticks (a SAT↔NORMAL flap reset the streak, so LOST never fired). A single out-of-band tick then injected several mm of `e_y` in one correct → runaway. The fix:

- **NORMAL** — measurement inside the band-margin window `[black+margin, white-margin]` **and** `|ey_p| ≤ sat_band`: run the Kalman correct, but **clamp the innovation** to `±|c_signed|·sat_band` first, so even a band-boundary measurement can never inject more than ~`sat_band` of `e_y` in one tick.
- **SATURATED** — outside that window (or `|ey_p| > sat_band`): **freeze** the observer (predict only, skip the correct), keep steering on the last good heading, and accumulate out-of-band **time**. After `lost_time_ms` of accumulated saturation → LOST. Time-based (not a consecutive-tick streak) so a SAT↔NORMAL flap can no longer defeat LOST.
- **LOST** — stop trusting the observer; command a fixed seek `-last_side·seek_dps` (curving back toward the line). After `lost_timeout_ms` in LOST → **auto-stop**: brake + disengage (the daemon stays alive, `lost_cnt` increments). Re-engage with `lqg`.

Band re-entry re-seeds the estimate from the measurement (`e_y = -err/c_signed`, clamped to ±`sat_band`) so there is no discontinuity from a drifted frozen estimate.

The saturation detection now keys off the **measured sensor range** (`--black` / `--white` / `--band-margin`), not the hard 0/1024 rails, because at the operating target the look-ahead intensity leaves the linear band well before the hard rails. This is validated in the oracle: `python3 tools/lqr_linetrace/lqr_sim.py --report satband --c 150 --L 55 --target 634` shows the pre-fix SM diverges (`peak|innov|≈960`) on a recoverable out-of-band kick while the fixed SM recovers to ~0.1 mm (`innov` capped ≈14) and fires LOST→seek→auto-stop on a true loss.

**Operating target.** P0a measured the edge midpoint at ≈634 (black ≈249 / white ≈1020). The PID setpoint stays 512, but at 512 the black rail sits only ~1.7 mm off-line while the band is ±3.4 mm, so a normal look-ahead-induced spot excursion saturates and the observer loses the line **regardless of the SM**. The oracle confirms the fixed SM still cannot track the band at target=512 but tracks cleanly at the symmetric midpoint. The LQG therefore defaults its target to the midpoint (`LQG_DEF_TARGET=634`); PID is untouched.

### 3.5.5 Verbs

```
linetrace lqg <speed_mmps> [target] [--hz N] [--c C] [--L L] \
              [--edge left|right] [--q1r Q] [--q2 Q] \
              [--kf-qpos Q] [--kf-qhead Q] [--kf-rmeas R] \
              [--sat-band MM] [--seek-dps D] [--lost-time MS] [--lost-timeout MS] \
              [--black N] [--white N] [--band-margin N] [--lost-ticks N]
linetrace lqgstat [duration_ms [interval_ms]]
```

`target` defaults to the measured edge midpoint (634), not the PID 512 — the observer needs the symmetric midpoint.

`lqg` mirrors `run`: unspecified params inherit the prior `lqg` (or compiled defaults on the first engage after `start`); `seek_dps` defaults to the base speed. `lqg 0` idles at zero (FOREVER(0,0), stays engaged). `run` flips back to PID. `target` stays live for both controllers via `linetrace target N`. If the Riccati build fails to converge, the engage is **rejected** (no state change).

`lqgstat` streams the observer internals (default 1000 ms interval; sub-period intervals rejected per the observability-jitter rule):

```
 time_ms      iter intens    ey_hat  eth_hat_deg        innov        K1        K2    turn  sat  lost
```

`innov` (innovation, `err_meas - yhat`) is the single best observer-health signal — large or growing innov flags a saturation/model mismatch. `sat` is `NORM|SAT|LOST`. `iter` keeps advancing at speed=0 (idle-zero is not a hang).

### 3.5.6 Defaults (compiled, sim-locked)

| param | default | meaning |
|---|---|---|
| `c` | 150 | intensity slope counts/mm (calibrate) |
| `L` | 52 | look-ahead mm (measure) |
| `target` | 634 | LQG operating setpoint = measured edge midpoint (PID stays 512) |
| `edge` | right | steer polarity |
| `q1r` | 0.888 | q1/r (state-feedback stiffness) |
| `q2` | 0 | heading weight (0 → zeta 0.707 all v) |
| `kf_qpos` / `kf_qhead` / `kf_rmeas` | 1.0 / 1e-4 / 49 | Kalman Q[0][0] / Q[1][1] / R |
| `sat_band` | 3.4 | linear-band half-width mm (also the innovation-clamp scale `|c|·sat_band`) |
| `black` / `white` / `band_margin` | 249 / 1020 / 100 | measured sensor range; saturated outside `[black+margin, white-margin]` |
| `lost_time_ms` | 50 | accumulated out-of-band time → LOST |
| `seek_dps` | = base speed | LOST seek magnitude |
| `lost_timeout_ms` | 500 | LOST → auto-stop |
| `lost_ticks` | 5 | retained for CLI compat (LOST now keys off `lost_time_ms`) |

At the bench `c=150 L=52` the sim oracle reports LQG IAE 0.6/0.3 (vs PID 1.9/3.4) and 0% overshoot (vs PID 64-111%) at v=150/300 with zeta ≡ 0.707. The C float port of the Kf table agrees with the oracle to ~0.4% relative (single-precision Riccati), and the closed-form K(v) matches to 6 decimals.

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
