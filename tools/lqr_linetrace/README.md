# Line-follower LQR design (pre-plan investigation)

Two stdlib-only tools (no numpy/scipy/matplotlib) to decide *before any
firmware work* whether replacing the PID line follower
(`apps/linetrace/linetrace_main.c`) with LQR is worth it, and in what form:

| tool | role |
|---|---|
| `measure_cl.py` | measure the sensor-model params `c` & `L` on the Hub (drives existing NSH verbs over serial; no firmware change) |
| `lqr_sim.py` | offline LQR-vs-PID simulator; feed it the measured `c` & `L` |

**The whole verdict pivots on the real `c` and `L`** — so measure them first
(see "Measuring c and L"), then run the sim with those values.

```bash
# 1. measure on the Hub (host wired to it):
python3 measure_cl.py --mode sweep --device /dev/ttyACM0 --L 40   # L from a ruler
# 2. feed the sim:
python3 lqr_sim.py --report all --c <measured> --L <ruler>
python3 lqr_sim.py --report step --plot --speeds 150 300
```

## What it models

The line follower is the **outer kinematic loop** that emits a turn-rate
command (`drivebase_send_forever(speed, turn_dps)`); the drivebase tracks it.
So LQR applies to a 2-state kinematic plant:

```
state x = [e_y, e_theta]    e_y     cross-track error [mm]
                            e_theta heading error     [rad]
  dot e_y     = v*sin(e_theta)
  dot e_theta = omega                 (omega = turn-rate command, the input)
  A=[[0,v],[0,0]]  B=[[0],[1]]
```

A single reflected-intensity sensor mounted a **look-ahead distance `L`**
ahead of the axle measures a *blend* of position and heading:

```
err = target - intensity = -c * (e_y + L*sin e_theta)        within the linear edge band
  c [counts/mm]  intensity slope across the line edge   -- MUST be calibrated
  L [mm]         sensor look-ahead distance             -- MUST be measured
```

## LQR closed form (verified to machine precision)

```
K1 = sqrt(q1/r)              gain on e_y      (SPEED-INDEPENDENT)
K2 = sqrt(2*v*K1 + q2/r)     gain on e_theta  (~sqrt(v))
omega = -K1*e_y - K2*e_theta
char poly s^2 + K2 s + v*K1  =>  zeta = K2/(2 sqrt(v K1))
```

`q2 = 0` gives `zeta == 0.7071` at **every** speed (bandwidth scales as
`sqrt(v)`).  A fixed-gain PID hits 0.707 at only one speed.  `--report verify`
checks the closed form against the CARE residual (~1e-13).

## Three controllers compared

| | law | e_theta estimate | file role |
|---|---|---|---|
| `FirmwarePID` | faithful fixed-point port of `linetrace_main.c` | implicit (look-ahead) | baseline |
| `LqrPD` (Tier-1) | `omega = Gp*err + Gd(v)*d(err)/dt`, `Gd~1/sqrt(v)` | naive derivative | drop-in |
| `LqgObserver` (Tier-3) | Kalman reconstructs `(e_y,e_theta)` from `err` + known `omega`, then clean `-K1 e_y -K2 e_theta` | observer | robust |

## Headline findings (PLACEHOLDER c=60, L=40 — re-run with measured values!)

1. **The look-ahead `L` is the dominant factor.**  Sweep `--L 0..80`:
   - **Small L** (sensor near axle): PID *rings* (it has no heading damping);
     Tier-1 and LQG win big.
   - **Large L** (forward mount): a well-tuned PID is *excellent* — the
     look-ahead injects heading damping through the P channel.  **Tier-1
     LQR-PD BREAKS DOWN**: `d(err)/dt` is dominated by `-c*L*omega`, so it
     differentiates the controller's own output instead of the heading.
   - **LQG is robust across all L** — the only structurally-correct LQR
     formulation for a single look-ahead sensor.

2. **Honest bottom line:** at a realistic forward look-ahead, LQG ≈ a
   well-tuned PID on a straight-line step (IAE/settle within a few %).  LQR
   does **not** crush PID on the nominal task.  Its real wins are:
   robustness (insensitive to L / speed / mount → no re-tuning), the
   small-look-ahead regime, principled single-knob tuning (`q1/r`), and a
   path to curvature feed-forward via an augmented observer that PID lacks.

3. **Do not ship Tier-1** unless `L` is verified small.  The recommended
   structure is **Tier-3 LQG**.

4. **The verdict pivots on the real `c` and `L`.**  Measure them first with
   `measure_cl.py` (next section), then re-run every report above.

## Measuring c and L (`measure_cl.py`)

Two independent measurements pin down all of `{c, L, c*L}` (since `c*L = c x L`).

### Geometry

```
              line edge (straight), robot forward = up
   ============================================
                     (.)  <- sensor spot (look-ahead point)
                      |
                      |  L  = forward distance, axle-center -> spot
                      |
                  [==(O)==]   O = wheel-axle midpoint = pivot of `drivebase turn`
                   L     R
                  robot (top view)
```

### Step 1 — `L` by ruler (once)

Measure the forward distance from the **wheel-axle midpoint** `O` (the pivot of
an in-place turn) to the **sensor spot**, projected on the forward axis. This
is a fixed geometric constant; a ruler is the most accurate tool. Pass it as
`--L`.

### Step 2 — `c*L` by rotation sweep (automated)

Park the robot so the sensor sits on the **line edge** with intensity ≈
`target` (use `linetrace cal` to find the edge, or eyeball it). Then:

```bash
python3 measure_cl.py --mode sweep --device /dev/ttyACM0 --L <ruler_mm>
```

It pivots in place over `theta in [-20, +20] deg` (1° steps, 30 dps, HOLD
between samples). `drivebase turn` is **asynchronous** (it enqueues and
returns immediately), so after each step the script polls `drivebase
get-state` until `done==1` and the heading is stable before sampling — never
mid-turn, never stacking relative turns. It reads the achieved heading
(`angle_mdeg`) and averages intensity (`sensor color watch`) at each settled
step. The look-ahead spot moves laterally by `L*sin(theta)`, so the slope
`d(intensity)/d(theta)` near the setpoint **is `c*L`**; with `--L` it reports
`c = c*L / L`, the linear-band width, and the sensor noise (→ sim `--noise`).
Results land in `cl_params.txt` + `sweep_samples.csv`, with the exact
`lqr_sim.py` command to run next.

A **validity gate** refuses to emit `c` (prints `MEASUREMENT INVALID`) if the
robot covered < 60% of the commanded angle (turns not executing — wheels off
the ground, slippery surface, motors not driving, or a stall), if the
intensity swing is < 150 counts (sensor never crossed the edge — re-park it),
or if the fit `R^2 < 0.985`.  A preflight print shows the starting intensity
so you can confirm placement (Ctrl-C and re-park if it's far from `target`).

Why rotation and not lateral translation: a differential drive cannot crab
sideways, but it pivots precisely about `O`. The slope is **robust to a
lateral sensor-mount offset** (that only shifts where `intensity==target`).

Preconditions: `linetrace stop` first (the daemon CLAIMs the sensor
exclusively); the script issues it. The drivebase does *not* claim the color
sensor, so reads during a HOLD are fine. Straight line, flat surface.

### Step 3 (optional) — cross-check `c` by manual jog

Slide the robot sideways by hand across the edge in known mm steps, record
`offset_mm,intensity` to a CSV, then fit offline (no `L` needed — the slope is
`c` directly):

```bash
python3 measure_cl.py --mode fit-csv --csv jog.csv          # x = offset_mm
python3 measure_cl.py --mode fit-csv --csv sweep_samples.csv --x-is-angle --L 40
```

Verify the math with no hardware: `python3 measure_cl.py --mode self-test`.

### Tips

- If the fit `R^2 < 0.985`, lower `--central-frac` (default 0.6) — the band is
  too wide and includes the saturating ends. The ASCII scatter shows which
  points (`#`) were used.
- Re-fit a saved sweep without re-running the robot via `--mode fit-csv
  --csv sweep_samples.csv --x-is-angle --L <mm>`.

## Reports

| `--report` | shows |
|---|---|
| `verify`  | CARE residual self-check of the closed form |
| `damping` | `zeta(v)` for PID vs LQR (the speed-scheduling argument) |
| `coeffs`  | Tier-1 firmware coefficients + per-speed schedule + suggested config keys |
| `step`    | closed-loop step response, metrics (IAE/overshoot/settle/peak-turn), ASCII plot, CSV |
| `sweep`   | `q1/r` sweep at low & high speed |

Key knobs: `--c --L` (calibrate!), `--kp --ki --kd --target --hz` (baseline),
`--q1r --q2` (LQR weights; default `q1/r` is matched-P to baseline `kp`),
`--speeds --e_y0 --e_th0 --tau` (inner-loop lag) `--noise --dist`.

## Measured on this Hub (2026-05-30)

`L = 52 mm`, `c ≈ 150 counts/mm` (R²=0.9996).  Consequence: the intensity
**linear band is only ±3.4 mm** — beyond that the sensor saturates (0/1024).
Re-run the sim with these: `python3 lqr_sim.py --report step --c 150 --L 52
--noise 3.5 --e_y0 2` (use a *small* `e_y0` inside the band; a 20 mm step is
the saturated "lost-line" regime, not normal tracking).

Verdict at these params, in the realistic ±2-3 mm band: **LQG (Tier-3) clearly
beats PID** — ~0% overshoot, lowest IAE, settles faster as speed rises, never
saturates the turn rate; the current PID overshoots 50-130 % (worse with
speed, and *not* from the integral) because the large look-ahead makes the
sensor heading-dominated.  **But** the LQG observer diverges in the saturated
(>±3.4 mm) regime, so a real implementation needs a **saturation-aware
observer** (clamp innovation / freeze / seek-mode) — PID handles that regime
trivially via the output clamp.

## Status

Pre-plan design artifact for the LQR line-tracing investigation.  Per the
project workflow a control-structure change needs Issue → plan →
codex-review (2-stage gate) → HW verify → commit.  This tool produces the
sim evidence for that plan; **it is not committed yet** (no Issue).
