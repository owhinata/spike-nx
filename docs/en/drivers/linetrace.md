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

## 3. Tuning

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
