/****************************************************************************
 * apps/drivebase/drivebase_settings.h
 *
 * SPIKE Medium Motor + drivebase default tuning.  Values are seeded
 * from pybricks (`pbio/src/control_settings.c`, the `spike_medium_*`
 * tables) for the same physical motor and refined per-Issue once the
 * full closed loop runs on the bench.  Callers query through getters
 * so a future runtime override (Issue #77's DRIVEBASE_SET_DRIVE_
 * SETTINGS ioctl, commit #9) can mutate the live values without
 * recompilation.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_SETTINGS_H
#define __APPS_DRIVEBASE_DRIVEBASE_SETTINGS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/* Axis selector for aggregate distance/heading PID (#141 Phase 2).  Used
 * by db_settings_pid_gains / db_settings_completion to fetch per-axis
 * tunables.  Each axis runs an independent PID + trajectory inside the
 * aggregate controller (apps/drivebase/drivebase_aggregate.c); the
 * per-side servo no longer carries PID/trajectory state.
 */

enum db_axis_e
{
  DB_AXIS_DISTANCE = 0,
  DB_AXIS_HEADING  = 1,
  DB_AXIS_NUM      = 2,
};

/* PID gain set for one aggregate control axis.  All gains are integer
 * scaled to keep the per-tick PID body branch-free / FPU-free.  Position
 * errors are in milli-deg of the corresponding state space:
 *   distance state = (sL_pos + sR_pos) / 2 (motor mdeg)
 *   heading  state = (sR_pos - sL_pos) / 2 (motor mdeg, spike-nx
 *                                           convention: positive = CCW)
 * Output is signed PWM duty (.01 %, range out_min..out_max).
 *
 * `deadband_mdeg` defines the +/- band around zero error where the
 * integrator stops accumulating (Codex's anti-windup recommendation
 * derived from pybricks `integrator.c`).
 *
 * pybricks-style drivebase uses ki = 0 because there is no constant
 * external force a P-only loop can't overcome (no gravity bias on a
 * differential drivebase).  Phase 2 (#141) adopted this.
 */

struct db_servo_gains_s
{
  int32_t kp_pos;            /* P on position error (duty / mdeg)       */
  int32_t ki_pos;            /* I on position error (duty / mdeg / s)   */
  int32_t kd_pos;            /* D on position error (duty / (mdeg/s))   */
  int32_t kp_speed;          /* P on speed error    (duty / (deg/s))    */
  int32_t ki_speed;          /* I on speed error    (duty / (deg/s) / s)*/
  int32_t deadband_mdeg;     /* anti-windup zero-error band             */
  int32_t out_min;           /* duty saturation low                     */
  int32_t out_max;           /* duty saturation high                    */
};

/* Trapezoidal trajectory limits.  Shared between distance and heading
 * controllers (the daemon scales heading with axle_track / wheel_dia).
 */

struct db_traj_limits_s
{
  int32_t v_max_mdegps;      /* peak speed                              */
  int32_t accel_mdegps2;     /* trapezoid up-slope                      */
  int32_t decel_mdegps2;     /* trapezoid down-slope                    */
};

/* Stall + completion thresholds. */

struct db_stall_settings_s
{
  int32_t stall_speed_mdegps;
                              /* speed below which we may stall          */
  int32_t stall_duty_min;     /* |duty| above which we may stall        */
  uint32_t stall_window_ms;   /* both true continuously for this long    */
};

struct db_completion_settings_s
{
  int32_t pos_tolerance_mdeg;
  int32_t speed_tolerance_mdegps;
  int32_t smart_continue_window_mdeg;
                              /* SMART continue-from-endpoint window;    */
                              /* decoupled from pos_tolerance (#140)      */
  uint32_t done_window_ms;
  uint32_t smart_passive_hold_ms;  /* COAST_SMART / BRAKE_SMART hold    */
};

/* Feed-forward gain — per-axis linear FF (Issue #127 Phase 6 Step 6.1).
 * Plan D6: kV/kA are per-axis (distance / heading), and the resulting
 * `duty_ff = kV * v_dps + kA * a_dps2` is summed inside db_pid_update
 * before saturation so anti-windup sees the full output.
 *
 * Units (Plan D7): kV is .01% duty per (deg/s), kA is .01% duty per
 * (deg/s^2).  Trajectory reference is in mdeg/s and mdeg/s^2; converted
 * to deg/{s,s^2} via /1000 in int32 BEFORE the multiplication so the
 * RT path never invokes __aeabi_ldivmod.  Setter validates each gain to
 * [-DB_FF_GAIN_LIMIT, +DB_FF_GAIN_LIMIT] = [-1000, +1000] to keep the
 * `gain * v_dps` product comfortably inside int32 (max |1000 * 1110| =
 * 1.11e6 for v, |1000 * 800| = 8e5 for a).
 *
 * Per-motor kS friction lives in a separate struct (added in Step 6.2).
 */

struct db_ff_axis_gains_s
{
  int32_t kV;                 /* .01% duty per (deg/s)                   */
  int32_t kA;                 /* .01% duty per (deg/s^2)                 */
};

/* Per-motor friction FF (Issue #127 Phase 6 Step 6.2).  Plan D1 + D6:
 * `sign(v) * kS` is non-linear in the L/R compose, so it must apply
 * per-motor AFTER the compose (`vL_ref = D - H`, `vR_ref = D + H`) and
 * not per-axis like kV/kA.  Gain itself is common (one kS), since the
 * SPIKE Medium Motor is identical on left and right.  Plan D4 prescribes
 * a sign-with-hysteresis state machine to suppress kS chatter around
 * v_ref=0; pybricks-style /2 attenuation (lib/pbio/src/observer.c:250)
 * keeps the rail-step on entry from looking like a kS-only saturation.
 *
 * Defaults are 0 (behavioural no-op).  Step 6.4 SysId measures kS from
 * the duty floor at which both wheels start to move; Step 6.5 seeds it.
 * Convention is `v_hyst_enter_mdegps > v_hyst_exit_mdegps > 0`, but the
 * setters do not enforce a cross-key invariant — pairs of writes via
 * db_config_load() can arrive in either order, and the hysteresis
 * function in drivebase_drivebase.c tolerates degenerate orderings
 * deterministically.  See db_settings_set_ff_v_hyst_enter_mdegps for
 * the rationale.
 */

struct db_ff_motor_friction_s
{
  int32_t kS;                       /* .01% duty, left = right common  */
  int32_t v_hyst_enter_mdegps;      /* enter: commit sign regardless   */
  int32_t v_hyst_exit_mdegps;       /* exit:  fall back to 0           */

  /* Terminal static-friction breakaway floor (Issue #158 Phase 7).
   * kS (above) is the small per-tick Coulomb assist applied while a
   * trajectory is running; it cannot break a dead-stopped wheel free
   * (kS <= DB_FF_GAIN_LIMIT = 10% even un-attenuated, vs ~25% measured
   * static friction).  When a finite move's trajectory has finished but
   * a wheel is trapped short of (or past) target, this raises that
   * wheel's pre-clamp duty to at least `terminal_breakaway` in the
   * direction of the wheel's position error — saturating (floor), not
   * additive — once per move per wheel.  Default 0 = behavioural no-op.
   * See drivebase_drivebase.c::apply_breakaway_floor for the one-shot
   * episode gate.
   */

  int32_t terminal_breakaway;       /* .01% duty floor, left = right    */
};

/* Per-side hysteresis state for sign(v_ref).  Only sign history, no
 * gain value — the gain lives in db_ff_motor_friction_s above.  Plan
 * D6: 2 instances on db_drivebase_s, one per motor side.
 */

struct db_ff_state_s
{
  int8_t  sign_v_held;        /* -1, 0, +1: hysteresised sign(v_ref)    */

  /* Terminal breakaway episode state (Issue #158 Phase 7).  Drives the
   * per-move one-shot in apply_breakaway_floor(): `breakaway_sign` is
   * the wheel-error sign at which the current episode started (0 = no
   * episode this move), `breakaway_consumed` latches once the wheel has
   * reached the deadband or crossed the target so the floor fires at
   * most once per move (no reverse re-engagement / buzz).  Both reset on
   * a new move (trajectory no longer done).  RT-thread-owned, no atomic.
   */

  bool    breakaway_consumed; /* episode done for this move             */
  int8_t  breakaway_sign;     /* -1, 0, +1: episode start direction     */
};

/* Battery sag correction (Issue #152 Phase 6 Step 6.3).  See Plan D3.
 *
 * `nominal_mv` is the voltage at which the compiled PID / FF gains were
 * tuned (SPIKE 6-cell Li-Ion: 7.2 V nominal).  Higher live vbat → no
 * correction needed (the math would scale duty DOWN, which is correct
 * for a constant control plant); lower live vbat → duty boosted by
 * `nominal_mv / vbat`.  `min_mv` caps the boost so a sub-nominal gauge
 * reading does not produce explosive overcorrection — at 6.0 V the
 * factor is 7200/6000 = 1.2× which is the upper bound the controller
 * sees regardless of how low vbat goes.
 */

struct db_battery_settings_s
{
  int32_t nominal_mv;
  int32_t min_mv;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Per-axis PID gain.  axis ∈ {DB_AXIS_DISTANCE, DB_AXIS_HEADING}.  As of
 * #141 (Phase 2) both axes share the same kp_pos/ki_pos/kd_pos/kp_speed/
 * deadband, but heading uses a tighter out_max (8000 vs 10000) so the
 * L/R duty compose (`left = D + H`, `right = D - H`) is less prone to
 * saturating both motors at once.
 */

const struct db_servo_gains_s *db_settings_pid_gains(enum db_axis_e axis);

/* Backwards-compat alias for any caller that still wants "the" servo
 * gains.  Returns the distance-axis gains.  New code should use
 * db_settings_pid_gains(axis) directly.
 */

const struct db_servo_gains_s        *db_settings_servo_gains(void);
const struct db_traj_limits_s        *db_settings_distance_limits(uint32_t wheel_d_um);
const struct db_traj_limits_s        *db_settings_heading_limits(uint32_t wheel_d_um,
                                                                 uint32_t axle_t_um);
const struct db_stall_settings_s     *db_settings_stall(void);

/* Per-axis completion settings (#141 Phase 2).  Each axis derives its
 * `pos_tolerance_mdeg` from its own kp_pos via the helper in
 * drivebase_settings.c.  Other fields (done_window_ms, smart_passive_
 * hold_ms, smart_continue_window_mdeg) are shared.
 */

const struct db_completion_settings_s *
    db_settings_completion_axis(enum db_axis_e axis);

/* Backwards-compat: returns the distance-axis completion settings.  New
 * code should use db_settings_completion_axis(axis).
 */

const struct db_completion_settings_s *db_settings_completion(void);

/* Per-axis feed-forward gain (Issue #127 Phase 6 Step 6.1).  Returns the
 * live mutable instance — caller treats the returned pointer as a
 * snapshot whose contents are stable for the RT thread lifetime once
 * db_settings_freeze() has been called.
 */

const struct db_ff_axis_gains_s *db_settings_ff_axis_gains(enum db_axis_e axis);

/* Per-motor friction FF (Issue #127 Phase 6 Step 6.2).  Same lifetime
 * contract as db_settings_ff_axis_gains() — the returned pointer is
 * cached at db_drivebase_init() time and dereferenced read-only from
 * the RT thread after db_settings_freeze().  Single common instance
 * (left == right == same physical motor model).
 */

const struct db_ff_motor_friction_s *db_settings_ff_motor_friction(void);

/* Battery sag correction settings (Issue #152 Phase 6 Step 6.3).  Same
 * lifetime contract as the other settings accessors — the returned
 * pointer is cached at db_drivebase_init() and dereferenced read-only
 * from the RT thread after db_settings_freeze().
 */

const struct db_battery_settings_s *db_settings_battery(void);

/****************************************************************************
 * Runtime override API (Issue #143)
 *
 * Per-key setters that callers (db_config_load() from /mnt/flash/drivebase.cfg)
 * use to mutate the live tunables.  Each setter performs range validation
 * and returns 0 on success, -EINVAL on out-of-range, or -EBUSY if the
 * settings module has been frozen.
 *
 * Invariant: settings may only be mutated before db_settings_freeze().
 * The drivebase daemon calls freeze() after config load and immediately
 * before launching the RT thread, so the RT-tick reader (which dereferences
 * the const pointer returned by db_settings_pid_gains() on every tick)
 * never races with a writer.
 ****************************************************************************/

int db_settings_set_pid_kp_pos(enum db_axis_e axis, int32_t value);
int db_settings_set_pid_ki_pos(enum db_axis_e axis, int32_t value);
int db_settings_set_pid_kd_pos(enum db_axis_e axis, int32_t value);
int db_settings_set_pid_kp_speed(enum db_axis_e axis, int32_t value);
int db_settings_set_pid_ki_speed(enum db_axis_e axis, int32_t value);
int db_settings_set_pid_deadband_mdeg(enum db_axis_e axis, int32_t value);
int db_settings_set_pid_out_max(enum db_axis_e axis, int32_t value);

/* For pos_tolerance_mdeg: -1 means "derive from kp_pos at query time"
 * (default).  Any value in [DB_ENCODER_FLOOR_MDEG, 60000] is an explicit
 * override that bypasses derivation.
 */

int db_settings_set_comp_pos_tol_mdeg(enum db_axis_e axis, int32_t value);
int db_settings_set_comp_speed_tol_mdegps(enum db_axis_e axis, int32_t value);
int db_settings_set_comp_smart_continue_mdeg(enum db_axis_e axis,
                                             int32_t value);
int db_settings_set_comp_done_window_ms(enum db_axis_e axis, uint32_t value);
int db_settings_set_comp_smart_passive_hold_ms(enum db_axis_e axis,
                                               uint32_t value);

int db_settings_set_stall_speed_mdegps(int32_t value);
int db_settings_set_stall_duty_min(int32_t value);
int db_settings_set_stall_window_ms(uint32_t value);

/* Feed-forward gain setters (Issue #127 Phase 6 Step 6.1).  Both gains
 * are bounded to [-DB_FF_GAIN_LIMIT, +DB_FF_GAIN_LIMIT] so the FF math
 * stays in int32 — see db_settings.c for the limit and the rationale.
 */

int db_settings_set_ff_kV(enum db_axis_e axis, int32_t value);
int db_settings_set_ff_kA(enum db_axis_e axis, int32_t value);

/* Per-motor friction FF setters (Issue #127 Phase 6 Step 6.2).  kS is
 * bounded to [0, DB_FF_GAIN_LIMIT] (Plan D4 — kS is friction torque
 * sign-multiplied at apply time, so the gain itself is non-negative).
 * The hysteresis enter / exit thresholds are each bounded independently
 * to [0, DB_FF_HYST_LIMIT_MDEGPS]; the cross-key relation
 * `v_hyst_enter > v_hyst_exit` is a convention enforced by the cfg
 * template, NOT by the setter (which would make cfg load order
 * load-bearing).  See drivebase_drivebase.c::ff_sign_with_hysteresis
 * for the tolerance to degenerate orderings.
 */

int db_settings_set_ff_kS(int32_t value);
int db_settings_set_ff_v_hyst_enter_mdegps(int32_t value);
int db_settings_set_ff_v_hyst_exit_mdegps(int32_t value);

/* Terminal static-friction breakaway floor (Issue #158 Phase 7).
 * Bounded to [0, DB_BREAKAWAY_LIMIT] (= [0, 3000] = 0..30 % duty); the
 * limit exceeds DB_FF_GAIN_LIMIT on purpose — see drivebase_settings.c.
 */

int db_settings_set_ff_terminal_breakaway(int32_t value);

/* Battery sag correction setters (Issue #152 Phase 6 Step 6.3).
 * Bounds are independent per-key (cross-key relation `min < nominal` is
 * a cfg convention, not a setter invariant — see the FF hysteresis
 * setter rationale).  Both in millivolts.
 */

int db_settings_set_battery_nominal_mv(int32_t value);
int db_settings_set_battery_min_mv(int32_t value);

/* Mark settings immutable.  Subsequent setter calls return -EBUSY.
 * Called by the drivebase daemon between config load and RT-thread start.
 */

void db_settings_freeze(void);
bool db_settings_is_frozen(void);

/* Reset freeze state.  Used by the daemon on each start so a fresh
 * config load can apply.
 */

void db_settings_thaw(void);

/* Restore all tunables to the values they had at first init.  Called
 * by the daemon before db_config_load() so each `drivebase start`
 * begins from the compiled-in defaults; previously-loaded keys that
 * are absent from the new cfg revert to their built-in value rather
 * than sticking from the last load.  Idempotent.  Must be called
 * before db_settings_freeze().
 */

void db_settings_reset_to_defaults(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_SETTINGS_H */
