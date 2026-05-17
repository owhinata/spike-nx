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
