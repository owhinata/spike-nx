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

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_SETTINGS_H */
