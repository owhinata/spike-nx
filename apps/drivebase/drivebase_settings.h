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

/* Per-side servo gains.  All gains are integer scaled to keep the
 * per-tick PID body branch-free / FPU-free.  Position errors are in
 * milli-deg, speeds are in deg/s, output is signed PWM duty (.01 %,
 * range -10000..10000 i.e. the LEGOSENSOR_SET_PWM ABI unit).
 *
 * `deadband_mdeg` defines the +/- band around zero error where the
 * integrator stops accumulating (Codex's anti-windup recommendation
 * derived from pybricks `integrator.c`).
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
  uint32_t done_window_ms;
  uint32_t smart_passive_hold_ms;  /* COAST_SMART / BRAKE_SMART hold    */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

const struct db_servo_gains_s        *db_settings_servo_gains(void);
const struct db_traj_limits_s        *db_settings_distance_limits(uint32_t wheel_d_mm);
const struct db_traj_limits_s        *db_settings_heading_limits(uint32_t wheel_d_mm,
                                                                 uint32_t axle_t_mm);
const struct db_stall_settings_s     *db_settings_stall(void);
const struct db_completion_settings_s *db_settings_completion(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_SETTINGS_H */
