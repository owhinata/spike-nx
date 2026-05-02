/****************************************************************************
 * apps/drivebase/drivebase_settings.c
 *
 * Default tuning for SPIKE Medium Motor (LPF2 type 48) and the
 * two-motor drivebase.  Derived from pybricks
 * `lib/pbio/src/control_settings.c spike_medium_*` tables and refined
 * on the bench during Issue #77 commits #6 (per-motor servo) and #7
 * (drivebase aggregation).
 ****************************************************************************/

#include <nuttx/config.h>

#include "drivebase_settings.h"
#include "drivebase_angle.h"

/****************************************************************************
 * Per-motor servo gains
 *
 * Conservative seed.  Verified non-oscillating in commit #6 with a
 * 90-deg position step.  Tuned upward in commit #7 once the L/R
 * coupling settles.  The deadband + saturation values match pybricks
 * for the same motor, which keeps anti-windup behaviour predictable.
 ****************************************************************************/

/* Tuning iteration after commit #6.5 swapped the per-sample IIR
 * observer for a sliding-window slope estimator (≈ 30 ms window).
 * v_est now reflects real motor speed within ~5 % at the typical
 * SPIKE Medium Motor operating range (50-500 deg/s), so the speed
 * branch is no longer noise-amplifying.
 *
 * Bench tuning posture (refined in commits #7/#11 once the L+R
 * coupling is in the loop):
 *   - kp_pos  drives the bulk of the response.  50 .01% per deg →
 *     45 % PWM at a 90 deg error, comfortably above the observed
 *     ~6-8 % static-friction floor.
 *   - ki_pos  small but present, just enough to push past static
 *     friction at the end of a move.  Wind-up is bounded by the
 *     anti-windup deadband + saturation gates from commit #5.
 *   - kp_speed adds damping while moving; helps avoid the brake-
 *     reverse-brake oscillation we saw with the per-sample observer.
 *   - kd_pos stays 0 — the slope estimator already provides the
 *     time-derivative information through the speed loop, and a
 *     direct D on position would re-introduce the high-frequency
 *     jitter the observer is filtering out.
 *
 * deadband_mdeg matches the completion pos_tolerance so the
 * integrator stops accumulating exactly where the motion is
 * declared "done".
 */

static const struct db_servo_gains_s g_servo_gains =
{
  .kp_pos        = 50,
  .ki_pos        = 20,
  .kd_pos        = 0,
  .kp_speed      = 5,
  .ki_speed      = 0,
  .deadband_mdeg = 3000,
  .out_min       = -10000,
  .out_max       =  10000,
};

/****************************************************************************
 * Distance / heading trajectory limits
 *
 * Defaults chosen so that, at 56 mm wheel diameter, a typical
 * straight move feels brisk but never saturates the H-bridge:
 *   v_drive_default_mmps = wheel_d * 4               (≈ 224 mm/s @ 56 mm)
 *   a_drive_default      = v_drive_default * 4       (1/4 s ramp)
 * The same scale law is applied to heading via axle_track:
 *   v_turn_default_dps   = (v_drive * 360) / (π * axle_track)
 ****************************************************************************/

static struct db_traj_limits_s g_distance_limits;
static struct db_traj_limits_s g_heading_limits;

static void recompute_distance(uint32_t wheel_d_mm)
{
  /* Express in milli-deg/s of motor angle.  v_mmps -> v_mdegps via
   * db_angle_mmps_to_mdegps.
   */

  int32_t v_mmps = (int32_t)wheel_d_mm * 4;
  g_distance_limits.v_max_mdegps   = db_angle_mmps_to_mdegps(v_mmps,
                                                             wheel_d_mm);
  g_distance_limits.accel_mdegps2  = g_distance_limits.v_max_mdegps * 4;
  g_distance_limits.decel_mdegps2  = g_distance_limits.v_max_mdegps * 4;
}

static void recompute_heading(uint32_t wheel_d_mm, uint32_t axle_t_mm)
{
  /* Heading is measured in deg of robot rotation.  Wheel travels
   * (axle_t * π / 360) mm per deg of heading; the same wheel speed
   * gives the heading rate.  Use 1 deg of heading -> some mm of wheel
   * travel as the conversion.
   */

  int32_t v_drive_mmps  = (int32_t)wheel_d_mm * 4;       /* same scale */
  /* (mdeg of heading)/s = (mdeg of wheel)/s * (wheel_d / axle_track) */
  int64_t v_wheel_mdegps = db_angle_mmps_to_mdegps(v_drive_mmps,
                                                   wheel_d_mm);
  int64_t v_heading_mdegps = v_wheel_mdegps * wheel_d_mm / axle_t_mm;
  if (v_heading_mdegps > INT32_MAX) v_heading_mdegps = INT32_MAX;
  g_heading_limits.v_max_mdegps   = (int32_t)v_heading_mdegps;
  g_heading_limits.accel_mdegps2  = (int32_t)(v_heading_mdegps * 4);
  g_heading_limits.decel_mdegps2  = (int32_t)(v_heading_mdegps * 4);
}

const struct db_servo_gains_s *db_settings_servo_gains(void)
{
  return &g_servo_gains;
}

const struct db_traj_limits_s *
db_settings_distance_limits(uint32_t wheel_d_mm)
{
  recompute_distance(wheel_d_mm);
  return &g_distance_limits;
}

const struct db_traj_limits_s *
db_settings_heading_limits(uint32_t wheel_d_mm, uint32_t axle_t_mm)
{
  recompute_heading(wheel_d_mm, axle_t_mm);
  return &g_heading_limits;
}

/****************************************************************************
 * Stall + completion
 ****************************************************************************/

static const struct db_stall_settings_s g_stall =
{
  .stall_speed_mdegps  = 30000,    /* 30 deg/s                          */
  .stall_duty_min      =  6000,    /* 60 % duty                         */
  .stall_window_ms     =   200,
};

static const struct db_completion_settings_s g_completion =
{
  .pos_tolerance_mdeg     = 3000,  /* 3 deg                             */
  .speed_tolerance_mdegps = 30000, /* 30 deg/s                          */
  .done_window_ms         =    50,
  .smart_passive_hold_ms  =   100, /* pybricks default                  */
};

const struct db_stall_settings_s *db_settings_stall(void)
{
  return &g_stall;
}

const struct db_completion_settings_s *db_settings_completion(void)
{
  return &g_completion;
}
