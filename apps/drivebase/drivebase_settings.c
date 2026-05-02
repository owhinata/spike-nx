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

/* Conservative seed for commit #6.  The 1-deg encoder resolution at
 * 1 kHz publish makes per-tick (5 ms) Δposition either 0 or 1 deg —
 * sample-to-sample velocity estimates jitter ±200 deg/s even under a
 * steady physical motion of ~100 deg/s.  Enable the speed and D
 * branches only after the observer is upgraded to a multi-sample
 * window (commit #7 / commit #11 retune work).  For now run pure
 * P-on-position so the closed loop converges deterministically inside
 * the deadband without amplifying observer noise.
 */

static const struct db_servo_gains_s g_servo_gains =
{
  .kp_pos        = 50,       /* duty.01% per deg ; 90 deg err → 4500 = */
                              /* 45 % PWM, comfortably above the SPIKE   */
                              /* Medium Motor's static-friction floor of */
                              /* ~6-8 % observed on the bench            */
  .ki_pos        = 0,        /* TODO commit #7: re-enable once observer */
  .kd_pos        = 0,        /*    settles                              */
  .kp_speed      = 0,
  .ki_speed      = 0,
  .deadband_mdeg = 1500,     /* ±1.5 deg around target                  */
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
