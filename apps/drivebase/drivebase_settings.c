/****************************************************************************
 * apps/drivebase/drivebase_settings.c
 *
 * Default tuning for SPIKE Medium Motor (LPF2 type 48) and the
 * two-motor drivebase.  Derived from pybricks
 * `lib/pbio/src/control_settings.c spike_medium_*` tables and refined
 * on the bench during Issue #77 commits #6 (per-motor servo) and #7
 * (drivebase aggregation).
 *
 * Completion tolerance is derived from physical motor characteristics
 * rather than a hard-coded angle (Issue #140):
 *
 *   pos_tolerance_mdeg = max(DB_ENCODER_FLOOR_MDEG,
 *                            DB_COMPLETION_P_DUTY_FLOOR_PCT01 * 1000 /
 *                            kp_pos)
 *
 * The duty floor expresses the smallest P-term duty considered
 * mechanically meaningful (conservative estimate of the SPIKE Medium
 * Motor's static-friction threshold).  Below the resulting angle the P
 * term alone cannot push the motor, so further closed-loop correction
 * is not productive and the move is declared complete.  The encoder
 * floor (1 LSB of LUMP mode 2) keeps the tolerance above the
 * observation quantum even at high kp_pos.
 *
 * Note that DB_COMPLETION_P_DUTY_FLOOR_PCT01 is a *separate* concept
 * from stall_duty_min: stall_duty_min (60 %) is "duty at which stall
 * detection becomes meaningful" and reflects 60 % of the motor's stall
 * current, not the static-friction threshold.
 *
 * Phase 2 (#141) split the gains and completion settings into per-axis
 * instances (DB_AXIS_DISTANCE / DB_AXIS_HEADING).  Both axes share the
 * same baseline tuning but heading uses `out_max=8000` (0.8x of
 * distance) to keep the L/R duty compose from saturating both motors
 * at once — pybricks uses `actuation_max * 2` for torque output, but
 * SPIKE drives duty directly so saturation is non-linear.  Drivebase
 * ki is forced to 0 per pybricks convention: there is no constant
 * external force a P-only loop can't overcome on a differential
 * drivebase, and ki>0 causes long-move wind-up oscillations
 * (observed pre-#141 in straight 300 / coast).
 ****************************************************************************/

#include <nuttx/config.h>

#include "drivebase_settings.h"
#include "drivebase_angle.h"

/****************************************************************************
 * Pre-processor Definitions — completion derivation (#140)
 ****************************************************************************/

/* "P-term duty floor" used to derive pos_tolerance.  Units are .01 %
 * duty (matching out_min/out_max).  400 = 4 % duty is below the
 * measured static-friction floor of the SPIKE Medium Motor (~6–8 %)
 * and is a conservative starting point: the tolerance ends up slightly
 * tighter than "just enough to overcome static friction", which is the
 * right side to err on.  Re-tune on the bench from pidstat 95/99
 * percentile pos_err.
 */

#define DB_COMPLETION_P_DUTY_FLOOR_PCT01   400

/* Floor on pos_tolerance to keep it above the encoder quantum.  LUMP
 * mode 2 reports 1° per LSB, so 1000 mdeg is the smallest difference
 * the observer can distinguish.
 */

#define DB_ENCODER_FLOOR_MDEG             1000

/* SMART continue-from-endpoint window (drivebase_servo.c uses this for
 * "previous endpoint reusable" arbitration).  Kept at 6000 mdeg so the
 * SMART behaviour is unchanged from the pre-#140 baseline where it was
 * `2 * pos_tolerance_mdeg` with pos_tolerance = 3000.
 */

#define DB_SMART_CONTINUE_WINDOW_MDEG     6000

/* Per-motor PID gain used both in g_servo_gains and in the completion
 * derivation.  Hoisted to a macro so the divide-by-zero guard can be
 * enforced at build time.
 */

#define DB_KP_POS_DEFAULT                   50

_Static_assert(DB_KP_POS_DEFAULT > 0,
               "kp_pos must be positive for tolerance derivation");

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
 *   - ki_pos: 15 for both axes.  pybricks's "no integral needed"
 *     rule was derived for torque-output controllers where static
 *     friction is folded into the feed-forward model.  SPIKE drives
 *     duty directly with no FF (until Phase 6), so a stopped motor
 *     sees ~12-18% breakaway friction which P alone often cannot
 *     overcome after the motor settles short of target.  ki=15
 *     takes ~1 s to add ~14% extra duty at the typical short-move
 *     terminal pos_err — fast enough to break out cleanly.
 *
 *     Both axes need this:
 *       * distance: bench (#141) showed motor sticking at ~76 % of
 *         the commanded distance with ki=0.  ki=5 only recovered
 *         partial; ki=15 reaches target exactly.
 *       * heading: same breakaway issue manifests on TURN commands
 *         (motor undershoots target heading by ~20° with ki=0).
 *         When heading is the *auxiliary* axis (HOLD during
 *         STRAIGHT), pos_err stays inside `deadband_mdeg=3000` and
 *         ki freezes naturally — so the historical "no integral for
 *         drivebase heading" worry about wind-up does not apply.
 *
 *     Wind-up worry from pre-#141: long-move oscillation came from
 *     per-motor ki=20 + no L/R closed loop.  Phase 2 has aggregate
 *     heading PID actively coupling the two motors, so distance ki
 *     drift no longer steers the robot, and heading ki only
 *     accumulates while genuinely off-target (not while merely
 *     drifting under HOLD).
 *   - kp_speed adds damping while moving; helps avoid the brake-
 *     reverse-brake oscillation we saw with the per-sample observer.
 *   - kd_pos stays 0 — the slope estimator already provides the
 *     time-derivative information through the speed loop, and a
 *     direct D on position would re-introduce the high-frequency
 *     jitter the observer is filtering out.
 *
 * deadband_mdeg is the I-term freeze threshold and is independent
 * from the completion pos_tolerance (#140 decoupled the two).  Stays
 * at 3000 mdeg.
 *
 * Heading axis (DB_AXIS_HEADING) uses the same base tuning but with
 * out_max=8000 (0.8x of distance) so the L/R duty compose does not
 * regularly saturate both motors at once.
 */

static const struct db_servo_gains_s g_pid_gains_distance =
{
  .kp_pos        = DB_KP_POS_DEFAULT,
  .ki_pos        = 15,          /* ki for static-friction break (#141  */
                                /* bench: ki=5 left ~8 mm undershoot,  */
                                /* ki=15 mirrors pre-#140 effective    */
                                /* duty rate within ~1 s.              */
                                /* #142 Step B confirmed ki=10 also    */
                                /* undershoots short straight 50 by    */
                                /* ~5 mm and prevents long turn 180    */
                                /* from completing due to slow i_acc   */
                                /* decay — ki=15 stays optimal until   */
                                /* a feed-forward path (Phase 6)       */
                                /* compensates static friction.)       */
  .kd_pos        = 0,
  .kp_speed      = 5,
  .ki_speed      = 0,
  .deadband_mdeg = 3000,
  .out_min       = -10000,
  .out_max       =  10000,
};

static const struct db_servo_gains_s g_pid_gains_heading =
{
  .kp_pos        = DB_KP_POS_DEFAULT,
  .ki_pos        = 15,          /* same breakaway rationale as dist;    */
                                /* #142 Step B: ki=10 leaves turn 90    */
                                /* hold at -2.52° (within tolerance     */
                                /* but quality regression).             */
  .kd_pos        = 0,
  .kp_speed      = 5,
  .ki_speed      = 0,
  .deadband_mdeg = 3000,
  .out_min       = -8000,       /* 0.8x distance — combine headroom    */
  .out_max       =  8000,
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
 *
 * Geometry inputs are micrometers (matching db_drivebase_s).  The
 * heuristic v_drive = wheel_d * 4 only needs mm precision so we
 * round (wheel_d_um / 1000) before multiplying — keeps the resulting
 * speed an integer mm/s and matches the historical default values.
 ****************************************************************************/

static struct db_traj_limits_s g_distance_limits;
static struct db_traj_limits_s g_heading_limits;

static void recompute_distance(uint32_t wheel_d_um)
{
  /* Express in milli-deg/s of motor angle.  v_mmps -> v_mdegps via
   * db_angle_mmps_to_mdegps.
   */

  int32_t v_mmps = (int32_t)((wheel_d_um + 500) / 1000) * 4;
  g_distance_limits.v_max_mdegps   = db_angle_mmps_to_mdegps(v_mmps,
                                                             wheel_d_um);
  g_distance_limits.accel_mdegps2  = g_distance_limits.v_max_mdegps * 4;
  g_distance_limits.decel_mdegps2  = g_distance_limits.v_max_mdegps * 4;
}

static void recompute_heading(uint32_t wheel_d_um, uint32_t axle_t_um)
{
  /* Heading is measured in deg of robot rotation.  Wheel travels
   * (axle_t * π / 360) mm per deg of heading; the same wheel speed
   * gives the heading rate.  Use 1 deg of heading -> some mm of wheel
   * travel as the conversion.
   */

  int32_t v_drive_mmps  = (int32_t)((wheel_d_um + 500) / 1000) * 4;
  /* (mdeg of heading)/s = (mdeg of wheel)/s * (wheel_d / axle_track) */
  int64_t v_wheel_mdegps = db_angle_mmps_to_mdegps(v_drive_mmps,
                                                   wheel_d_um);
  int64_t v_heading_mdegps =
      axle_t_um == 0 ? 0
                     : v_wheel_mdegps * (int64_t)wheel_d_um / axle_t_um;
  if (v_heading_mdegps > INT32_MAX) v_heading_mdegps = INT32_MAX;
  g_heading_limits.v_max_mdegps   = (int32_t)v_heading_mdegps;
  g_heading_limits.accel_mdegps2  = (int32_t)(v_heading_mdegps * 4);
  g_heading_limits.decel_mdegps2  = (int32_t)(v_heading_mdegps * 4);
}

const struct db_servo_gains_s *db_settings_pid_gains(enum db_axis_e axis)
{
  return (axis == DB_AXIS_HEADING) ? &g_pid_gains_heading
                                   : &g_pid_gains_distance;
}

const struct db_servo_gains_s *db_settings_servo_gains(void)
{
  return &g_pid_gains_distance;
}

const struct db_traj_limits_s *
db_settings_distance_limits(uint32_t wheel_d_um)
{
  recompute_distance(wheel_d_um);
  return &g_distance_limits;
}

const struct db_traj_limits_s *
db_settings_heading_limits(uint32_t wheel_d_um, uint32_t axle_t_um)
{
  recompute_heading(wheel_d_um, axle_t_um);
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

/* pos_tolerance_mdeg and smart_continue_window_mdeg are filled in at
 * each query by db_settings_completion_axis() so the values track the
 * current per-axis gain.  Two instances (distance, heading); other
 * fields are bench-tuned constants shared across axes.
 */

static struct db_completion_settings_s g_completion_distance =
{
  .pos_tolerance_mdeg         = 0,            /* derived (#140)        */
  .speed_tolerance_mdegps     = 30000,        /* 30 deg/s              */
  .smart_continue_window_mdeg = 0,            /* derived (#140)        */
  .done_window_ms             =    50,
  .smart_passive_hold_ms      =   100,        /* pybricks default      */
};

static struct db_completion_settings_s g_completion_heading =
{
  .pos_tolerance_mdeg         = 0,            /* derived               */
  .speed_tolerance_mdegps     = 30000,
  .smart_continue_window_mdeg = 0,            /* derived               */
  .done_window_ms             =    50,
  .smart_passive_hold_ms      =   100,
};

const struct db_stall_settings_s *db_settings_stall(void)
{
  return &g_stall;
}

static int32_t derive_pos_tolerance_mdeg(int32_t kp_pos)
{
  int32_t derived = (DB_COMPLETION_P_DUTY_FLOOR_PCT01 * 1000) / kp_pos;
  return derived > DB_ENCODER_FLOOR_MDEG ? derived : DB_ENCODER_FLOOR_MDEG;
}

const struct db_completion_settings_s *
db_settings_completion_axis(enum db_axis_e axis)
{
  const struct db_servo_gains_s *gains = db_settings_pid_gains(axis);
  struct db_completion_settings_s *out =
      (axis == DB_AXIS_HEADING) ? &g_completion_heading
                                : &g_completion_distance;

  out->pos_tolerance_mdeg         = derive_pos_tolerance_mdeg(gains->kp_pos);
  out->smart_continue_window_mdeg = DB_SMART_CONTINUE_WINDOW_MDEG;
  return out;
}

const struct db_completion_settings_s *db_settings_completion(void)
{
  return db_settings_completion_axis(DB_AXIS_DISTANCE);
}
