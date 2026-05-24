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

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

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

/* Non-const since Issue #143: the daemon may override these from
 * /mnt/flash/drivebase.cfg before the RT thread starts, then
 * db_settings_freeze() makes the structs effectively read-only for the
 * remainder of the daemon's lifetime.
 */

static struct db_servo_gains_s g_pid_gains_distance =
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

static struct db_servo_gains_s g_pid_gains_heading =
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

/* Feed-forward gain — per-axis (Issue #127 Phase 6 Step 6.1, Plan D6).
 *
 * Defaults are 0 so this Step is a behavioural no-op until cfg overrides
 * land — the existing PID stack keeps full responsibility for actuation.
 * Step 6.5 will seed kV from the Issue #127 spec (~9) and kA from the
 * SysId fit.  Per-motor kS friction is in a separate struct added in
 * Step 6.2 (it is non-linear in the L/R compose, so cannot share the
 * per-axis plumbing — Plan D1 Codex BLOCKING).
 */

static struct db_ff_axis_gains_s g_ff_axis_gains_distance =
{
  .kV = 0,
  .kA = 0,
};

static struct db_ff_axis_gains_s g_ff_axis_gains_heading =
{
  .kV = 0,
  .kA = 0,
};

/* Per-motor friction FF (Issue #127 Phase 6 Step 6.2, Plan D1+D6).
 *
 * Default kS=0 keeps this Step a behavioural no-op; Step 6.5 seeds it
 * from `_sysid ramp-ks` results (Plan target ~700, applied as
 * sign·kS/2 so effective per-side is ~350 at full breakaway).  The
 * hysteresis enter/exit defaults are 5 dps / 1 dps (Plan D4): tight
 * enough that any non-trivial trajectory is well past the enter
 * threshold the moment it leaves the start segment, and wide enough
 * that v_ref oscillation around 0 (the only failure mode the
 * hysteresis exists for) never flips the held sign.
 */

static struct db_ff_motor_friction_s g_ff_motor_friction =
{
  .kS                       = 0,
  .v_hyst_enter_mdegps      = 5000,
  .v_hyst_exit_mdegps       = 1000,
};

/* Battery sag correction defaults (Issue #152 Phase 6 Step 6.3, Plan
 * D3).  7200 mV = 6-cell Li-Ion nominal (SPIKE pack mid-voltage).
 * 6000 mV = end-of-discharge cutoff where correction factor caps at
 * 1.2× (above which we'd be amplifying noise on a depleted cell).
 */

static struct db_battery_settings_s g_battery_settings =
{
  .nominal_mv = 7200,
  .min_mv     = 6000,
};

/* Bounds used by the per-key setters.  Kept here so config_load and the
 * defaults stay in sync.  out_max is bounded by the duty rail (10000 =
 * 100%) — heading axis defaults to 8000 but config may push it back up
 * to 10000 if the operator wants to give heading equal authority.
 */

#define DB_OUT_MAX_LIMIT             10000
#define DB_KI_LIMIT                  10000   /* sanity cap */
#define DB_KP_LIMIT                  10000   /* sanity cap */
#define DB_DEADBAND_LIMIT           100000   /* 100 deg = generous */
#define DB_POS_TOL_LIMIT_MDEG        60000   /* 60 deg = generous */
#define DB_SPEED_TOL_LIMIT_MDEGPS  1000000   /* 1000 dps = generous */
#define DB_DONE_WINDOW_LIMIT_MS      10000
#define DB_STALL_WINDOW_LIMIT_MS     10000

/* Feed-forward gain limit (Issue #127 Phase 6 Step 6.1, Plan D7).  Both
 * |kV| and |kA| are capped at 1000 so that the RT-path math
 *   kV * (v_mdegps / 1000)        max  1000 * 1110 = 1.11e6
 *   kA * (a_mdegps2 / 1000)       max  1000 *  800 = 8.0e5
 * stays comfortably inside int32 (1.11e6 + 8.0e5 ~= 2e6 << 2^31).
 * The v_mdegps / a_mdegps2 ceilings used here are taken from the
 * compiled-in trajectory limits (SPIKE Medium Motor at 56 mm wheel).
 */

#define DB_FF_GAIN_LIMIT              1000

/* Hysteresis threshold limit (Issue #127 Phase 6 Step 6.2, Plan D4).
 * 100 deg/s is well past the SPIKE Medium Motor's stall speed and lets
 * even a paranoid configurator pick a wide deadband if needed; the
 * compiled defaults sit at 5 dps (enter) / 1 dps (exit).
 */

#define DB_FF_HYST_LIMIT_MDEGPS     100000

/* Battery sag correction limits (Issue #152 Phase 6 Step 6.3).
 *
 *   1 mV  : implausibly small but prevents DIV/0; the RT side also
 *           clamps live vbat to battery_min_mv before dividing.
 *   12 V  : plausible upper bound for an over-charged 6S pack (the
 *           SPIKE Hub real ceiling is ~8.4 V).  Wider than needed
 *           leaves room for unusual cell chemistry experimentation.
 *
 * `nominal_mv * out_max = 12000 * 10000 = 1.2e8`, comfortably inside
 * int32 even before the divide.
 */

#define DB_BATTERY_MV_MIN                 1
#define DB_BATTERY_MV_MAX             12000

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

const struct db_ff_axis_gains_s *
db_settings_ff_axis_gains(enum db_axis_e axis)
{
  return (axis == DB_AXIS_HEADING) ? &g_ff_axis_gains_heading
                                   : &g_ff_axis_gains_distance;
}

const struct db_ff_motor_friction_s *db_settings_ff_motor_friction(void)
{
  return &g_ff_motor_friction;
}

const struct db_battery_settings_s *db_settings_battery(void)
{
  return &g_battery_settings;
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

/* Non-const since Issue #143 (runtime override).  See note on
 * g_pid_gains_distance above.
 */

static struct db_stall_settings_s g_stall =
{
  .stall_speed_mdegps  = 30000,    /* 30 deg/s                          */
  .stall_duty_min      =  6000,    /* 60 % duty                         */
  .stall_window_ms     =   200,
};

/* Storage for completion settings.  pos_tolerance_mdeg == -1 means
 * "derive from kp_pos on query" (the default, #140 behaviour).  An
 * explicit positive value bypasses derivation and is returned as-is.
 * smart_continue_window_mdeg follows the same convention.
 */

static struct db_completion_settings_s g_completion_distance =
{
  .pos_tolerance_mdeg         =    -1,        /* -1 = derive (#140)   */
  .speed_tolerance_mdegps     = 30000,        /* 30 deg/s              */
  .smart_continue_window_mdeg =    -1,        /* -1 = use default     */
  .done_window_ms             =    50,
  .smart_passive_hold_ms      =   100,        /* pybricks default      */
};

static struct db_completion_settings_s g_completion_heading =
{
  .pos_tolerance_mdeg         =    -1,
  .speed_tolerance_mdegps     = 30000,
  .smart_continue_window_mdeg =    -1,
  .done_window_ms             =    50,
  .smart_passive_hold_ms      =   100,
};

/* Scratch buffer returned by db_settings_completion_axis().  Filled in
 * on every call so derivations stay in sync with the latest kp_pos.
 * Per-axis to avoid concurrent calls (distance + heading from the same
 * tick) clobbering each other.
 */

static struct db_completion_settings_s g_completion_scratch_distance;
static struct db_completion_settings_s g_completion_scratch_heading;

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
  const struct db_completion_settings_s *src =
      (axis == DB_AXIS_HEADING) ? &g_completion_heading
                                : &g_completion_distance;
  struct db_completion_settings_s *out =
      (axis == DB_AXIS_HEADING) ? &g_completion_scratch_heading
                                : &g_completion_scratch_distance;

  *out = *src;
  if (out->pos_tolerance_mdeg < 0)
    {
      out->pos_tolerance_mdeg = derive_pos_tolerance_mdeg(gains->kp_pos);
    }
  if (out->smart_continue_window_mdeg < 0)
    {
      out->smart_continue_window_mdeg = DB_SMART_CONTINUE_WINDOW_MDEG;
    }
  return out;
}

const struct db_completion_settings_s *db_settings_completion(void)
{
  return db_settings_completion_axis(DB_AXIS_DISTANCE);
}

/****************************************************************************
 * Runtime override API (Issue #143)
 ****************************************************************************/

static bool g_settings_frozen = false;

static struct db_servo_gains_s *pid_gains_mut(enum db_axis_e axis)
{
  return (axis == DB_AXIS_HEADING) ? &g_pid_gains_heading
                                   : &g_pid_gains_distance;
}

static struct db_completion_settings_s *comp_mut(enum db_axis_e axis)
{
  return (axis == DB_AXIS_HEADING) ? &g_completion_heading
                                   : &g_completion_distance;
}

static int check_writable(void)
{
  if (g_settings_frozen)
    {
      syslog(LOG_WARNING,
             "drivebase: settings frozen; runtime setter ignored\n");
      return -EBUSY;
    }
  return 0;
}

static int set_in_range_i32(int32_t *dst, int32_t value,
                            int32_t lo, int32_t hi)
{
  int rc = check_writable();
  if (rc < 0) return rc;
  if (value < lo || value > hi) return -EINVAL;
  *dst = value;
  return 0;
}

static int set_in_range_u32(uint32_t *dst, uint32_t value, uint32_t hi)
{
  int rc = check_writable();
  if (rc < 0) return rc;
  if (value > hi) return -EINVAL;
  *dst = value;
  return 0;
}

int db_settings_set_pid_kp_pos(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  /* kp_pos must be strictly positive: completion tolerance derivation
   * divides by it and a zero-tolerance window is meaningless.
   */
  return set_in_range_i32(&pid_gains_mut(axis)->kp_pos, value,
                          1, DB_KP_LIMIT);
}

int db_settings_set_pid_ki_pos(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&pid_gains_mut(axis)->ki_pos, value,
                          0, DB_KI_LIMIT);
}

int db_settings_set_pid_kd_pos(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&pid_gains_mut(axis)->kd_pos, value,
                          0, DB_KP_LIMIT);
}

int db_settings_set_pid_kp_speed(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&pid_gains_mut(axis)->kp_speed, value,
                          0, DB_KP_LIMIT);
}

int db_settings_set_pid_ki_speed(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&pid_gains_mut(axis)->ki_speed, value,
                          0, DB_KI_LIMIT);
}

int db_settings_set_pid_deadband_mdeg(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&pid_gains_mut(axis)->deadband_mdeg, value,
                          0, DB_DEADBAND_LIMIT);
}

int db_settings_set_pid_out_max(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  int rc = check_writable();
  if (rc < 0) return rc;
  if (value <= 0 || value > DB_OUT_MAX_LIMIT) return -EINVAL;
  /* Keep out_min as the symmetric negative — pybricks/PID assumes
   * symmetric clamp for anti-windup invariants.
   */
  struct db_servo_gains_s *g = pid_gains_mut(axis);
  g->out_max = value;
  g->out_min = -value;
  return 0;
}

int db_settings_set_comp_pos_tol_mdeg(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  int rc = check_writable();
  if (rc < 0) return rc;
  /* -1 sentinel keeps the derive-from-kp_pos path active.  An explicit
   * value must be in [encoder floor, generous upper limit] so the
   * completion gate is reachable and meaningful.
   */
  if (value == -1)
    {
      comp_mut(axis)->pos_tolerance_mdeg = -1;
      return 0;
    }
  if (value < DB_ENCODER_FLOOR_MDEG || value > DB_POS_TOL_LIMIT_MDEG)
    {
      return -EINVAL;
    }
  comp_mut(axis)->pos_tolerance_mdeg = value;
  return 0;
}

int db_settings_set_comp_speed_tol_mdegps(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&comp_mut(axis)->speed_tolerance_mdegps, value,
                          0, DB_SPEED_TOL_LIMIT_MDEGPS);
}

int db_settings_set_comp_smart_continue_mdeg(enum db_axis_e axis,
                                             int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  int rc = check_writable();
  if (rc < 0) return rc;
  if (value == -1)
    {
      comp_mut(axis)->smart_continue_window_mdeg = -1;
      return 0;
    }
  if (value < 0 || value > DB_POS_TOL_LIMIT_MDEG) return -EINVAL;
  comp_mut(axis)->smart_continue_window_mdeg = value;
  return 0;
}

int db_settings_set_comp_done_window_ms(enum db_axis_e axis, uint32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_u32(&comp_mut(axis)->done_window_ms, value,
                          DB_DONE_WINDOW_LIMIT_MS);
}

int db_settings_set_comp_smart_passive_hold_ms(enum db_axis_e axis,
                                               uint32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_u32(&comp_mut(axis)->smart_passive_hold_ms, value,
                          DB_DONE_WINDOW_LIMIT_MS);
}

int db_settings_set_stall_speed_mdegps(int32_t value)
{
  return set_in_range_i32(&g_stall.stall_speed_mdegps, value,
                          0, DB_SPEED_TOL_LIMIT_MDEGPS);
}

int db_settings_set_stall_duty_min(int32_t value)
{
  return set_in_range_i32(&g_stall.stall_duty_min, value,
                          0, DB_OUT_MAX_LIMIT);
}

int db_settings_set_stall_window_ms(uint32_t value)
{
  return set_in_range_u32(&g_stall.stall_window_ms, value,
                          DB_STALL_WINDOW_LIMIT_MS);
}

static struct db_ff_axis_gains_s *ff_axis_mut(enum db_axis_e axis)
{
  return (axis == DB_AXIS_HEADING) ? &g_ff_axis_gains_heading
                                   : &g_ff_axis_gains_distance;
}

int db_settings_set_ff_kV(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&ff_axis_mut(axis)->kV, value,
                          -DB_FF_GAIN_LIMIT, DB_FF_GAIN_LIMIT);
}

int db_settings_set_ff_kA(enum db_axis_e axis, int32_t value)
{
  if (axis >= DB_AXIS_NUM) return -EINVAL;
  return set_in_range_i32(&ff_axis_mut(axis)->kA, value,
                          -DB_FF_GAIN_LIMIT, DB_FF_GAIN_LIMIT);
}

int db_settings_set_ff_kS(int32_t value)
{
  /* kS is friction torque magnitude; the sign is applied at use time
   * via the per-side hysteresis state, so the gain itself must be
   * non-negative.  Upper bound matches DB_FF_GAIN_LIMIT for symmetry.
   */
  return set_in_range_i32(&g_ff_motor_friction.kS, value,
                          0, DB_FF_GAIN_LIMIT);
}

int db_settings_set_ff_v_hyst_enter_mdegps(int32_t value)
{
  /* The cross-key relation `enter > exit` is documented in the cfg
   * template, not enforced at setter time — pairs of writes through
   * db_config_load() can arrive in either order, and the hysteresis
   * function (ff_sign_with_hysteresis in drivebase_drivebase.c)
   * tolerates `enter <= exit` deterministically (it just collapses or
   * inverts the held-zone semantics, with no risk of getting stuck).
   * Enforcing the cross-key invariant per-setter would make load
   * order load-bearing, which is fragile.
   */
  return set_in_range_i32(&g_ff_motor_friction.v_hyst_enter_mdegps, value,
                          0, DB_FF_HYST_LIMIT_MDEGPS);
}

int db_settings_set_ff_v_hyst_exit_mdegps(int32_t value)
{
  return set_in_range_i32(&g_ff_motor_friction.v_hyst_exit_mdegps, value,
                          0, DB_FF_HYST_LIMIT_MDEGPS);
}

int db_settings_set_battery_nominal_mv(int32_t value)
{
  return set_in_range_i32(&g_battery_settings.nominal_mv, value,
                          DB_BATTERY_MV_MIN, DB_BATTERY_MV_MAX);
}

int db_settings_set_battery_min_mv(int32_t value)
{
  return set_in_range_i32(&g_battery_settings.min_mv, value,
                          DB_BATTERY_MV_MIN, DB_BATTERY_MV_MAX);
}

void db_settings_freeze(void)
{
  g_settings_frozen = true;
}

bool db_settings_is_frozen(void)
{
  return g_settings_frozen;
}

void db_settings_thaw(void)
{
  g_settings_frozen = false;
}

void db_settings_reset_to_defaults(void)
{
  /* Snapshot the compiled-in defaults the first time we are called
   * (which is before any setter has had a chance to mutate them),
   * then restore from that snapshot on every subsequent call.  This
   * keeps the daemon's stop/start cycle behaving as if each start
   * had a fresh process: any key not mentioned in the new cfg reverts
   * to its built-in default rather than sticking from the previous
   * load.
   */

  if (g_settings_frozen)
    {
      syslog(LOG_WARNING,
             "drivebase: settings frozen; reset_to_defaults ignored\n");
      return;
    }

  static bool captured = false;
  static struct db_servo_gains_s          d0, h0;
  static struct db_stall_settings_s       s0;
  static struct db_completion_settings_s  cd0, ch0;
  static struct db_ff_axis_gains_s        ffd0, ffh0;
  static struct db_ff_motor_friction_s    ffm0;
  static struct db_battery_settings_s     bat0;

  if (!captured)
    {
      d0       = g_pid_gains_distance;
      h0       = g_pid_gains_heading;
      s0       = g_stall;
      cd0      = g_completion_distance;
      ch0      = g_completion_heading;
      ffd0     = g_ff_axis_gains_distance;
      ffh0     = g_ff_axis_gains_heading;
      ffm0     = g_ff_motor_friction;
      bat0     = g_battery_settings;
      captured = true;
    }
  else
    {
      g_pid_gains_distance     = d0;
      g_pid_gains_heading      = h0;
      g_stall                  = s0;
      g_completion_distance    = cd0;
      g_completion_heading     = ch0;
      g_ff_axis_gains_distance = ffd0;
      g_ff_axis_gains_heading  = ffh0;
      g_ff_motor_friction      = ffm0;
      g_battery_settings       = bat0;
    }
}
