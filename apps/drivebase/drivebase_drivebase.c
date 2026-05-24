/****************************************************************************
 * apps/drivebase/drivebase_drivebase.c
 *
 * Two-motor drivebase aggregator.  Phase 2 (#141) refactored this from
 * a per-side trajectory dispatcher into a 2-loop aggregate PID:
 *
 *   1. Drain each per-motor encoder + update its observer.
 *   2. Build state_distance = (sL.x + sR.x) / 2 and state_heading =
 *      (sR.x - sL.x) / 2 (motor-mdeg space; positive heading = R
 *      ahead = CCW per spike-nx public convention).
 *   3. Run control_distance and control_heading independently — each
 *      has its own trajectory + PID + completion state.
 *   4. Compose per-side duty:
 *         left_duty  = clamp(duty_distance - duty_heading, ±10000)
 *         right_duty = clamp(duty_distance + duty_heading, ±10000)
 *   5. Apply COAST/BRAKE propagation (pybricks-style: if EITHER axis
 *      decides COAST or BRAKE, stop both).
 *   6. Refresh cached aggregate state (distance_mm, angle_mdeg, etc.)
 *      for the chardev get_state surface.
 *
 * User commands route to one "primary" axis (DRIVE_STRAIGHT=distance,
 * TURN=heading, DRIVE_CURVE/ARC=both) and HOLD the other in place via
 * a zero-length trajectory.  on_completion is honoured per-primary
 * only; the auxiliary axis runs with on_completion=HOLD so its done
 * judgement does not trigger early COAST/BRAKE propagation.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include "drivebase_drivebase.h"
#include "drivebase_imu.h"         /* Phase 3b (#148) heading injection  */
#include "drivebase_motor.h"
#include "drivebase_rt.h"          /* DB_RT_TICK_MS_DEFAULT (Issue #120) */
#include "drivebase_settings.h"
#include "drivebase_angle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_PI_NUM   355
#define DB_PI_DEN   113

/* Phase 4 C stretch ratio threshold (Issue #144).  Stretch only kicks
 * in when the duration ratio between the longer and shorter axes
 * exceeds this threshold (expressed as a fraction × 1000).  Stretching
 * a near-symmetric pair slows the follower trajectory enough to let the
 * heading PID accumulate i_acc and overshoot — the regression observed
 * on `curve 150 90` (ratio ~1.25×).  At ratio >= 1.5× the tangent
 * regression dominates and stretching wins.  1500 = 1.5 split the two
 * regimes cleanly on bench (post-#142):
 *   - ratio 1.25× (curve 150 90): skip stretch, keep baseline behaviour
 *   - ratio 1.73× (curve 200 360): stretch, eliminate tangent
 *   - ratio 2.26× (curve 300 90):  stretch, eliminate tangent
 * Tuning this knob downwards is a Phase 6 (feed-forward) follow-up.
 */

#define DB_TRAJECTORY_STRETCH_RATIO_THRESHOLD_MIL  1500

/****************************************************************************
 * Private Helpers — geometry conversions
 ****************************************************************************/

/* Wheel travel (mm) → motor angle (mdeg) for a single wheel.           */

static int64_t mm_to_motor_mdeg(int32_t mm, uint32_t d_um)
{
  return db_angle_mm_to_mdeg(mm, d_um);
}

/* Heading angle (robot-mdeg) ↔ wheel-differential travel (mm).         */

static int32_t diff_mm_to_heading_mdeg(int32_t diff_mm, uint32_t a_um)
{
  if (a_um == 0) return 0;
  int64_t pi_diff = db_angle_div_round((int64_t)diff_mm * 180000000LL,
                                       (int64_t)a_um);
  return (int32_t)db_angle_div_round(pi_diff * DB_PI_DEN, DB_PI_NUM);
}

/* Convert a heading rotation in robot mdeg to a state_heading delta in
 * motor-mdeg (state-space = (sR - sL) / 2).  Derivation:
 *   per-wheel diff travel (mm)   = h_rad * axle_t = (h_mdeg / 1000) *
 *                                  π/180 * axle_t_mm
 *   per-wheel diff travel (motor mdeg)
 *                                = (per_wheel_mm × 360 × 1000) /
 *                                  (π × wheel_d_mm)
 *   state_heading_delta_mdeg     = per_wheel_diff_motor_mdeg / 2 ...
 *   but actually 2 × state_heading_delta = sR - sL = full diff, so
 *     state_heading_delta = (sR - sL) / 2
 *                         = h_mdeg × axle_t / wheel_d
 *
 * (Algebraic simplification: π and 360 cancel out cleanly.)
 */

static int64_t heading_mdeg_to_state_mdeg(int64_t heading_mdeg,
                                          uint32_t wheel_d_um,
                                          uint32_t axle_t_um)
{
  /* Phase 3b (#148) widened the input to int64 so the Madgwick-derived
   * world heading (unwrapped over an entire run, ±2^63 mdeg headroom)
   * can flow straight into the state-space PID without an int32 saturate.
   * Encoder callers stay correct: int32 lossless to int64.
   */

  if (wheel_d_um == 0) return 0;
  return heading_mdeg * (int64_t)axle_t_um / (int64_t)wheel_d_um;
}

/* Clamp helper for the L/R compose. */

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  if (v > hi) return hi;
  if (v < lo) return lo;
  return v;
}

/* Phase 6 Step 6.2 (#152) per-motor friction FF sign hysteresis.
 *
 * Plan D4 spec — evaluate top-down:
 *   |v_ref| >=  enter           → +1 (commit, ignore prev state)
 *   |v_ref| <= -enter           → -1
 *   |v_ref|  <  exit            →  0 (kS contribution off, safe zone)
 *   else (exit <= |v_ref| < enter) → prev (hold)
 *
 * Strong signals (`|v_ref| >= enter`) override the held sign so a
 * direction reversal cannot get stuck.  Centre deadband (`|v_ref| <
 * exit`) zeros out kS so v_ref oscillating around 0 does not chatter
 * the friction term.  The hysteresis zone holds the previous sign so
 * a brief dip below `enter` (e.g. trapezoidal cruise modulation) does
 * not drop kS to 0 and then re-add the rail step on the next tick.
 *
 * The function is tolerant of arbitrary `enter`/`exit` orderings (the
 * setter does not enforce `enter > exit` — see drivebase_settings.c
 * for the cfg load-order rationale).  Degenerate orderings collapse
 * the hysteresis zone but never deadlock.
 */

static int8_t ff_sign_with_hysteresis(int32_t v_ref_mdegps, int8_t prev,
                                      int32_t enter, int32_t exit)
{
  if (v_ref_mdegps >=  enter) return +1;
  if (v_ref_mdegps <= -enter) return -1;
  int32_t mag = v_ref_mdegps < 0 ? -v_ref_mdegps : v_ref_mdegps;
  if (mag < exit) return 0;
  return prev;
}

/****************************************************************************
 * Public Functions — lifecycle
 ****************************************************************************/

int db_drivebase_init(struct db_drivebase_s *db,
                      uint32_t wheel_d_um, uint32_t axle_t_um,
                      uint32_t tick_ms)
{
  if (wheel_d_um == 0 || axle_t_um == 0)
    {
      return -EINVAL;
    }

  if (tick_ms == 0)
    {
      tick_ms = DB_RT_TICK_MS_DEFAULT;
    }

  memset(db, 0, sizeof(*db));
  db->wheel_d_um     = wheel_d_um;
  db->axle_t_um      = axle_t_um;
  db->tick_ms        = tick_ms;
  db->active_command = DRIVEBASE_ACTIVE_NONE;

  db_servo_init(&db->servo[DB_SIDE_LEFT],  DB_SIDE_LEFT,  tick_ms);
  db_servo_init(&db->servo[DB_SIDE_RIGHT], DB_SIDE_RIGHT, tick_ms);

  db_aggregate_control_init(&db->control[DB_AXIS_DISTANCE],
                            DB_AXIS_DISTANCE, tick_ms);
  db_aggregate_control_init(&db->control[DB_AXIS_HEADING],
                            DB_AXIS_HEADING,  tick_ms);

  /* Phase 6 Step 6.2 (#152): cache the per-motor friction FF pointer
   * for the RT thread.  Settings are frozen before the RT thread
   * starts so the dereference is single-writer / many-reader.  The
   * memset above already zeroed ff_state_left/right.sign_v_held.
   */

  db->ff_motor = db_settings_ff_motor_friction();
  return 0;
}

/* Compute the raw (encoder-derived, pre-origin) avg distance and
 * heading from the currently-cached per-servo positions.  Used by
 * reset() / set_origin() to capture an offset so the public state
 * surface starts from a known value.
 */

static void db_drivebase_raw_state(const struct db_drivebase_s *db,
                                   int32_t *avg_mm,
                                   int32_t *heading_mdeg)
{
  struct db_servo_status_s sL;
  struct db_servo_status_s sR;
  db_servo_get_status(&db->servo[DB_SIDE_LEFT],  &sL);
  db_servo_get_status(&db->servo[DB_SIDE_RIGHT], &sR);

  int32_t lL_mm = db_angle_mdeg_to_mm(sL.act_x_mdeg, db->wheel_d_um);
  int32_t lR_mm = db_angle_mdeg_to_mm(sR.act_x_mdeg, db->wheel_d_um);
  *avg_mm       = (lL_mm + lR_mm) / 2;
  *heading_mdeg = diff_mm_to_heading_mdeg(lR_mm - lL_mm, db->axle_t_um);
}

/* Phase 3b (#148) — guard for "can we trust the IMU right now?".
 *   IMU attached, user requested gyro mode, calibrated, AND a fresh
 *   sample arrived within the stale threshold.  Used to gate origin
 *   capture and command-start latching so a boot-time pending
 *   activation never wires up a stale baseline.
 */

static bool gyro_origin_capturable(const struct db_drivebase_s *db,
                                   uint64_t now_us)
{
  if (db->imu == NULL) return false;
  if (db->use_gyro_requested == DRIVEBASE_USE_GYRO_NONE) return false;
  if (!db_imu_is_calibrated(db->imu)) return false;
  if (db_imu_is_stale(db->imu, now_us,
                      DB_IMU_DEFAULT_STALE_THRESHOLD_US)) return false;
  return true;
}

int db_drivebase_reset(struct db_drivebase_s *db, uint64_t now_us)
{
  int rc = db_servo_reset(&db->servo[DB_SIDE_LEFT], now_us);
  if (rc < 0) return rc;
  rc = db_servo_reset(&db->servo[DB_SIDE_RIGHT], now_us);
  if (rc < 0) return rc;

  /* Stop both axes passively and reset PID/trajectory state. */

  db_aggregate_control_reset(&db->control[DB_AXIS_DISTANCE]);
  db_aggregate_control_reset(&db->control[DB_AXIS_HEADING]);

  /* Capture the raw encoder-derived avg/heading as the new origin so
   * the next get_state returns 0/0 (Issue #113).
   */

  db_drivebase_raw_state(db, &db->distance_origin_mm,
                             &db->angle_origin_mdeg);

  db->distance_mm      = 0;
  db->drive_speed_mmps = 0;
  db->angle_mdeg       = 0;
  db->turn_rate_dps    = 0;
  db->active_command   = DRIVEBASE_ACTIVE_NONE;
  db->done             = true;
  db->stalled          = false;
  db->last_tick_us     = now_us;

  /* Phase 3b (#148) — fresh origin means publish should read 0 mdeg
   * once the IMU comes online; capture if possible, otherwise drop
   * any stale baseline so the next command-start latch starts fresh.
   */

  if (gyro_origin_capturable(db, now_us))
    {
      db->gyro_origin_mdeg  = db_imu_get_heading_mdeg(db->imu);
      db->gyro_origin_valid = true;
    }
  else
    {
      db->gyro_origin_valid = false;
    }
  return 0;
}

void db_drivebase_set_origin(struct db_drivebase_s *db,
                             int32_t distance_mm, int32_t angle_mdeg,
                             uint64_t now_us)
{
  int32_t raw_dist;
  int32_t raw_heading;
  db_drivebase_raw_state(db, &raw_dist, &raw_heading);

  db->distance_origin_mm = raw_dist    - distance_mm;
  db->angle_origin_mdeg  = raw_heading - angle_mdeg;

  db->distance_mm = distance_mm;
  db->angle_mdeg  = angle_mdeg;

  /* Phase 3b (#148) — re-evaluate the gyro origin so the published
   * heading and the PID input stay coherent with the new motor-side
   * origin.  See db_drivebase_set_use_gyro() for the full rationale.
   * When the IMU is not ready (capturable false) we explicitly drop
   * any stale baseline so a later command-start latch starts fresh.
   */

  if (gyro_origin_capturable(db, now_us))
    {
      db->gyro_origin_mdeg = db_imu_get_heading_mdeg(db->imu) -
                             (int64_t)angle_mdeg;
      db->gyro_origin_valid = true;
    }
  else
    {
      db->gyro_origin_valid = false;
    }
}

void db_drivebase_attach_imu(struct db_drivebase_s *db,
                             struct db_imu_s *imu)
{
  db->imu = imu;
  /* Any cached origin is now meaningless — wait for the next set_use_gyro,
   * set_origin, reset, or command-start latch to re-capture.
   */

  db->gyro_origin_valid = false;
}

int db_drivebase_set_use_gyro(struct db_drivebase_s *db, uint8_t mode,
                              uint64_t now_us)
{
  /* Validation first — keep the active check after, so unsupported modes
   * are rejected even while the drivebase is idle.
   */

  if (mode == DRIVEBASE_USE_GYRO_3D)
    {
      db->last_set_gyro_rc = -ENOSYS;
      return -ENOSYS;
    }
  if (mode != DRIVEBASE_USE_GYRO_NONE && mode != DRIVEBASE_USE_GYRO_1D)
    {
      db->last_set_gyro_rc = -EINVAL;
      return -EINVAL;
    }

  /* Refuse to flip the source mid-motion: the heading PID's state
   * input would jump between encoder and gyro coordinates, blowing up
   * the I-term.  Caller's option is stop → set-gyro → resume.
   */

  if (db->active_command != DRIVEBASE_ACTIVE_NONE)
    {
      db->last_set_gyro_rc = -EBUSY;
      return -EBUSY;
    }

  /* Snapshot old state BEFORE mutating — needed for the same-mode-1D
   * no-op below (Codex BLOCKING #1, Issue #148 implementation review).
   */

  uint8_t old_mode  = db->use_gyro_requested;
  bool    old_valid = db->gyro_origin_valid;

  bool mode_changed = (old_mode != mode);
  db->use_gyro_requested = mode;

  /* I-acc reset only on actual mode change.  Re-issuing the same mode
   * is a no-op for the PID memory; the origin re-evaluation below still
   * runs (subject to the same-mode preservation guard) so a stale
   * baseline can't survive a same-mode re-issue from an invalid state.
   */

  if (mode_changed)
    {
      db_aggregate_control_reset(&db->control[DB_AXIS_HEADING]);
    }

  /* Origin re-snapshot / invalidation (Codex 6th + 7th review,
   *   amended after implementation-round Codex BLOCKING #1).
   *
   *   - mode == NONE              → drop any cached origin so a future
   *                                  re-enable can't silently pick up
   *                                  a stale baseline.
   *   - same mode 1D & valid &
   *     capturable                → KEEP the existing origin as-is.
   *                                  Re-deriving from `db->angle_mdeg`
   *                                  would compute the baseline from
   *                                  the encoder publish value while
   *                                  the user-visible heading is
   *                                  `raw - old_origin` (gyro-relative),
   *                                  causing publish to snap.
   *   - capturable                → snapshot vs encoder publish so the
   *                                  next gyro publish lands at the
   *                                  same place the encoder publish
   *                                  was already showing.
   *   - else                      → IMU not ready; drop the cache and
   *                                  defer to the first command-start
   *                                  latch.
   */

  if (mode == DRIVEBASE_USE_GYRO_NONE)
    {
      db->gyro_origin_valid = false;
    }
  else if (!mode_changed && mode == DRIVEBASE_USE_GYRO_1D && old_valid &&
           gyro_origin_capturable(db, now_us))
    {
      /* No-op: preserve the existing origin so publish doesn't jump. */
    }
  else if (gyro_origin_capturable(db, now_us))
    {
      db->gyro_origin_mdeg =
          db_imu_get_heading_mdeg(db->imu) - (int64_t)db->angle_mdeg;
      db->gyro_origin_valid = true;
    }
  else
    {
      db->gyro_origin_valid = false;
    }

  db->last_set_gyro_rc = 0;
  return 0;
}

/****************************************************************************
 * Private Helpers — collect state from per-side servos
 ****************************************************************************/

struct db_agg_state_s
{
  int64_t  distance_x_mdeg;       /* (sL + sR) / 2  in motor mdeg       */
  int32_t  distance_v_mdegps;     /* (vL + vR) / 2                       */
  int64_t  heading_x_mdeg;        /* (sR - sL) / 2  in motor mdeg       */
  int32_t  heading_v_mdegps;
};

static void collect_aggregate_state(const struct db_drivebase_s *db,
                                    struct db_agg_state_s *out)
{
  struct db_servo_status_s sL, sR;
  db_servo_get_status(&db->servo[DB_SIDE_LEFT],  &sL);
  db_servo_get_status(&db->servo[DB_SIDE_RIGHT], &sR);

  out->distance_x_mdeg   = (sL.act_x_mdeg + sR.act_x_mdeg) / 2;
  out->distance_v_mdegps = (sL.act_v_mdegps + sR.act_v_mdegps) / 2;
  out->heading_x_mdeg    = (sR.act_x_mdeg - sL.act_x_mdeg) / 2;
  out->heading_v_mdegps  = (sR.act_v_mdegps - sL.act_v_mdegps) / 2;
}

/****************************************************************************
 * Private Helpers — Phase 3b (#148) gyro injection
 ****************************************************************************/

/* Lock in the latched mode at the start of a motion.  Capturable ⇒
 * latched = requested.  Else ⇒ latched = NONE (encoder-only this
 * motion).  Per-tick injection consults the latched value so an IMU
 * stall mid-motion can fall back to encoder without flipping the
 * heading PID's intent mid-run.
 */

static void db_drivebase_latch_use_gyro(struct db_drivebase_s *db,
                                        uint64_t now_us)
{
  if (gyro_origin_capturable(db, now_us))
    {
      db->use_gyro_latched = db->use_gyro_requested;
    }
  else
    {
      db->use_gyro_latched = DRIVEBASE_USE_GYRO_NONE;
    }
}

/* Capture pre-motion aggregate state for the entry points and, when
 * `do_latch` is true, also lock in use_gyro_latched and snapshot the
 * gyro origin (in-place only when no valid origin exists yet).  Per-
 * tick callers pass do_latch=false: they only fold the IMU heading
 * into state->heading_x_mdeg when all runtime guards are satisfied,
 * otherwise the encoder value collected by collect_aggregate_state
 * passes through untouched (encoder fallback during transient IMU
 * stalls or calibration loss).
 */

static void db_drivebase_capture_start_heading(
    struct db_drivebase_s *db,
    struct db_agg_state_s *state,
    uint64_t now_us,
    bool do_latch)
{
  if (do_latch)
    {
      db_drivebase_latch_use_gyro(db, now_us);

      /* Origin in-place snapshot: only when there is no valid baseline
       * yet.  An in-flight retarget (drive_forever re-arm in particular)
       * must NOT clobber the origin captured at the original command-
       * start, or the publish basis would jump under the user.
       */

      if (!db->gyro_origin_valid &&
          db->use_gyro_latched == DRIVEBASE_USE_GYRO_1D &&
          db->imu != NULL)
        {
          db->gyro_origin_mdeg =
              db_imu_get_heading_mdeg(db->imu) - (int64_t)db->angle_mdeg;
          db->gyro_origin_valid = true;
        }
    }

  collect_aggregate_state(db, state);

  /* Fold the IMU heading into the PID state input.  All guards re-
   * checked per call so any transient (stale sample, lost cal) causes
   * a one-tick encoder fallback rather than running on a stale IMU
   * value.  Latched is the *intent*; the guards are the *trust check*.
   */

  if (db->use_gyro_latched == DRIVEBASE_USE_GYRO_1D &&
      db->imu != NULL &&
      db->gyro_origin_valid &&
      db_imu_is_calibrated(db->imu) &&
      !db_imu_is_stale(db->imu, now_us,
                       DB_IMU_DEFAULT_STALE_THRESHOLD_US))
    {
      int64_t raw = db_imu_get_heading_mdeg(db->imu);
      int64_t rel = raw - db->gyro_origin_mdeg;
      state->heading_x_mdeg =
          heading_mdeg_to_state_mdeg(rel, db->wheel_d_um,
                                     db->axle_t_um);
      /* heading_v_mdegps stays encoder-derived: LSM6DSL ODR/quantisation
       * makes the per-tick gyro-rate-only D-term noisy.  Following the
       * pybricks pattern, only the position term flips to IMU; the rate
       * term keeps the encoder differential.
       */
    }
}

/****************************************************************************
 * Public Functions — drive primitives
 ****************************************************************************/

/* Common helper: configure both axes for a position-mode move.  The
 * caller specifies the *primary* axis's delta + on_completion; the
 * auxiliary axis is set to HOLD at its current state.  Velocity
 * limits come from the per-axis trajectory limit tables; the primary
 * axis can override v_max via the speed argument (#137).
 */

static int setup_position_move(struct db_drivebase_s *db,
                               uint64_t now_us,
                               int64_t distance_delta_mdeg,
                               int64_t heading_delta_mdeg,
                               int32_t v_distance_override_mdegps,
                               int32_t v_heading_override_mdegps,
                               enum db_axis_e primary,
                               uint8_t on_completion)
{
  struct db_agg_state_s s;
  db_drivebase_capture_start_heading(db, &s, now_us, true);

  const struct db_traj_limits_s *dl =
      db_settings_distance_limits(db->wheel_d_um);
  const struct db_traj_limits_s *hl =
      db_settings_heading_limits(db->wheel_d_um, db->axle_t_um);

  int32_t v_d = v_distance_override_mdegps ? v_distance_override_mdegps
                                            : dl->v_max_mdegps;
  int32_t v_h = v_heading_override_mdegps  ? v_heading_override_mdegps
                                            : hl->v_max_mdegps;

  uint8_t comp_d = (primary == DB_AXIS_DISTANCE) ?
                   on_completion : DRIVEBASE_ON_COMPLETION_HOLD;
  uint8_t comp_h = (primary == DB_AXIS_HEADING)  ?
                   on_completion : DRIVEBASE_ON_COMPLETION_HOLD;

  if (distance_delta_mdeg == 0 && primary != DB_AXIS_DISTANCE)
    {
      /* Pure turn (no distance change requested): HOLD distance at the
       * current encoder-derived state, do not drive it through a
       * zero-length trajectory (which would still latch SMART_continue
       * etc. unnecessarily).
       */

      db_aggregate_control_drive_hold(&db->control[DB_AXIS_DISTANCE],
                                      now_us, s.distance_x_mdeg);
    }
  else
    {
      db_aggregate_control_drive_position(
        &db->control[DB_AXIS_DISTANCE], now_us,
        s.distance_x_mdeg, distance_delta_mdeg,
        v_d, dl->accel_mdegps2, dl->decel_mdegps2, comp_d);
    }

  if (heading_delta_mdeg == 0 && primary != DB_AXIS_HEADING)
    {
      db_aggregate_control_drive_hold(&db->control[DB_AXIS_HEADING],
                                      now_us, s.heading_x_mdeg);
    }
  else
    {
      db_aggregate_control_drive_position(
        &db->control[DB_AXIS_HEADING], now_us,
        s.heading_x_mdeg, heading_delta_mdeg,
        v_h, hl->accel_mdegps2, hl->decel_mdegps2, comp_h);
    }

  return 0;
}

int db_drivebase_drive_straight(struct db_drivebase_s *db,
                                uint64_t now_us,
                                int32_t distance_mm,
                                int32_t speed_mmps,
                                uint8_t on_completion)
{
  int64_t dist_delta_mdeg = mm_to_motor_mdeg(distance_mm, db->wheel_d_um);

  int32_t v_override = 0;
  if (speed_mmps != 0)
    {
      int32_t v_abs = (speed_mmps < 0) ? -speed_mmps : speed_mmps;
      v_override = db_angle_mmps_to_mdegps(v_abs, db->wheel_d_um);
    }

  db->active_command = DRIVEBASE_ACTIVE_STRAIGHT;
  return setup_position_move(db, now_us,
                             dist_delta_mdeg, 0,
                             v_override, 0,
                             DB_AXIS_DISTANCE, on_completion);
}

int db_drivebase_turn(struct db_drivebase_s *db,
                      uint64_t now_us,
                      int32_t angle_deg,
                      int32_t turn_rate_dps,
                      uint8_t on_completion)
{
  int64_t heading_delta_mdeg =
      heading_mdeg_to_state_mdeg((int64_t)angle_deg * 1000,
                                 db->wheel_d_um, db->axle_t_um);

  int32_t v_override = 0;
  if (turn_rate_dps != 0)
    {
      int32_t r_abs = (turn_rate_dps < 0) ? -turn_rate_dps : turn_rate_dps;
      /* turn_rate_dps is heading angular rate (deg/s).  Convert to
       * state-space velocity (motor-mdeg/s) via the same axle/wheel
       * ratio as the position delta: v_state = r_dps × axle_t / wheel_d
       * × 1000 (the 1000 reconciles deg/s vs mdeg/s).
       */

      v_override = (int32_t)((int64_t)r_abs * 1000 *
                              (int64_t)db->axle_t_um / db->wheel_d_um);
    }

  db->active_command = DRIVEBASE_ACTIVE_TURN;
  return setup_position_move(db, now_us,
                             0, heading_delta_mdeg,
                             0, v_override,
                             DB_AXIS_HEADING, on_completion);
}

int db_drivebase_drive_curve(struct db_drivebase_s *db,
                             uint64_t now_us,
                             int32_t radius_mm,
                             int32_t angle_deg,
                             uint8_t on_completion)
{
  /* Curve: travel arc of `angle_deg` along a circle of `radius_mm`.
   *   centre travel  (mm)         = R · |angle_rad|   sign = sgn(angle)
   *   heading change (deg)        = angle_deg
   *
   * Stationary-axis fallbacks (Issue #144 B2): degenerate inputs route
   * through the dedicated turn/straight paths.  This avoids planning a
   * direction=0 trajectory on one axis (which db_trajectory_stretch_to
   * _total cannot stretch) and the matching COAST/BRAKE early-done
   * propagation that would otherwise stop the active axis prematurely.
   */

  if (radius_mm == 0)
    {
      return db_drivebase_turn(db, now_us, angle_deg, 0, on_completion);
    }
  if (angle_deg == 0)
    {
      db->active_command = DRIVEBASE_ACTIVE_NONE;
      return 0;
    }

  int32_t dl_avg_mm = (int32_t)((int64_t)radius_mm * angle_deg *
                                 DB_PI_NUM / DB_PI_DEN / 180);
  int64_t dist_delta_mdeg = mm_to_motor_mdeg(dl_avg_mm, db->wheel_d_um);

  int64_t heading_delta_mdeg =
      heading_mdeg_to_state_mdeg((int64_t)angle_deg * 1000,
                                 db->wheel_d_um, db->axle_t_um);

  /* Rounding can collapse one delta to zero even though the original
   * (radius, angle) pair is non-degenerate.  Route to the single-axis
   * primitives in that case so the auxiliary axis is held via HOLD.
   */

  if (dist_delta_mdeg == 0 && heading_delta_mdeg != 0)
    {
      return db_drivebase_turn(db, now_us, angle_deg, 0, on_completion);
    }
  if (heading_delta_mdeg == 0 && dist_delta_mdeg != 0)
    {
      return db_drivebase_drive_straight(db, now_us, dl_avg_mm, 0,
                                         on_completion);
    }
  if (dist_delta_mdeg == 0 && heading_delta_mdeg == 0)
    {
      db->active_command = DRIVEBASE_ACTIVE_NONE;
      return 0;
    }

  db->active_command = DRIVEBASE_ACTIVE_CURVE;

  /* For curves both axes are "primary" in the sense that both move on
   * a finite trajectory.  Apply the user's on_completion to the
   * distance axis (the dominant travel) and use the same on the
   * heading axis so SMART continuation works on both.
   */

  struct db_agg_state_s s;
  db_drivebase_capture_start_heading(db, &s, now_us, true);

  const struct db_traj_limits_s *dl =
      db_settings_distance_limits(db->wheel_d_um);
  const struct db_traj_limits_s *hl =
      db_settings_heading_limits(db->wheel_d_um, db->axle_t_um);

  db_aggregate_control_drive_position(
      &db->control[DB_AXIS_DISTANCE], now_us,
      s.distance_x_mdeg, dist_delta_mdeg,
      dl->v_max_mdegps, dl->accel_mdegps2, dl->decel_mdegps2,
      on_completion);
  db_aggregate_control_drive_position(
      &db->control[DB_AXIS_HEADING], now_us,
      s.heading_x_mdeg, heading_delta_mdeg,
      hl->v_max_mdegps, hl->accel_mdegps2, hl->decel_mdegps2,
      on_completion);

  /* Phase 4 C: equalise trajectory durations so both axes complete
   * simultaneously.  Without this step, a large-radius curve has the
   * heading axis finishing well ahead of the distance axis, after
   * which the heading PID holds yaw while distance continues straight
   * — visible as a tangent line at the end of the arc (Issue #144).
   *
   * Threshold gate: when the duration ratio is below
   * DB_TRAJECTORY_STRETCH_RATIO_THRESHOLD_MIL, the two trajectories
   * are similar enough that the tangent is barely visible AND
   * stretching the shorter trajectory invites integral wind-up on the
   * heading PID (regression observed at ratio ~1.25× on bench).
   *
   * Tmp-copy commit pattern (Issue #144 B3): build the stretched
   * trajectory on a stack copy, only overwrite the live trajectory on
   * success.  On failure (precondition violation, numerical solver
   * giving up, or residual gate trip inside the helper) leave the
   * trajectory unstretched and log a warning so the regression is
   * observable in dmesg.  Valid (radius, angle) inputs satisfy the
   * helper's preconditions, so this path is defensive only.
   *
   * Note (Issue #144 C4): the per-axis reference-time pause introduced
   * in #142 can desynchronise the two trajectories if only one axis
   * pauses mid-curve.  This is the existing #142 design and is
   * accepted for Phase 4; a drivebase-level shared pause would
   * require pybricks-style restructuring (out of scope here).
   */

  struct db_trajectory_s *trj_d =
      &db->control[DB_AXIS_DISTANCE].trajectory;
  struct db_trajectory_s *trj_h =
      &db->control[DB_AXIS_HEADING].trajectory;

  struct db_trajectory_s *follower_p = NULL;
  uint64_t                target_dt  = 0;
  uint64_t                short_dt   = 0;
  enum db_axis_e          follower_axis = DB_AXIS_DISTANCE;

  if (trj_d->total_dt_us > trj_h->total_dt_us)
    {
      follower_p    = trj_h;
      follower_axis = DB_AXIS_HEADING;
      target_dt     = trj_d->total_dt_us;
      short_dt      = trj_h->total_dt_us;
    }
  else if (trj_h->total_dt_us > trj_d->total_dt_us)
    {
      follower_p    = trj_d;
      follower_axis = DB_AXIS_DISTANCE;
      target_dt     = trj_h->total_dt_us;
      short_dt      = trj_d->total_dt_us;
    }

  /* Ratio gate: skip stretch when target_dt / short_dt < threshold.
   * `target_dt * 1000 < short_dt * threshold_mil` keeps the comparison
   * in integers and matches the comment.  short_dt > 0 because we set
   * follower_p only when one duration strictly exceeds the other and
   * both came from db_trajectory_init_position with non-zero deltas.
   */

  if (follower_p != NULL && short_dt > 0 &&
      target_dt * 1000ULL <
        short_dt * (uint64_t)DB_TRAJECTORY_STRETCH_RATIO_THRESHOLD_MIL)
    {
      follower_p = NULL;
    }

  if (follower_p != NULL)
    {
      struct db_trajectory_s tmp = *follower_p;
      int rc = db_trajectory_stretch_to_total(&tmp, target_dt);
      if (rc == 0)
        {
          *follower_p = tmp;
        }
      else
        {
          syslog(LOG_WARNING,
                 "drivebase: curve stretch failed axis=%d rc=%d "
                 "(R=%ld a=%ld dist_dt=%llu hdg_dt=%llu); falling back "
                 "to unstretched trajectories\n",
                 (int)follower_axis, rc, (long)radius_mm, (long)angle_deg,
                 (unsigned long long)trj_d->total_dt_us,
                 (unsigned long long)trj_h->total_dt_us);
        }
    }

  return 0;
}

int db_drivebase_drive_arc_distance(struct db_drivebase_s *db,
                                    uint64_t now_us,
                                    int32_t radius_mm,
                                    int32_t distance_mm,
                                    uint8_t on_completion)
{
  if (radius_mm == 0)
    {
      return db_drivebase_drive_straight(db, now_us, distance_mm,
                                         0, on_completion);
    }

  int32_t angle_mdeg = (int32_t)((int64_t)distance_mm * 1000 * 180 *
                                  DB_PI_DEN /
                                  ((int64_t)radius_mm * DB_PI_NUM));

  return db_drivebase_drive_curve(db, now_us, radius_mm,
                                  angle_mdeg / 1000, on_completion);
}

int db_drivebase_drive_forever(struct db_drivebase_s *db,
                               uint64_t now_us,
                               int32_t speed_mmps,
                               int32_t turn_rate_dps)
{
  /* Phase 3b (#148) Codex BLOCKING #2: drive_forever supports
   * in-flight retargeting (aggregate_control_drive_forever does NOT
   * reset PID/trajectory).  Re-latching on every retarget would let a
   * transient IMU stall flip use_gyro_latched to NONE mid-motion while
   * the PID accumulator stays loaded — exactly the source-flip race
   * `set_use_gyro` rejects with -EBUSY.  Only latch on the FIRST entry
   * (active_command != FOREVER yet); retarget calls keep the existing
   * latched mode and origin untouched.
   */

  bool is_retarget = (db->active_command == DRIVEBASE_ACTIVE_FOREVER);
  struct db_agg_state_s s;
  db_drivebase_capture_start_heading(db, &s, now_us, !is_retarget);

  int32_t v_dist_mdegps = db_angle_mmps_to_mdegps(speed_mmps,
                                                  db->wheel_d_um);
  int32_t v_hdg_mdegps  = (int32_t)((int64_t)turn_rate_dps * 1000 *
                                    (int64_t)db->axle_t_um /
                                    db->wheel_d_um);

  const struct db_traj_limits_s *dl =
      db_settings_distance_limits(db->wheel_d_um);
  const struct db_traj_limits_s *hl =
      db_settings_heading_limits(db->wheel_d_um, db->axle_t_um);

  db_aggregate_control_drive_forever(&db->control[DB_AXIS_DISTANCE],
                                     now_us,
                                     s.distance_x_mdeg,
                                     v_dist_mdegps,
                                     dl->accel_mdegps2);
  db_aggregate_control_drive_forever(&db->control[DB_AXIS_HEADING],
                                     now_us,
                                     s.heading_x_mdeg,
                                     v_hdg_mdegps,
                                     hl->accel_mdegps2);

  db->active_command = DRIVEBASE_ACTIVE_FOREVER;
  return 0;
}

int db_drivebase_stop(struct db_drivebase_s *db,
                      uint64_t now_us, uint8_t on_completion)
{
  struct db_agg_state_s s;
  db_drivebase_capture_start_heading(db, &s, now_us, false);

  db->active_command = DRIVEBASE_ACTIVE_STOP;

  db_aggregate_control_stop(&db->control[DB_AXIS_DISTANCE], now_us,
                            s.distance_x_mdeg, on_completion);
  db_aggregate_control_stop(&db->control[DB_AXIS_HEADING],  now_us,
                            s.heading_x_mdeg, on_completion);

  /* Apply the immediate actuation to the motors so the user-visible
   * effect is instantaneous; subsequent ticks confirm via the
   * compose path (which will see latched_passive_done for COAST/BRAKE
   * or zero-length HOLD trajectory active).
   */

  switch (on_completion)
    {
      case DRIVEBASE_ON_COMPLETION_COAST:
      case DRIVEBASE_ON_COMPLETION_COAST_SMART:
        db_servo_apply(&db->servo[DB_SIDE_LEFT],  0,
                       DRIVEBASE_ON_COMPLETION_COAST);
        db_servo_apply(&db->servo[DB_SIDE_RIGHT], 0,
                       DRIVEBASE_ON_COMPLETION_COAST);
        break;
      case DRIVEBASE_ON_COMPLETION_BRAKE:
      case DRIVEBASE_ON_COMPLETION_BRAKE_SMART:
        db_servo_apply(&db->servo[DB_SIDE_LEFT],  0,
                       DRIVEBASE_ON_COMPLETION_BRAKE);
        db_servo_apply(&db->servo[DB_SIDE_RIGHT], 0,
                       DRIVEBASE_ON_COMPLETION_BRAKE);
        break;
      default:
        /* HOLD: don't touch motors here — the next tick will compose
         * the zero-length HOLD trajectory output normally.
         */
        break;
    }
  return 0;
}

/****************************************************************************
 * Public Functions — per-tick update + state snapshot
 ****************************************************************************/

int db_drivebase_update(struct db_drivebase_s *db, uint64_t now_us)
{
  /* 1. Per-servo encoder + observer update. */

  int rc = db_servo_update(&db->servo[DB_SIDE_LEFT],  now_us);
  if (rc < 0) return rc;
  rc = db_servo_update(&db->servo[DB_SIDE_RIGHT], now_us);
  if (rc < 0) return rc;

  /* 2. Collect aggregate state from per-servo cached x/v. */

  struct db_agg_state_s state;
  db_drivebase_capture_start_heading(db, &state, now_us, false);

  /* 3. Compute dt_ms.  Like pre-#141 servo, prefer the nominal tick
   * (Issue #120) so PID gains stay tick-proportional under jitter.
   */

  uint32_t dt_ms = db->tick_ms;
  if (db->last_tick_us != 0 && now_us > db->last_tick_us)
    {
      uint32_t measured = (uint32_t)((now_us - db->last_tick_us) / 1000);
      if (measured > 0 && measured < (db->tick_ms * 4))
        {
          /* keep the nominal tick — measured is just a sanity gate. */
        }
    }
  db->last_tick_us = now_us;

  /* 4. Run both aggregate PIDs. */

  struct db_aggregate_output_s out_d;
  struct db_aggregate_output_s out_h;
  db_aggregate_control_update(&db->control[DB_AXIS_DISTANCE], now_us,
                              state.distance_x_mdeg,
                              state.distance_v_mdegps,
                              dt_ms, &out_d);
  db_aggregate_control_update(&db->control[DB_AXIS_HEADING],  now_us,
                              state.heading_x_mdeg,
                              state.heading_v_mdegps,
                              dt_ms, &out_h);

  /* 5. COAST/BRAKE propagation (pybricks-style): if EITHER axis
   * decides COAST or BRAKE this tick, stop both axes and apply the
   * matching passive actuation to both motors.  HOLD does not
   * propagate (auxiliary axes always run HOLD).
   */

  uint8_t propagate = 0;
  if (out_d.actuation == DRIVEBASE_ON_COMPLETION_COAST ||
      out_h.actuation == DRIVEBASE_ON_COMPLETION_COAST)
    {
      propagate = DRIVEBASE_ON_COMPLETION_COAST;
    }
  else if (out_d.actuation == DRIVEBASE_ON_COMPLETION_BRAKE ||
           out_h.actuation == DRIVEBASE_ON_COMPLETION_BRAKE)
    {
      propagate = DRIVEBASE_ON_COMPLETION_BRAKE;
    }

  if (propagate != 0)
    {
      /* Latch the non-firing axis too so it stays passive on the next
       * tick (otherwise its HOLD trajectory would continue driving
       * duty against the freshly-applied COAST/BRAKE).
       */

      db_aggregate_control_stop(&db->control[DB_AXIS_DISTANCE], now_us,
                                state.distance_x_mdeg, propagate);
      db_aggregate_control_stop(&db->control[DB_AXIS_HEADING],  now_us,
                                state.heading_x_mdeg, propagate);

      db_servo_apply(&db->servo[DB_SIDE_LEFT],  0, propagate);
      db_servo_apply(&db->servo[DB_SIDE_RIGHT], 0, propagate);
    }
  else
    {
      /* 6. Compose per-side duty (spike-nx convention: state_heading
       *    = (sR - sL) / 2, so heading-positive → R faster → CCW yaw).
       *
       * Phase 6 Step 6.2 (#152) — per-motor friction FF (Plan D1+D4):
       * kV/kA already lives inside the aggregate PID output via the
       * duty_ff plumbing (Step 6.1).  kS is non-linear in the compose
       * (`sign(D)±sign(H)` differs from `sign(D±H)`, e.g. a pivot
       * `vD=vH>0` makes per-axis kS apply 2·kS to one wheel and 0 to
       * the other, vs. the correct kS to each), so kS must apply
       * per-motor AFTER the compose.  Compute `vL_ref = D - H` and
       * `vR_ref = D + H` from the per-axis ref_v exports (added in
       * Step 6.1), drive the hysteresis state machine independently
       * per side, then add `sign·kS/2` before the per-side clamp.
       *
       * The /2 attenuation mirrors pybricks (lib/pbio/src/observer.c
       * L250: `torque_friction / 2 * sign(rate_ref)`) and keeps the
       * entry rail-step small enough that anti-windup behaves and
       * final-approach decel does not overshoot the target.
       */

      int32_t vL_ref_mdegps = out_d.ref_v_mdegps - out_h.ref_v_mdegps;
      int32_t vR_ref_mdegps = out_d.ref_v_mdegps + out_h.ref_v_mdegps;

      int32_t enter = db->ff_motor->v_hyst_enter_mdegps;
      int32_t exit  = db->ff_motor->v_hyst_exit_mdegps;

      int8_t signL = ff_sign_with_hysteresis(vL_ref_mdegps,
                                             db->ff_state_left.sign_v_held,
                                             enter, exit);
      int8_t signR = ff_sign_with_hysteresis(vR_ref_mdegps,
                                             db->ff_state_right.sign_v_held,
                                             enter, exit);
      db->ff_state_left.sign_v_held  = signL;
      db->ff_state_right.sign_v_held = signR;

      int32_t kS_half = db->ff_motor->kS / 2;   /* pybricks /2 attenuation */
      int32_t kS_L    = (int32_t)signL * kS_half;
      int32_t kS_R    = (int32_t)signR * kS_half;

      int32_t left_duty  = clamp_i32(out_d.duty - out_h.duty + kS_L,
                                     -10000, 10000);
      int32_t right_duty = clamp_i32(out_d.duty + out_h.duty + kS_R,
                                     -10000, 10000);

      db_servo_apply(&db->servo[DB_SIDE_LEFT],  left_duty,
                     DRIVEBASE_ON_COMPLETION_HOLD);
      db_servo_apply(&db->servo[DB_SIDE_RIGHT], right_duty,
                     DRIVEBASE_ON_COMPLETION_HOLD);
    }

  /* 7. Refresh cached aggregate state from the freshly-updated per-
   *    servo positions (computed once more to capture this tick).
   */

  struct db_servo_status_s sL, sR;
  db_servo_get_status(&db->servo[DB_SIDE_LEFT],  &sL);
  db_servo_get_status(&db->servo[DB_SIDE_RIGHT], &sR);

  int32_t lL_mm = db_angle_mdeg_to_mm(sL.act_x_mdeg, db->wheel_d_um);
  int32_t lR_mm = db_angle_mdeg_to_mm(sR.act_x_mdeg, db->wheel_d_um);
  int32_t avg_mm  = (lL_mm + lR_mm) / 2;
  int32_t diff_mm = lR_mm - lL_mm;

  db->distance_mm      = avg_mm - db->distance_origin_mm;
  db->angle_mdeg       = diff_mm_to_heading_mdeg(diff_mm,
                                                 db->axle_t_um) -
                         db->angle_origin_mdeg;

  int32_t vL_mmps = db_angle_mdegps_to_mmps(sL.act_v_mdegps,
                                            db->wheel_d_um);
  int32_t vR_mmps = db_angle_mdegps_to_mmps(sR.act_v_mdegps,
                                            db->wheel_d_um);
  db->drive_speed_mmps = (vL_mmps + vR_mmps) / 2;
  int32_t v_diff = vR_mmps - vL_mmps;
  int32_t v_diff_mdegps = diff_mm_to_heading_mdeg(v_diff,
                                                  db->axle_t_um);
  db->turn_rate_dps    = v_diff_mdegps / 1000;

  /* Aggregate done: both axes' PIDs report done.  This naturally
   * never fires for DRIVE_FOREVER (trajectory.done stays false).
   */

  db->done    = out_d.done && out_h.done;
  db->stalled = sL.stalled || sR.stalled;

  if (db->done)
    {
      db->active_command = DRIVEBASE_ACTIVE_NONE;
    }
  return 0;
}

void db_drivebase_get_state(const struct db_drivebase_s *db,
                            struct drivebase_state_s *out)
{
  out->distance_mm       = db->distance_mm;
  out->drive_speed_mmps  = db->drive_speed_mmps;
  out->angle_mdeg        = db->angle_mdeg;
  out->turn_rate_dps     = db->turn_rate_dps;
  out->tick_seq          = 0;     /* daemon FSM sets this               */
  out->is_done           = db->done;
  out->is_stalled        = db->stalled;
  out->active_command    = db->active_command;
  out->reserved          = 0;
}

bool db_drivebase_is_done(const struct db_drivebase_s *db)
{
  return db->done;
}
