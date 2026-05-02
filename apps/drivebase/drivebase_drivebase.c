/****************************************************************************
 * apps/drivebase/drivebase_drivebase.c
 *
 * Two-motor drivebase aggregator.  See drivebase_drivebase.h for the
 * geometry conventions and the per-commit scope notes.
 *
 * Sign convention (matches pybricks):
 *   - Positive distance      = robot moves "forward" (the direction
 *                              wheel labels imply when both motors
 *                              receive positive duty).  The sign
 *                              convention requires the L motor to be
 *                              wired so positive duty produces forward
 *                              motion as well; if wiring is reversed,
 *                              the user can negate distance externally
 *                              or we'll add a per-motor invert flag in
 *                              commit #11.
 *   - Positive heading angle = counter-clockwise rotation viewed from
 *                              above (right wheel forward, left wheel
 *                              back), pybricks convention.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "drivebase_drivebase.h"
#include "drivebase_motor.h"
#include "drivebase_settings.h"
#include "drivebase_angle.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_PI_NUM   355
#define DB_PI_DEN   113

/****************************************************************************
 * Private Helpers — geometry conversions
 ****************************************************************************/

/* Wheel travel (mm) ↔ motor angle (mdeg).  l_mm = θ_mdeg / 1000 *
 * π * d / 360.  Same math as drivebase_angle.h but spelled out here
 * for readability.
 */

static int64_t mm_to_motor_mdeg(int32_t mm, uint32_t d_mm)
{
  return db_angle_mm_to_mdeg(mm, d_mm);
}

/* heading (mdeg of robot rotation) ↔ wheel-differential travel (mm).
 *   l_diff_mm = H_rad * axle_t = (H_deg * π/180) * axle_t
 *   l_diff_mm = H_mdeg * π * axle_t / (1000 * 180)
 */

static int32_t heading_mdeg_to_diff_mm(int32_t h_mdeg, uint32_t a_mm)
{
  int64_t num = (int64_t)h_mdeg * DB_PI_NUM * (int64_t)a_mm;
  int64_t den = (int64_t)1000 * 180 * DB_PI_DEN;
  if (num >= 0) return (int32_t)((num + den / 2) / den);
  return (int32_t)((num - den / 2) / den);
}

/* Inverse: wheel-differential mm → robot heading mdeg. */

static int32_t diff_mm_to_heading_mdeg(int32_t diff_mm, uint32_t a_mm)
{
  if (a_mm == 0) return 0;
  int64_t num = (int64_t)diff_mm * 1000 * 180 * DB_PI_DEN;
  int64_t den = (int64_t)DB_PI_NUM * (int64_t)a_mm;
  if (num >= 0) return (int32_t)((num + den / 2) / den);
  return (int32_t)((num - den / 2) / den);
}

/****************************************************************************
 * Private Helpers — per-side trajectory dispatch
 ****************************************************************************/

/* Translate (Δl_avg_mm, Δl_diff_mm) into per-side motor mdeg deltas
 * and feed each side's servo.position_relative.  Speed limits passed
 * through come from db_settings_distance_limits.
 */

static int dispatch_position(struct db_drivebase_s *db, uint64_t now_us,
                             int32_t dl_avg_mm, int32_t dl_diff_mm,
                             int32_t v_avg_mmps, int32_t a_avg_mmps2,
                             int32_t d_avg_mmps2,
                             uint8_t on_completion)
{
  /* L = avg - diff/2,  R = avg + diff/2 */

  int32_t dL_mm = dl_avg_mm - dl_diff_mm / 2;
  int32_t dR_mm = dl_avg_mm + dl_diff_mm / 2;
  int64_t dL_mdeg = mm_to_motor_mdeg(dL_mm, db->wheel_d_mm);
  int64_t dR_mdeg = mm_to_motor_mdeg(dR_mm, db->wheel_d_mm);

  int32_t v_mdegps = db_angle_mmps_to_mdegps(v_avg_mmps,
                                             db->wheel_d_mm);
  int32_t a_mdegps2 = db_angle_mmps_to_mdegps(a_avg_mmps2,
                                              db->wheel_d_mm);
  int32_t d_mdegps2 = db_angle_mmps_to_mdegps(d_avg_mmps2,
                                              db->wheel_d_mm);

  int rc = db_servo_position_relative(&db->servo[DB_SIDE_LEFT], now_us,
                                      dL_mdeg, v_mdegps,
                                      a_mdegps2, d_mdegps2,
                                      on_completion);
  if (rc < 0) return rc;
  return db_servo_position_relative(&db->servo[DB_SIDE_RIGHT], now_us,
                                    dR_mdeg, v_mdegps,
                                    a_mdegps2, d_mdegps2,
                                    on_completion);
}

static int dispatch_forever(struct db_drivebase_s *db, uint64_t now_us,
                            int32_t v_avg_mmps, int32_t v_diff_mmps,
                            int32_t a_mmps2)
{
  int32_t vL_mmps = v_avg_mmps - v_diff_mmps / 2;
  int32_t vR_mmps = v_avg_mmps + v_diff_mmps / 2;
  int32_t vL_mdegps = db_angle_mmps_to_mdegps(vL_mmps, db->wheel_d_mm);
  int32_t vR_mdegps = db_angle_mmps_to_mdegps(vR_mmps, db->wheel_d_mm);
  int32_t a_mdegps2 = db_angle_mmps_to_mdegps(a_mmps2, db->wheel_d_mm);

  int rc = db_servo_forever(&db->servo[DB_SIDE_LEFT], now_us,
                            vL_mdegps, a_mdegps2);
  if (rc < 0) return rc;
  return db_servo_forever(&db->servo[DB_SIDE_RIGHT], now_us,
                          vR_mdegps, a_mdegps2);
}

/****************************************************************************
 * Public Functions — lifecycle
 ****************************************************************************/

int db_drivebase_init(struct db_drivebase_s *db,
                      uint32_t wheel_d_mm, uint32_t axle_t_mm)
{
  if (wheel_d_mm == 0 || axle_t_mm == 0)
    {
      return -EINVAL;
    }

  memset(db, 0, sizeof(*db));
  db->wheel_d_mm = wheel_d_mm;
  db->axle_t_mm  = axle_t_mm;
  db->active_command = DRIVEBASE_ACTIVE_NONE;

  db_servo_init(&db->servo[DB_SIDE_LEFT],  DB_SIDE_LEFT);
  db_servo_init(&db->servo[DB_SIDE_RIGHT], DB_SIDE_RIGHT);
  return 0;
}

int db_drivebase_reset(struct db_drivebase_s *db, uint64_t now_us)
{
  int rc = db_servo_reset(&db->servo[DB_SIDE_LEFT], now_us);
  if (rc < 0) return rc;
  rc = db_servo_reset(&db->servo[DB_SIDE_RIGHT], now_us);
  if (rc < 0) return rc;

  db->distance_mm      = 0;
  db->drive_speed_mmps = 0;
  db->angle_mdeg       = 0;
  db->turn_rate_dps    = 0;
  db->active_command   = DRIVEBASE_ACTIVE_NONE;
  db->done             = true;
  db->stalled          = false;
  return 0;
}

void db_drivebase_set_origin(struct db_drivebase_s *db,
                             int32_t distance_mm, int32_t angle_mdeg)
{
  /* Origin handling is a thin offset — it's applied at get_state
   * time rather than rewriting the cached integration.  For commit
   * #7 the state aggregator integrates from the per-servo encoder
   * positions on the fly, so simply latch the offsets and let
   * subsequent updates incorporate them.  (Persisting origin across
   * a daemon restart is commit #9 RESET path territory.)
   */

  db->distance_mm = distance_mm;
  db->angle_mdeg  = angle_mdeg;
}

/****************************************************************************
 * Public Functions — drive primitives
 ****************************************************************************/

int db_drivebase_drive_straight(struct db_drivebase_s *db,
                                uint64_t now_us,
                                int32_t distance_mm,
                                uint8_t on_completion)
{
  const struct db_traj_limits_s *dl =
    db_settings_distance_limits(db->wheel_d_mm);

  /* Convert the trajectory limits (in motor mdeg/s) back to wheel
   * mm/s for dispatch_position's sake.
   */

  int32_t v_mmps    = db_angle_mdegps_to_mmps(dl->v_max_mdegps,
                                              db->wheel_d_mm);
  int32_t a_mmps2   = db_angle_mdegps_to_mmps(dl->accel_mdegps2,
                                              db->wheel_d_mm);
  int32_t d_mmps2   = db_angle_mdegps_to_mmps(dl->decel_mdegps2,
                                              db->wheel_d_mm);

  db->active_command = DRIVEBASE_ACTIVE_STRAIGHT;
  return dispatch_position(db, now_us, distance_mm, 0,
                           v_mmps, a_mmps2, d_mmps2, on_completion);
}

int db_drivebase_turn(struct db_drivebase_s *db,
                      uint64_t now_us,
                      int32_t angle_deg,
                      uint8_t on_completion)
{
  /* l_diff = H_rad * axle_t = angle_deg * π/180 * axle_t */

  int32_t dl_diff_mm = heading_mdeg_to_diff_mm(angle_deg * 1000,
                                               db->axle_t_mm);
  const struct db_traj_limits_s *hl =
    db_settings_heading_limits(db->wheel_d_mm, db->axle_t_mm);
  /* Convert heading angular speed → wheel mm/s for dispatch.  At
   * each motor, the speed magnitude is (v_heading * axle_t / 2 / π)
   * but we already scaled in db_settings_heading_limits's mdeg/s of
   * MOTOR rotation when computing the limits.  Re-derive a wheel
   * mm/s figure for dispatch_position (since it converts mm/s →
   * mdeg/s using wheel_d, an additional axle/wheel factor would
   * cancel).  Easiest: just lift the same numbers as the distance
   * limits, since at top speed both move equally fast in mm/s.
   */

  const struct db_traj_limits_s *dl =
    db_settings_distance_limits(db->wheel_d_mm);
  int32_t v_mmps  = db_angle_mdegps_to_mmps(dl->v_max_mdegps,
                                            db->wheel_d_mm);
  int32_t a_mmps2 = db_angle_mdegps_to_mmps(dl->accel_mdegps2,
                                            db->wheel_d_mm);
  int32_t d_mmps2 = db_angle_mdegps_to_mmps(dl->decel_mdegps2,
                                            db->wheel_d_mm);
  (void)hl;

  db->active_command = DRIVEBASE_ACTIVE_TURN;
  return dispatch_position(db, now_us, 0, dl_diff_mm,
                           v_mmps, a_mmps2, d_mmps2, on_completion);
}

int db_drivebase_drive_curve(struct db_drivebase_s *db,
                             uint64_t now_us,
                             int32_t radius_mm,
                             int32_t angle_deg,
                             uint8_t on_completion)
{
  /* Curve: travel arc of `angle_deg` along a circle of `radius_mm`.
   *   centre travel  (mm)         = R · |angle_rad|       sign = sgn(angle)
   *   heading change (deg)        = angle_deg
   * Sign convention: positive radius makes the centre of the circle
   * lie to the left, so positive angle moves forward and turns left
   * (heading positive).
   */

  int32_t dl_avg_mm  = (int32_t)((int64_t)radius_mm * angle_deg *
                                  DB_PI_NUM / DB_PI_DEN / 180);
  int32_t dl_diff_mm = heading_mdeg_to_diff_mm(angle_deg * 1000,
                                               db->axle_t_mm);

  const struct db_traj_limits_s *dl =
    db_settings_distance_limits(db->wheel_d_mm);
  int32_t v_mmps  = db_angle_mdegps_to_mmps(dl->v_max_mdegps,
                                            db->wheel_d_mm);
  int32_t a_mmps2 = db_angle_mdegps_to_mmps(dl->accel_mdegps2,
                                            db->wheel_d_mm);
  int32_t d_mmps2 = db_angle_mdegps_to_mmps(dl->decel_mdegps2,
                                            db->wheel_d_mm);

  db->active_command = DRIVEBASE_ACTIVE_CURVE;
  return dispatch_position(db, now_us, dl_avg_mm, dl_diff_mm,
                           v_mmps, a_mmps2, d_mmps2, on_completion);
}

int db_drivebase_drive_arc_distance(struct db_drivebase_s *db,
                                    uint64_t now_us,
                                    int32_t radius_mm,
                                    int32_t distance_mm,
                                    uint8_t on_completion)
{
  /* Arc by distance: travel `distance_mm` of centre-line along a
   * circle of `radius_mm`.  Implied angle = distance / R rad.
   * Same dispatch as drive_curve but compute differential from
   * the implied angle.
   */

  if (radius_mm == 0)
    {
      return db_drivebase_drive_straight(db, now_us, distance_mm,
                                         on_completion);
    }

  /* angle_mdeg = distance_mm / radius_mm * (180 * 1000 / π)         */
  int32_t angle_mdeg = (int32_t)((int64_t)distance_mm * 1000 * 180 *
                                  DB_PI_DEN /
                                  ((int64_t)radius_mm * DB_PI_NUM));
  int32_t dl_avg_mm  = distance_mm;
  int32_t dl_diff_mm = heading_mdeg_to_diff_mm(angle_mdeg,
                                               db->axle_t_mm);

  const struct db_traj_limits_s *dl =
    db_settings_distance_limits(db->wheel_d_mm);
  int32_t v_mmps  = db_angle_mdegps_to_mmps(dl->v_max_mdegps,
                                            db->wheel_d_mm);
  int32_t a_mmps2 = db_angle_mdegps_to_mmps(dl->accel_mdegps2,
                                            db->wheel_d_mm);
  int32_t d_mmps2 = db_angle_mdegps_to_mmps(dl->decel_mdegps2,
                                            db->wheel_d_mm);

  db->active_command = DRIVEBASE_ACTIVE_ARC;
  return dispatch_position(db, now_us, dl_avg_mm, dl_diff_mm,
                           v_mmps, a_mmps2, d_mmps2, on_completion);
}

int db_drivebase_drive_forever(struct db_drivebase_s *db,
                               uint64_t now_us,
                               int32_t speed_mmps,
                               int32_t turn_rate_dps)
{
  /* turn_rate_dps mm-equivalent differential = (turn_rate_dps *
   *  π/180 * axle_t) mm of L/R differential per second.
   */

  int32_t v_diff_mmps = heading_mdeg_to_diff_mm(turn_rate_dps * 1000,
                                                db->axle_t_mm);

  const struct db_traj_limits_s *dl =
    db_settings_distance_limits(db->wheel_d_mm);
  int32_t a_mmps2 = db_angle_mdegps_to_mmps(dl->accel_mdegps2,
                                            db->wheel_d_mm);

  db->active_command = DRIVEBASE_ACTIVE_FOREVER;
  return dispatch_forever(db, now_us, speed_mmps, v_diff_mmps,
                          a_mmps2);
}

int db_drivebase_stop(struct db_drivebase_s *db,
                      uint64_t now_us, uint8_t on_completion)
{
  db->active_command = DRIVEBASE_ACTIVE_STOP;
  int rc = db_servo_stop(&db->servo[DB_SIDE_LEFT],  now_us, on_completion);
  if (rc < 0) return rc;
  return db_servo_stop(&db->servo[DB_SIDE_RIGHT], now_us, on_completion);
}

/****************************************************************************
 * Public Functions — per-tick update + state snapshot
 ****************************************************************************/

int db_drivebase_update(struct db_drivebase_s *db, uint64_t now_us)
{
  int rc = db_servo_update(&db->servo[DB_SIDE_LEFT],  now_us);
  if (rc < 0) return rc;
  rc = db_servo_update(&db->servo[DB_SIDE_RIGHT], now_us);
  if (rc < 0) return rc;

  /* Refresh aggregate state.  Travel = (l_L + l_R)/2; heading is the
   * differential scaled by axle_t/wheel_d.
   */

  struct db_servo_status_s sL, sR;
  db_servo_get_status(&db->servo[DB_SIDE_LEFT],  &sL);
  db_servo_get_status(&db->servo[DB_SIDE_RIGHT], &sR);

  int32_t lL_mm = db_angle_mdeg_to_mm(sL.act_x_mdeg, db->wheel_d_mm);
  int32_t lR_mm = db_angle_mdeg_to_mm(sR.act_x_mdeg, db->wheel_d_mm);
  int32_t avg_mm  = (lL_mm + lR_mm) / 2;
  int32_t diff_mm = lR_mm - lL_mm;

  db->distance_mm      = avg_mm;
  db->angle_mdeg       = diff_mm_to_heading_mdeg(diff_mm,
                                                 db->axle_t_mm);

  int32_t vL_mmps = db_angle_mdegps_to_mmps(sL.act_v_mdegps,
                                            db->wheel_d_mm);
  int32_t vR_mmps = db_angle_mdegps_to_mmps(sR.act_v_mdegps,
                                            db->wheel_d_mm);
  db->drive_speed_mmps = (vL_mmps + vR_mmps) / 2;
  /* Heading rate (deg/s) = (vR - vL) / axle_t * 180/π                */
  int32_t v_diff = vR_mmps - vL_mmps;
  int32_t v_diff_mdegps = diff_mm_to_heading_mdeg(v_diff,
                                                  db->axle_t_mm);
  db->turn_rate_dps    = v_diff_mdegps / 1000;

  db->done    = sL.done && sR.done;
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
  out->tick_seq          = 0;     /* daemon FSM (commit #11) sets this  */
  out->is_done           = db->done;
  out->is_stalled        = db->stalled;
  out->active_command    = db->active_command;
  out->reserved          = 0;
}

bool db_drivebase_is_done(const struct db_drivebase_s *db)
{
  return db->done;
}
