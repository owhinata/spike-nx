/****************************************************************************
 * apps/drivebase/drivebase_drivebase.h
 *
 * Two-motor drivebase aggregator (Issue #77 commit #7; #141 Phase 2 A
 * refactor to aggregate distance/heading PID).  Wraps two slim
 * `db_servo_s` instances (DB_SIDE_LEFT and DB_SIDE_RIGHT) and two
 * aggregate controllers (control_distance + control_heading) and
 * exposes the user-level drive primitives (drive_straight, drive_curve,
 * drive_arc_*, drive_forever, turn, stop) the public chardev ABI
 * serves.
 *
 * Geometry (assuming a tank-style drivebase: parallel wheels, no
 * castor, no steering linkage):
 *
 *   Per wheel travel (mm):   l_X    = θ_X · π · wheel_d / 360
 *   Robot center distance:   D      = (l_L + l_R) / 2
 *   Robot heading (radians): H_rad  = (l_R - l_L) / axle_t
 *
 *   spike-nx public convention (matches pybricks public docs):
 *     positive heading angle = counter-clockwise viewed from above
 *     (right wheel ahead, left wheel back).
 *
 *   Internal state convention (Phase 2):
 *     state_distance = (sL.x + sR.x) / 2 in motor-mdeg
 *     state_heading  = (sR.x - sL.x) / 2 in motor-mdeg  (positive =
 *                                                       R ahead = CCW)
 *
 *   Per-side compose (Phase 2):
 *     left_duty  = clamp(duty_distance - duty_heading, ±10000)
 *     right_duty = clamp(duty_distance + duty_heading, ±10000)
 *
 *   (Positive heading PID output → right wheel faster than left →
 *   robot yaws CCW, matching the publicly exposed `angle_mdeg`
 *   semantics with no sign flip at the API boundary.  This is the
 *   mirror of pybricks's internal `(left - right) / 2` + L=D+H/R=D-H
 *   convention but is functionally equivalent and easier to reason
 *   about in spike-nx context.)
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_DRIVEBASE_H
#define __APPS_DRIVEBASE_DRIVEBASE_DRIVEBASE_H

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board_drivebase.h>

#include "drivebase_internal.h"
#include "drivebase_servo.h"
#include "drivebase_aggregate.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_drivebase_s
{
  /* Geometry — micrometers (0.001 mm) so user can specify sub-mm
   * wheel diameters; angle math in drivebase_angle.h is two-step int64
   * to keep the extra precision without overflow.
   */

  uint32_t wheel_d_um;
  uint32_t axle_t_um;

  /* RT control-loop period in ms (= db_rt_s.tick_us / 1000).  The
   * aggregate PIDs inherit this for their dt_ms fallback so gains stay
   * tick-proportional.  Issue #120.
   */

  uint32_t tick_ms;

  /* Per-side servos — encoder I/O + observer + duty apply only.  The
   * trajectory + PID live in the aggregate controllers below (#141).
   */

  struct db_servo_s servo[DB_SIDE_NUM];

  /* Aggregate controllers — one per axis (distance, heading).  Each
   * owns a trajectory + PID + completion latch (#141 Phase 2 A).
   */

  struct db_aggregate_control_s control[DB_AXIS_NUM];

  /* Last tick time, used for dt_ms fallback in aggregate update. */

  uint64_t last_tick_us;

  /* Origin offset (raw encoder-derived avg / heading at the last
   * reset).  db_drivebase_update() subtracts these from the raw
   * encoder readings so the daemon publishes distance/heading
   * relative to the latest reset point, not the absolute LUMP-mode-2
   * position the motor returned at power-on.  See Issue #113.
   */

  int32_t  distance_origin_mm;
  int32_t  angle_origin_mdeg;

  /* Cached aggregate state (rebuilt from per-servo + aggregate
   * controller status each tick).
   */

  int32_t  distance_mm;
  int32_t  drive_speed_mmps;
  int32_t  angle_mdeg;        /* heading × 1000                          */
  int32_t  turn_rate_dps;
  uint8_t  active_command;    /* enum drivebase_active_command_e         */
  bool     done;
  bool     stalled;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Lifecycle */

int  db_drivebase_init(struct db_drivebase_s *db,
                       uint32_t wheel_d_um, uint32_t axle_t_um,
                       uint32_t tick_ms);

int  db_drivebase_reset(struct db_drivebase_s *db, uint64_t now_us);

/* Reset the daemon's distance/heading origin without touching motors. */

void db_drivebase_set_origin(struct db_drivebase_s *db,
                             int32_t distance_mm, int32_t angle_mdeg);

/* User-level drive commands.  speed_mmps / turn_rate_dps == 0 falls
 * back to the db_settings_distance_limits / heading_limits defaults
 * (Issue #137).  Pass non-zero to override per-call.
 */

int  db_drivebase_drive_straight(struct db_drivebase_s *db,
                                 uint64_t now_us,
                                 int32_t distance_mm,
                                 int32_t speed_mmps,
                                 uint8_t on_completion);

int  db_drivebase_turn(struct db_drivebase_s *db,
                       uint64_t now_us,
                       int32_t angle_deg,
                       int32_t turn_rate_dps,
                       uint8_t on_completion);

int  db_drivebase_drive_curve(struct db_drivebase_s *db,
                              uint64_t now_us,
                              int32_t radius_mm,
                              int32_t angle_deg,
                              uint8_t on_completion);

int  db_drivebase_drive_arc_distance(struct db_drivebase_s *db,
                                     uint64_t now_us,
                                     int32_t radius_mm,
                                     int32_t distance_mm,
                                     uint8_t on_completion);

int  db_drivebase_drive_forever(struct db_drivebase_s *db,
                                uint64_t now_us,
                                int32_t speed_mmps,
                                int32_t turn_rate_dps);

int  db_drivebase_stop(struct db_drivebase_s *db,
                       uint64_t now_us,
                       uint8_t on_completion);

/* Per-tick update.  Drains both encoders, computes state_distance/
 * state_heading, runs the two aggregate PIDs, composes per-side duty,
 * applies actuation, refreshes cached aggregate state.
 */

int  db_drivebase_update(struct db_drivebase_s *db, uint64_t now_us);

/* Snapshot accessors — match the wire format of struct
 * drivebase_state_s so the chardev handler can memcpy directly.
 */

void db_drivebase_get_state(const struct db_drivebase_s *db,
                            struct drivebase_state_s *out);

bool db_drivebase_is_done(const struct db_drivebase_s *db);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_DRIVEBASE_H */
