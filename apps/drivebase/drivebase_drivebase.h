/****************************************************************************
 * apps/drivebase/drivebase_drivebase.h
 *
 * Two-motor drivebase aggregator (Issue #77 commit #7).  Wraps two
 * `db_servo_s` instances (DB_SIDE_LEFT and DB_SIDE_RIGHT) and exposes
 * the user-level drive primitives (drive_straight, drive_curve,
 * drive_arc_*, drive_forever, turn, stop) the public chardev ABI
 * eventually serves.
 *
 * Geometry (assuming a tank-style drivebase: parallel wheels, no
 * castor, no steering linkage):
 *
 *   Per wheel travel (mm):   l_X    = θ_X · π · wheel_d / 360
 *   Robot center distance:   D      = (l_L + l_R) / 2
 *   Robot heading (radians): H_rad  = (l_R - l_L) / axle_t
 *
 * Inverse — to advance the robot by ΔD mm and ΔH deg of heading
 * (positive = counter-clockwise viewed from above):
 *
 *   Δl_avg  = ΔD                              (mm of avg wheel travel)
 *   Δl_diff = ΔH · π/180 · axle_t             (mm of L↔R differential)
 *   l_L = l_avg - l_diff/2 ;  l_R = l_avg + l_diff/2
 *
 * For the present commit each drive command turns into per-side
 * trajectories that the existing `db_servo_s` PIDs track
 * independently.  L/R coupling correction (a separate distance and
 * heading PID that overrides the per-motor tracking error) is
 * deferred until commit #7 retune work — bench measurements first
 * decide whether SPIKE Medium Motor variance is large enough to need
 * it.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_DRIVEBASE_H
#define __APPS_DRIVEBASE_DRIVEBASE_DRIVEBASE_H

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board_drivebase.h>

#include "drivebase_internal.h"
#include "drivebase_servo.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_drivebase_s
{
  /* Geometry */

  uint32_t wheel_d_mm;
  uint32_t axle_t_mm;

  /* Per-side servos — owned outright by the drivebase */

  struct db_servo_s servo[DB_SIDE_NUM];

  /* Cached aggregate state (rebuilt from per-servo status on demand) */

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
                       uint32_t wheel_d_mm, uint32_t axle_t_mm);

int  db_drivebase_reset(struct db_drivebase_s *db, uint64_t now_us);

/* Reset the daemon's distance/heading origin without touching motors. */

void db_drivebase_set_origin(struct db_drivebase_s *db,
                             int32_t distance_mm, int32_t angle_mdeg);

/* User-level drive commands.  Speed defaults come from
 * db_settings_distance_limits / heading_limits.
 */

int  db_drivebase_drive_straight(struct db_drivebase_s *db,
                                 uint64_t now_us,
                                 int32_t distance_mm,
                                 uint8_t on_completion);

int  db_drivebase_turn(struct db_drivebase_s *db,
                       uint64_t now_us,
                       int32_t angle_deg,
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

/* Per-tick update.  Calls each side's db_servo_update() and refreshes
 * cached aggregate state.
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
