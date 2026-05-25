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
#include "drivebase_settings.h"  /* db_ff_motor_friction_s + db_ff_state_s */

#ifdef __cplusplus
extern "C"
{
#endif

/* Forward decl — Phase 3b heading PID consumes IMU via a daemon-owned
 * instance attached at startup, but we keep the full drivebase_imu.h
 * out of this header so build deps stay minimal.
 */

struct db_imu_s;

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

  /* Phase 3b (#148): IMU-locked heading injection.
   *
   * `imu` is attached by the daemon after db_imu_open() succeeds.  All
   * Phase 3b state below is owned by the RT-tick thread (chardev
   * handler, set_use_gyro, set_origin, reset, db_drivebase_update all
   * dispatch from the same RT callback), with two exceptions:
   *
   *   - use_gyro_requested / use_gyro_latched / last_set_gyro_rc are
   *     read by the daemon idle thread (drivebase_daemon.c publish_status,
   *     ~20 Hz) and written by the RT thread.  Single-byte accesses on
   *     Cortex-M4 are atomic for aligned types; `volatile` documents the
   *     cross-thread access and prevents the compiler from caching the
   *     value across the idle-loop sleep.
   *
   *   - gyro_origin_{mdeg,valid} are touched only by the RT thread.
   */

  struct db_imu_s   *imu;
  volatile uint8_t   use_gyro_requested;
                                /* enum drivebase_use_gyro_e — user-       */
                                /* requested mode (cfg or ioctl)           */
  volatile uint8_t   use_gyro_latched;
                                /* enum drivebase_use_gyro_e — mode in     */
                                /* effect for the in-flight motion; only   */
                                /* set at command-start when capturable    */
  volatile int8_t    last_set_gyro_rc;
                                /* last db_drivebase_set_use_gyro return   */
                                /* code (0 / -EBUSY / -EINVAL)             */
  int64_t            gyro_origin_mdeg;
                                /* robot-heading mdeg baseline; subtract   */
                                /* from raw IMU heading to obtain the      */
                                /* user-visible / PID-state value          */
  bool               gyro_origin_valid;

  /* Phase 6 Step 6.2 (#152): per-motor friction FF.  `ff_motor` points
   * at the live db_settings instance, fetched once at db_drivebase_init
   * (settings is frozen before the RT thread launches).  Per-side
   * `ff_state_left/right` holds the friction sign state: `sign_v_held`
   * tracks the hysteresised sign of v_ref so small oscillation around 0
   * does not chatter the kS contribution (Plan D1 + D4 + D6), and
   * `breakaway_consumed` / `breakaway_sign` drive the Phase 7 (#158)
   * per-move terminal breakaway one-shot (see drivebase_drivebase.c::
   * apply_breakaway_floor).  All RT-thread-owned, no atomics needed.
   */

  const struct db_ff_motor_friction_s *ff_motor;
  struct db_ff_state_s                 ff_state_left;
  struct db_ff_state_s                 ff_state_right;

  /* Phase 6 Step 6.3 (#152): battery sag correction settings.  Same
   * lifetime contract as ff_motor — fetched once at db_drivebase_init
   * after the settings module is frozen.  The live vbat reading comes
   * from db_battery_get_mv() (drivebase_battery.h), updated by the
   * daemon's 200 ms poll, not stored here.
   */

  const struct db_battery_settings_s  *battery;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Lifecycle */

int  db_drivebase_init(struct db_drivebase_s *db,
                       uint32_t wheel_d_um, uint32_t axle_t_um,
                       uint32_t tick_ms);

int  db_drivebase_reset(struct db_drivebase_s *db, uint64_t now_us);

/* Reset the daemon's distance/heading origin without touching motors.
 * `now_us` is needed by Phase 3b (#148) to re-evaluate the IMU origin
 * via gyro_origin_capturable() — encoder-only callers can ignore the
 * meaning and just thread the RT tick's clock through.
 */

void db_drivebase_set_origin(struct db_drivebase_s *db,
                             int32_t distance_mm, int32_t angle_mdeg,
                             uint64_t now_us);

/* Phase 3b (#148) — attach the daemon's IMU instance so the heading
 * PID can pull world-vertical mdeg into its state input.  May be called
 * once at daemon start, before the RT thread; passing NULL detaches.
 * Not thread-safe — caller must guarantee no in-flight RT tick.
 */

void db_drivebase_attach_imu(struct db_drivebase_s *db,
                             struct db_imu_s *imu);

/* Phase 3b (#148) — change the requested gyro mode.  Returns 0 on
 * success, -EBUSY if a motion is in flight, -EINVAL for unsupported
 * modes.  Issue #157: only NONE and 3D are accepted; 1D was removed
 * (its fused-quaternion forward-projection heading is what 3D selects).
 * Updates the db's latched `last_set_gyro_rc` regardless of the path
 * taken so a status snapshot can observe the last attempt.
 * Single-thread (RT tick) only.
 */

int  db_drivebase_set_use_gyro(struct db_drivebase_s *db, uint8_t mode,
                               uint64_t now_us);

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
