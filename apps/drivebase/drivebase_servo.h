/****************************************************************************
 * apps/drivebase/drivebase_servo.h
 *
 * Per-motor encoder I/O + observer + duty application for the drivebase
 * daemon (Issue #77 commit #6; Phase 2 #141 slim-down).  Phase 2 moved
 * the trajectory, PID, and completion latch into the aggregate
 * controller (apps/drivebase/drivebase_aggregate.c) so this struct only
 * holds per-motor concerns: encoder drain, slope observer, and the
 * "last duty / last actuation" for telemetry + ad-hoc apply.
 *
 * Pre-#141 per-motor PID is gone; closed-loop control now happens only
 * at the drivebase aggregate level.  The `drivebase _servo` CLI was
 * deleted alongside this change since it relied on per-motor PID.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_SERVO_H
#define __APPS_DRIVEBASE_DRIVEBASE_SERVO_H

#include <stdbool.h>
#include <stdint.h>

#include "drivebase_internal.h"
#include "drivebase_observer.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_servo_s
{
  enum db_side_e            side;

  /* Nominal control-loop period in ms (= drivebase RT tick).  Used by
   * the observer's dt fallback when measured Δt is unavailable.
   */

  uint32_t                  tick_ms;

  struct db_observer_s      observer;

  /* Cached state from the last db_servo_update() — handed to the
   * aggregate controller each tick to compute state_distance /
   * state_heading.
   */

  int64_t                   x_actual_mdeg;
  int32_t                   v_estimate_mdegps;
  uint64_t                  t_last_us;

  /* Last actuation applied via db_servo_apply().  Recorded for
   * telemetry (db_servo_get_status) and stall detection bookkeeping.
   */

  int32_t                   last_applied_duty;
  uint8_t                   last_actuation;
};

struct db_servo_status_s
{
  int64_t  act_x_mdeg;
  int32_t  act_v_mdegps;
  int32_t  applied_duty;
  uint8_t  actuation;
  bool     stalled;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void db_servo_init(struct db_servo_s *s, enum db_side_e side,
                   uint32_t tick_ms);

/* Drain any pending encoder samples + zero the observer state so a
 * subsequent db_servo_update() returns the new absolute position as
 * the start point.  Must be called once after drivebase_motor_init
 * succeeds and before any aggregate move is issued.
 */

int  db_servo_reset(struct db_servo_s *s, uint64_t now_us);

/* Per-tick observer update.  Drains the encoder, updates the slope
 * observer, refreshes `x_actual_mdeg` and `v_estimate_mdegps`.  Does
 * NOT touch motor I/O (the aggregate controller's compose result is
 * applied separately via db_servo_apply).
 */

int  db_servo_update(struct db_servo_s *s, uint64_t now_us);

/* Apply a duty (in .01 % units, signed) or a passive actuation to the
 * motor.  `actuation` ∈ enum drivebase_on_completion_e:
 *   DRIVE_DUTY / HOLD:  set PWM to `duty` (clamped to ±10000)
 *   COAST / COAST_SMART: motor coast, `duty` ignored
 *   BRAKE / BRAKE_SMART: motor brake, `duty` ignored
 *   CONTINUE:            treated like DRIVE_DUTY
 * Returns 0 or a negated errno (motor I/O error).  Records the applied
 * duty + actuation in the struct for telemetry.
 */

int  db_servo_apply(struct db_servo_s *s, int32_t duty, uint8_t actuation);

void db_servo_get_status(const struct db_servo_s *s,
                         struct db_servo_status_s *out);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_SERVO_H */
