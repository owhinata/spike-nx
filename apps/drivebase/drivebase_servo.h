/****************************************************************************
 * apps/drivebase/drivebase_servo.h
 *
 * Per-motor closed-loop servo for the drivebase daemon (Issue #77
 * commit #6).  Wires drivebase_motor (encoder + actuation),
 * drivebase_observer (speed estimate + stall), drivebase_trajectory
 * (reference profile) and drivebase_control (PID + on_completion)
 * into a single per-tick update step.
 *
 * The drivebase aggregator (commit #7) instantiates two of these
 * (DB_SIDE_LEFT and DB_SIDE_RIGHT) and feeds their trajectories from
 * the distance + heading controllers.  Standalone use through the
 * `drivebase _servo …` CLI verb runs each side independently for
 * commissioning.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_SERVO_H
#define __APPS_DRIVEBASE_DRIVEBASE_SERVO_H

#include <stdbool.h>
#include <stdint.h>

#include "drivebase_internal.h"
#include "drivebase_observer.h"
#include "drivebase_control.h"
#include "drivebase_trajectory.h"

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

  struct db_observer_s      observer;
  struct db_pid_state_s     pid;
  struct db_trajectory_s    trajectory;

  /* Trajectory state.  `prev_endpoint_*` is the previous finite
   * trajectory's endpoint; SMART completion uses it to decide whether
   * to start the next relative move from the recorded endpoint
   * (continue-from-endpoint) or from the current measured position.
   */

  bool                      trajectory_active;
  bool                      prev_endpoint_valid;
  int64_t                   prev_endpoint_mdeg;

  /* Tick bookkeeping */

  int64_t                   x_actual_mdeg;     /* last seen encoder    */
  uint64_t                  t_last_us;
  uint8_t                   on_completion;
  int32_t                   last_applied_duty;
  uint8_t                   last_actuation;

  /* Cached gain pointers — same defaults for both sides. */

  const struct db_servo_gains_s        *gains;
  const struct db_completion_settings_s *completion;
};

struct db_servo_status_s
{
  int64_t  ref_x_mdeg;
  int32_t  ref_v_mdegps;
  int64_t  act_x_mdeg;
  int32_t  act_v_mdegps;
  int32_t  applied_duty;
  uint8_t  actuation;
  bool     done;
  bool     stalled;
  bool     trajectory_active;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void db_servo_init(struct db_servo_s *s, enum db_side_e side);

/* Reset internal state (observer + PID + trajectory) to "stopped at
 * the current encoder reading".  Call once after drivebase_motor_init
 * succeeds and before issuing any move.
 */

int  db_servo_reset(struct db_servo_s *s, uint64_t now_us);

/* Issue a finite relative move of `delta_mdeg` from the controller's
 * current reference frame.  on_completion ∈ enum
 * drivebase_on_completion_e.  Returns 0 or a negated errno.  If the
 * previous trajectory's endpoint is still nearby (`|prev_endpoint -
 * current_pos| < pos_tol × 2`) and the previous trajectory ran with a
 * SMART completion, the new move starts from prev_endpoint instead of
 * current_pos (pybricks "continue from endpoint" behaviour).
 */

int  db_servo_position_relative(struct db_servo_s *s,
                                uint64_t now_us,
                                int64_t delta_mdeg,
                                int32_t v_max_mdegps,
                                int32_t accel_mdegps2,
                                int32_t decel_mdegps2,
                                uint8_t on_completion);

/* Drive forever at the signed speed.  No completion. */

int  db_servo_forever(struct db_servo_s *s,
                      uint64_t now_us,
                      int32_t v_target_mdegps,
                      int32_t accel_mdegps2);

/* Stop trajectory immediately and apply the requested actuation. */

int  db_servo_stop(struct db_servo_s *s, uint64_t now_us,
                   uint8_t on_completion);

/* Per-tick update: drain encoder, run observer + PID, push duty/coast/
 * brake to the motor.  Returns 0 or a negated errno (motor I/O error).
 * Caller is expected to honour the 5 ms cadence externally; `now_us`
 * is the absolute monotonic timestamp at tick wake.
 */

int  db_servo_update(struct db_servo_s *s, uint64_t now_us);

void db_servo_get_status(const struct db_servo_s *s,
                         struct db_servo_status_s *out);

bool db_servo_is_done(const struct db_servo_s *s);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_SERVO_H */
