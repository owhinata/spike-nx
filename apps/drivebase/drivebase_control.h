/****************************************************************************
 * apps/drivebase/drivebase_control.h
 *
 * PID + anti-windup + on_completion handling for the drivebase daemon
 * (Issue #77 commit #5).  The same controller is reused on three sides:
 *
 *   - per-motor servo:  position error from trajectory(t) → reference,
 *                       observer → actual, output → motor duty
 *   - drivebase distance: aggregated L+R position → reference, output
 *                       → distance contribution to per-motor speed
 *   - drivebase heading:  L−R or gyro → reference, output → heading
 *                       contribution to per-motor speed
 *
 * The anti-windup rule mirrors pybricks `pbio/src/integrator.c`: stop
 * accumulating I when (a) the error is inside the deadband, (b) the
 * output is saturated and the error has the same sign as the output,
 * or (c) the controller is paused.  Completion judging follows the
 * pybricks pattern: hold |pos_err| < tol AND |speed_err| < tol for
 * `done_window_ms` continuous milliseconds.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_CONTROL_H
#define __APPS_DRIVEBASE_DRIVEBASE_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board_drivebase.h>  /* enum drivebase_on_completion_e */

#include "drivebase_settings.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_pid_state_s
{
  int64_t  i_acc;             /* integrator accumulator, scaled by      */
                              /* DB_IACC_SCALE (=256, i.e. Q8 of duty)   */
                              /* — divide by DB_IACC_SCALE before adding */
                              /* to the duty output (Issue #133).        */
  int32_t  prev_pos_err_mdeg; /* for D term                              */
  bool     paused;
  bool     output_saturated_high;
  bool     output_saturated_low;

  /* Completion bookkeeping (per-tick) */

  uint32_t done_streak_ms;
  bool     done;

  /* SMART completion timed transition */

  uint32_t smart_hold_streak_ms;
  bool     smart_hold_active;     /* true while still in passive hold    */
  bool     smart_hold_expired;    /* set true once hold time elapsed     */

  /* Latched passive completion (Issue #132).  Once a natural COAST/    */
  /* BRAKE completion fires, the controller stays off until the caller  */
  /* arms a new trajectory (db_pid_reset) or unpauses explicitly        */
  /* (db_pid_pause false).  Without this latch, a disturbance after     */
  /* completion shrinks the done streak, drops actuation back to HOLD   */
  /* (the default), and re-engages the PID at whatever duty the gains   */
  /* produce.                                                           */

  bool     latched_passive_done;
  uint8_t  latched_actuation;     /* enum drivebase_on_completion_e —   */
                                  /* COAST or BRAKE while latched       */
};

struct db_pid_input_s
{
  /* Reference */

  int64_t  ref_x_mdeg;
  int32_t  ref_v_mdegps;
  int32_t  ref_a_mdegps2;     /* trajectory accel ref; consumed by FF   */
                              /* (Issue #127 Phase 6 Step 6.1).         */

  /* Actual (from observer) */

  int64_t  act_x_mdeg;
  int32_t  act_v_mdegps;

  /* Tick interval (ms) */

  uint32_t dt_ms;

  /* Servo gains + completion thresholds */

  const struct db_servo_gains_s        *gains;
  const struct db_completion_settings_s *completion;

  /* Feed-forward duty pre-computed by the caller (Issue #127 Phase 6
   * Step 6.1).  Plan D2: summed inside db_pid_update before saturation
   * so the anti-windup gate sees the saturated/un-saturated state of
   * the *full* output and freezes I correctly.  Caller is responsible
   * for kV/kA math; per-motor kS (Step 6.2) is applied OUTSIDE this
   * struct after the L/R compose.  Defaults to 0 if the caller leaves
   * the field at its designated-initializer zero.
   */

  int32_t  duty_ff;

  /* Trajectory done flag (derived externally — control just consumes  */
  /* it for completion / smart logic).                                 */

  bool     trajectory_done;

  uint8_t  on_completion;     /* enum drivebase_on_completion_e          */
};

struct db_pid_output_s
{
  int32_t  duty;              /* signed PWM duty in .01 % units          */
  uint8_t  actuation;         /* enum drivebase_on_completion_e — what   */
                              /* the caller should do this tick: drive  */
                              /* duty / coast / brake / hold             */
  bool     done;              /* mirrors state->done after this update   */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void db_pid_init(struct db_pid_state_s *st);
void db_pid_reset(struct db_pid_state_s *st);
void db_pid_pause(struct db_pid_state_s *st, bool paused);

/* Passive stop: clear integrator/error, freeze I update, mark done.
 * Used when the controller transitions out of an active trajectory
 * (explicit STOP) and the caller wants get-state.is_done == true while
 * keeping PID inputs from re-driving the motor on the next tick.
 */

void db_pid_stop_passive(struct db_pid_state_s *st);

void db_pid_update(struct db_pid_state_s *st,
                   const struct db_pid_input_s *in,
                   struct db_pid_output_s *out);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_CONTROL_H */
