/****************************************************************************
 * apps/drivebase/drivebase_aggregate.c
 *
 * Aggregate distance/heading PID controller (Issue #141 Phase 2 A).
 * See drivebase_aggregate.h for the architecture overview.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <arch/board/board_drivebase.h>  /* enum drivebase_on_completion_e */

#include "drivebase_aggregate.h"
#include "drivebase_settings.h"

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int64_t abs64(int64_t v)
{
  return v < 0 ? -v : v;
}

/* Refresh cached gain / completion pointers — db_settings_completion_axis
 * recomputes the derived tolerance on every call, so re-fetching keeps
 * the controller in sync with the latest kp_pos.
 */

static void aggregate_refresh_settings(struct db_aggregate_control_s *ctl)
{
  ctl->gains      = db_settings_pid_gains(ctl->axis);
  ctl->completion = db_settings_completion_axis(ctl->axis);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_aggregate_control_init(struct db_aggregate_control_s *ctl,
                               enum db_axis_e axis,
                               uint32_t tick_ms)
{
  memset(ctl, 0, sizeof(*ctl));
  ctl->axis           = axis;
  ctl->tick_ms        = tick_ms ? tick_ms : 2;
  ctl->on_completion  = DRIVEBASE_ON_COMPLETION_COAST;
  aggregate_refresh_settings(ctl);
  db_pid_init(&ctl->pid);
}

void db_aggregate_control_reset(struct db_aggregate_control_s *ctl)
{
  aggregate_refresh_settings(ctl);
  ctl->trajectory_active   = false;
  ctl->prev_endpoint_valid = false;
  ctl->prev_endpoint_mdeg  = 0;
  ctl->on_completion       = DRIVEBASE_ON_COMPLETION_COAST;
  db_pid_reset(&ctl->pid);
}

int db_aggregate_control_drive_position(struct db_aggregate_control_s *ctl,
                                        uint64_t now_us,
                                        int64_t origin_mdeg,
                                        int64_t delta_mdeg,
                                        int32_t v_max_mdegps,
                                        int32_t accel_mdegps2,
                                        int32_t decel_mdegps2,
                                        uint8_t on_completion)
{
  aggregate_refresh_settings(ctl);

  /* SMART continue-from-endpoint: if the previous finite trajectory's
   * endpoint is still within `smart_continue_window_mdeg` of the
   * requested origin, start the new move from the recorded endpoint
   * (pybricks "continue from endpoint" behaviour, #140 decoupled the
   * window from pos_tolerance so retuning tolerance does not widen
   * this window).
   */

  int64_t actual_origin = origin_mdeg;
  if (ctl->prev_endpoint_valid)
    {
      int64_t err = abs64(ctl->prev_endpoint_mdeg - origin_mdeg);
      if (err <= (int64_t)ctl->completion->smart_continue_window_mdeg)
        {
          actual_origin = ctl->prev_endpoint_mdeg;
        }
    }

  int64_t target = actual_origin + delta_mdeg;
  db_trajectory_init_position(&ctl->trajectory, now_us,
                              actual_origin, target,
                              v_max_mdegps,
                              accel_mdegps2, decel_mdegps2);

  ctl->trajectory_active   = true;
  ctl->prev_endpoint_valid = true;
  ctl->prev_endpoint_mdeg  = target;
  ctl->on_completion       = on_completion;
  db_pid_pause(&ctl->pid, false);
  db_pid_reset(&ctl->pid);
  return 0;
}

int db_aggregate_control_drive_forever(struct db_aggregate_control_s *ctl,
                                       uint64_t now_us,
                                       int64_t origin_mdeg,
                                       int32_t v_target_mdegps,
                                       int32_t accel_mdegps2)
{
  aggregate_refresh_settings(ctl);

  /* In-flight retarget — keep PID accumulators / trajectory phase so a
   * 100 Hz re-issue stream does not pin v=0 forever.
   */

  if (ctl->trajectory_active && ctl->trajectory.infinite)
    {
      db_trajectory_retarget_forever(&ctl->trajectory, now_us,
                                     v_target_mdegps, accel_mdegps2);
      ctl->prev_endpoint_valid = false;
      ctl->on_completion       = DRIVEBASE_ON_COMPLETION_COAST;
      return 0;
    }

  db_trajectory_init_forever(&ctl->trajectory, now_us, origin_mdeg,
                             v_target_mdegps, accel_mdegps2);
  ctl->trajectory_active   = true;
  ctl->prev_endpoint_valid = false;
  ctl->on_completion       = DRIVEBASE_ON_COMPLETION_COAST;
  db_pid_pause(&ctl->pid, false);
  db_pid_reset(&ctl->pid);
  return 0;
}

int db_aggregate_control_drive_hold(struct db_aggregate_control_s *ctl,
                                    uint64_t now_us,
                                    int64_t origin_mdeg)
{
  aggregate_refresh_settings(ctl);

  /* Zero-length trajectory at the current state position.  The PID
   * actively tracks origin_mdeg as the reference; small disturbances
   * pull the controller back to origin.  No SMART continuation —
   * HOLD is not a finite move with a meaningful endpoint.
   */

  db_trajectory_init_position(&ctl->trajectory, now_us,
                              origin_mdeg, origin_mdeg,
                              1, 1, 1);
  ctl->trajectory_active   = true;
  ctl->prev_endpoint_valid = false;
  ctl->on_completion       = DRIVEBASE_ON_COMPLETION_HOLD;
  db_pid_pause(&ctl->pid, false);
  db_pid_reset(&ctl->pid);
  return 0;
}

int db_aggregate_control_stop(struct db_aggregate_control_s *ctl,
                              uint64_t now_us,
                              int64_t  current_state_mdeg,
                              uint8_t  on_completion)
{
  ctl->trajectory_active   = false;
  ctl->prev_endpoint_valid = false;
  ctl->on_completion       = on_completion;

  switch (on_completion)
    {
      case DRIVEBASE_ON_COMPLETION_COAST:
      case DRIVEBASE_ON_COMPLETION_COAST_SMART:
        db_pid_stop_passive(&ctl->pid);
        ctl->pid.latched_passive_done = true;
        ctl->pid.latched_actuation    = DRIVEBASE_ON_COMPLETION_COAST;
        return 0;

      case DRIVEBASE_ON_COMPLETION_BRAKE:
      case DRIVEBASE_ON_COMPLETION_BRAKE_SMART:
        db_pid_stop_passive(&ctl->pid);
        ctl->pid.latched_passive_done = true;
        ctl->pid.latched_actuation    = DRIVEBASE_ON_COMPLETION_BRAKE;
        return 0;

      case DRIVEBASE_ON_COMPLETION_HOLD:
      default:
        /* HOLD-on-stop = active hold at the current state position.
         * Re-arm with a zero-length trajectory so the PID has a fixed
         * reference and small disturbances get pulled back.
         */

        return db_aggregate_control_drive_hold(ctl, now_us,
                                               current_state_mdeg);
    }
}

void db_aggregate_control_update(struct db_aggregate_control_s *ctl,
                                 uint64_t now_us,
                                 int64_t  state_x_mdeg,
                                 int32_t  state_v_mdegps,
                                 uint32_t dt_ms,
                                 struct db_aggregate_output_s *out)
{
  /* Inactive AND not latched = no command staged.  Safe-default
   * COAST output keeps motors free until someone arms a trajectory.
   */

  if (!ctl->trajectory_active && !ctl->pid.latched_passive_done)
    {
      out->duty      = 0;
      out->actuation = DRIVEBASE_ON_COMPLETION_COAST;
      out->done      = ctl->pid.done;
      return;
    }

  /* If we are inactive but latched, db_pid_update's early-return path
   * will surface the latched actuation/duty.  The trajectory reference
   * values it ignores can be left at "hold here" so the input struct
   * is well-formed.
   */

  struct db_trajectory_ref_s ref;
  if (ctl->trajectory_active)
    {
      db_trajectory_get_reference(&ctl->trajectory, now_us, &ref);
    }
  else
    {
      ref.x_mdeg    = state_x_mdeg;
      ref.v_mdegps  = 0;
      ref.a_mdegps2 = 0;
      ref.done      = true;
    }

  struct db_pid_input_s in =
    {
      .ref_x_mdeg      = ref.x_mdeg,
      .ref_v_mdegps    = ref.v_mdegps,
      .act_x_mdeg      = state_x_mdeg,
      .act_v_mdegps    = state_v_mdegps,
      .dt_ms           = dt_ms ? dt_ms : ctl->tick_ms,
      .gains           = ctl->gains,
      .completion      = ctl->completion,
      .trajectory_done = ref.done,
      .on_completion   = ctl->on_completion,
    };

  struct db_pid_output_s pout;
  db_pid_update(&ctl->pid, &in, &pout);
  out->duty      = pout.duty;
  out->actuation = pout.actuation;
  out->done      = pout.done;
}

bool db_aggregate_control_is_done(const struct db_aggregate_control_s *ctl)
{
  return ctl->pid.done;
}

bool db_aggregate_control_is_active(const struct db_aggregate_control_s *ctl)
{
  return ctl->trajectory_active;
}
