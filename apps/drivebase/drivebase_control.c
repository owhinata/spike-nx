/****************************************************************************
 * apps/drivebase/drivebase_control.c
 *
 * PID + anti-windup + on_completion handling.  Pure integer math.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include "drivebase_control.h"

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static int32_t abs32(int32_t v)
{
  return v < 0 ? -v : v;
}

/* Decide whether the I term should accumulate this tick.  Returns true
 * when we should *skip* the integration (anti-windup conditions).
 */

static bool should_freeze_integrator(const struct db_pid_state_s *st,
                                     const struct db_servo_gains_s *g,
                                     int32_t pos_err_mdeg,
                                     bool sat_high, bool sat_low)
{
  if (st->paused)
    {
      return true;
    }
  if (abs32(pos_err_mdeg) <= g->deadband_mdeg)
    {
      return true;
    }
  if (sat_high && pos_err_mdeg > 0)
    {
      return true;
    }
  if (sat_low && pos_err_mdeg < 0)
    {
      return true;
    }
  return false;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_pid_init(struct db_pid_state_s *st)
{
  db_pid_reset(st);
}

void db_pid_reset(struct db_pid_state_s *st)
{
  st->i_acc                 = 0;
  st->prev_pos_err_mdeg     = 0;
  st->paused                = false;
  st->output_saturated_high = false;
  st->output_saturated_low  = false;
  st->done_streak_ms        = 0;
  st->done                  = false;
  st->smart_hold_streak_ms  = 0;
  st->smart_hold_active     = false;
  st->smart_hold_expired    = false;
}

void db_pid_pause(struct db_pid_state_s *st, bool paused)
{
  st->paused = paused;
}

void db_pid_update(struct db_pid_state_s *st,
                   const struct db_pid_input_s *in,
                   struct db_pid_output_s *out)
{
  const struct db_servo_gains_s        *g  = in->gains;
  const struct db_completion_settings_s *c  = in->completion;

  /* Compute errors.  Position error is clamped to int32 to keep the
   * P term well-behaved even after a long-running RESET (the daemon
   * uses int64 cumulative angles, but errors should be small).
   */

  int64_t pos_err_64  = in->ref_x_mdeg - in->act_x_mdeg;
  if (pos_err_64 >  INT32_MAX) pos_err_64 = INT32_MAX;
  if (pos_err_64 <  INT32_MIN) pos_err_64 = INT32_MIN;
  int32_t pos_err     = (int32_t)pos_err_64;
  int32_t speed_err   = in->ref_v_mdegps - in->act_v_mdegps;

  /* P + D on position, P on speed (cascaded structure). */

  int64_t p_term  = (int64_t)g->kp_pos * pos_err / 1000;
  int64_t d_term  = 0;
  if (in->dt_ms > 0)
    {
      int32_t dpe = pos_err - st->prev_pos_err_mdeg;
      d_term = (int64_t)g->kd_pos * dpe / (int64_t)in->dt_ms;
    }
  int64_t kpv     = (int64_t)g->kp_speed * speed_err / 1000;

  int64_t out_unsat = p_term + d_term + kpv + (st->i_acc / 4096);

  int32_t duty = clamp_i32((int32_t)out_unsat, g->out_min, g->out_max);

  bool sat_high = (out_unsat > g->out_max);
  bool sat_low  = (out_unsat < g->out_min);

  /* Anti-windup gated I update.  Integrate against the position error
   * (Q12 internal scale to avoid resolution loss with small errors).
   */

  if (!should_freeze_integrator(st, g, pos_err, sat_high, sat_low))
    {
      int64_t step = (int64_t)g->ki_pos * pos_err *
                     (int64_t)in->dt_ms / 1000;        /* mdeg·ms / s   */
      st->i_acc += step;

      /* Speed-based I (helps when at near-zero pos error but still    */
      /* needing torque to maintain speed against load).               */

      step = (int64_t)g->ki_speed * speed_err *
             (int64_t)in->dt_ms / 1000;
      st->i_acc += step;

      /* Cap accumulator at ±out_max scaled by the 4096 factor used at */
      /* the contribution stage, so it can never single-handedly       */
      /* saturate the output.                                          */

      int64_t i_clip = (int64_t)g->out_max * 4096;
      if (st->i_acc >  i_clip) st->i_acc =  i_clip;
      if (st->i_acc < -i_clip) st->i_acc = -i_clip;
    }

  st->prev_pos_err_mdeg     = pos_err;
  st->output_saturated_high = sat_high;
  st->output_saturated_low  = sat_low;

  /* Completion bookkeeping.  Only meaningful for finite trajectories;
   * infinite trajectories never set `trajectory_done=true`.
   */

  bool within_pos = abs32(pos_err)   <= c->pos_tolerance_mdeg;
  bool within_v   = abs32(speed_err) <= c->speed_tolerance_mdegps;
  if (in->trajectory_done && within_pos && within_v)
    {
      st->done_streak_ms += in->dt_ms;
      if (st->done_streak_ms >= c->done_window_ms)
        {
          st->done = true;
        }
    }
  else
    {
      st->done_streak_ms = 0;
      st->done           = false;
    }

  /* on_completion arbitration.  Default = drive duty from PID.
   * Done + COAST/BRAKE/COAST_SMART/BRAKE_SMART changes the actuation
   * the caller should perform; HOLD keeps PID active; CONTINUE keeps
   * PID active (caller does not stop the trajectory either).
   */

  uint8_t actuation = DRIVEBASE_ON_COMPLETION_HOLD;  /* drive duty by default */
  bool effective_done = st->done;

  if (effective_done)
    {
      switch (in->on_completion)
        {
          case DRIVEBASE_ON_COMPLETION_COAST:
            actuation = DRIVEBASE_ON_COMPLETION_COAST;
            db_pid_pause(st, true);
            duty      = 0;
            break;

          case DRIVEBASE_ON_COMPLETION_BRAKE:
            actuation = DRIVEBASE_ON_COMPLETION_BRAKE;
            db_pid_pause(st, true);
            duty      = 0;
            break;

          case DRIVEBASE_ON_COMPLETION_HOLD:
            /* keep driving duty; controller stays active.  */
            actuation = DRIVEBASE_ON_COMPLETION_HOLD;
            break;

          case DRIVEBASE_ON_COMPLETION_CONTINUE:
            /* trajectory caller chooses to extend the move; the
             * controller stays active and uses whatever the next
             * trajectory_done==false reference looks like.
             */
            actuation = DRIVEBASE_ON_COMPLETION_HOLD;
            break;

          case DRIVEBASE_ON_COMPLETION_COAST_SMART:
          case DRIVEBASE_ON_COMPLETION_BRAKE_SMART:
            /* SMART: keep PID active for smart_passive_hold_ms after
             * is_done first asserts, then degrade to coast/brake.
             */
            if (!st->smart_hold_active && !st->smart_hold_expired)
              {
                st->smart_hold_active    = true;
                st->smart_hold_streak_ms = 0;
              }
            if (st->smart_hold_active)
              {
                st->smart_hold_streak_ms += in->dt_ms;
                if (st->smart_hold_streak_ms >= c->smart_passive_hold_ms)
                  {
                    st->smart_hold_active  = false;
                    st->smart_hold_expired = true;
                  }
              }
            if (st->smart_hold_expired)
              {
                actuation = (in->on_completion ==
                             DRIVEBASE_ON_COMPLETION_BRAKE_SMART) ?
                            DRIVEBASE_ON_COMPLETION_BRAKE :
                            DRIVEBASE_ON_COMPLETION_COAST;
                db_pid_pause(st, true);
                duty = 0;
              }
            else
              {
                /* Passive hold: keep the controller alive driving duty */
                actuation = DRIVEBASE_ON_COMPLETION_HOLD;
              }
            break;

          default:
            actuation = DRIVEBASE_ON_COMPLETION_HOLD;
            break;
        }
    }

  out->duty       = duty;
  out->actuation  = actuation;
  out->done       = effective_done;
}
