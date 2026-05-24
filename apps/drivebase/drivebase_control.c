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
 * Pre-processor Definitions
 ****************************************************************************/

/* Integrator accumulator scale (Issue #133).  The I-term internally
 * runs in (duty << 8) so very small ki * pos_err * dt steps don't lose
 * resolution to integer truncation.  The contribution to the output
 * divides by DB_IACC_SCALE, and the saturation clip multiplies by
 * DB_IACC_SCALE — keep these in lockstep through this single define.
 */

#define DB_IACC_SCALE  256

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/* Clamp an int64 to [lo, hi] (both int32) and return as int32.  Lets
 * the caller avoid the implementation-defined narrowing conversion
 * that would occur if a large out-of-range int64 were cast to int32
 * before being clamped (Issue #134).
 */

static int32_t clamp_i64_to_i32(int64_t v, int32_t lo, int32_t hi)
{
  if (v < (int64_t)lo) return lo;
  if (v > (int64_t)hi) return hi;
  return (int32_t)v;
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
  st->latched_passive_done  = false;
  st->latched_actuation     = 0;
}

void db_pid_pause(struct db_pid_state_s *st, bool paused)
{
  st->paused = paused;

  /* Explicit unpause restarts the controller; drop any stale passive
   * latch so the next update can drive duty again.  Pause-true is
   * also called from the COAST/BRAKE completion branch which sets the
   * latch immediately afterwards, so we only clear on pause-false.
   */

  if (!paused)
    {
      st->latched_passive_done = false;
      st->latched_actuation    = 0;
    }
}

void db_pid_stop_passive(struct db_pid_state_s *st)
{
  /* db_pid_reset() clears paused=false and done=false; reapply both
   * here so the order dependency stays inside this helper.
   */

  db_pid_reset(st);
  st->paused = true;
  st->done   = true;
}

void db_pid_update(struct db_pid_state_s *st,
                   const struct db_pid_input_s *in,
                   struct db_pid_output_s *out)
{
  const struct db_servo_gains_s        *g  = in->gains;
  const struct db_completion_settings_s *c  = in->completion;

  /* Latched passive completion (Issue #132).  Once natural COAST/BRAKE
   * completion has fired, stay off — return the latched actuation with
   * zero duty regardless of pos_err/speed_err.  Cleared by
   * db_pid_reset() or an explicit db_pid_pause(false).
   */

  if (st->latched_passive_done)
    {
      out->duty      = 0;
      out->actuation = st->latched_actuation;
      out->done      = true;
      return;
    }

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

  /* Feed-forward duty (Issue #127 Phase 6 Step 6.1 — Plan D2).  Sum
   * inside the PID body BEFORE saturation so anti-windup's sat_high /
   * sat_low flags reflect the full output: otherwise FF that already
   * pins the rail would let I keep accumulating (false negative on the
   * "saturated in same direction as error" gate).  pybricks `servo.c:
   * 103-112` follows the same pattern (torque_ff summed before clamp).
   */

  int64_t out_unsat = p_term + d_term + kpv + (st->i_acc / DB_IACC_SCALE)
                      + (int64_t)in->duty_ff;

  /* Clamp in int64 first then cast — casting out_unsat to int32 before
   * clamping is implementation-defined when it overflows, and a large
   * positive out_unsat could wrap into a negative int32 that then
   * clamp_i32 would pin at g->out_min, producing reverse full-duty.
   * Issue #134.
   */

  int32_t duty = clamp_i64_to_i32(out_unsat, g->out_min, g->out_max);

  bool sat_high = (out_unsat > g->out_max);
  bool sat_low  = (out_unsat < g->out_min);

  /* Anti-windup gated I update.  i_acc carries DB_IACC_SCALE × duty
   * so very small ki·err·dt steps survive integer truncation.
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

      /* Cap accumulator at ±out_max × DB_IACC_SCALE so the post-divide
       * contribution can never single-handedly saturate the output.
       */

      int64_t i_clip = (int64_t)g->out_max * DB_IACC_SCALE;
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
            st->latched_passive_done = true;
            st->latched_actuation    = DRIVEBASE_ON_COMPLETION_COAST;
            break;

          case DRIVEBASE_ON_COMPLETION_BRAKE:
            actuation = DRIVEBASE_ON_COMPLETION_BRAKE;
            db_pid_pause(st, true);
            duty      = 0;
            st->latched_passive_done = true;
            st->latched_actuation    = DRIVEBASE_ON_COMPLETION_BRAKE;
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
                st->latched_passive_done = true;
                st->latched_actuation    = actuation;
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
