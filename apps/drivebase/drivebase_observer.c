/****************************************************************************
 * apps/drivebase/drivebase_observer.c
 *
 * IIR speed observer + stall detector.  Pure integer math.
 ****************************************************************************/

#include <nuttx/config.h>

#include "drivebase_observer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define US_PER_S       ((int64_t)1000000)

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int32_t lp_step(int32_t v_old, int32_t v_meas, uint32_t alpha_q15)
{
  /* v_new = (1 - α) * v_old + α * v_meas
   *       = v_old + α * (v_meas - v_old)
   * Computed in int64 to avoid intermediate overflow.
   */

  int64_t delta = (int64_t)v_meas - (int64_t)v_old;
  int64_t step  = (delta * (int64_t)alpha_q15) >> 15;
  return v_old + (int32_t)step;
}

static void update_stall(struct db_observer_s *o,
                         uint32_t dt_ms,
                         uint32_t applied_duty_abs)
{
  uint32_t v_abs = (uint32_t)(o->v_est_mdegps >= 0 ?
                              o->v_est_mdegps : -o->v_est_mdegps);

  if (v_abs < o->stall_low_speed_mdegps &&
      applied_duty_abs > o->stall_min_duty)
    {
      o->stall_streak_ms += dt_ms;
      if (o->stall_streak_ms >= o->stall_window_ms)
        {
          o->stalled = true;
        }
    }
  else
    {
      o->stall_streak_ms = 0;
      o->stalled         = false;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_observer_init(struct db_observer_s *o,
                      uint32_t low_speed_mdegps,
                      uint32_t min_duty,
                      uint32_t stall_window_ms,
                      uint32_t alpha_q15)
{
  o->x_mdeg                 = 0;
  o->t_us                   = 0;
  o->primed                 = false;
  o->v_est_mdegps           = 0;
  o->alpha_q15              = alpha_q15 ? alpha_q15 : 13107;  /* ≈ 0.4 */
  o->stall_low_speed_mdegps = low_speed_mdegps;
  o->stall_min_duty         = min_duty;
  o->stall_window_ms        = stall_window_ms;
  o->stall_streak_ms        = 0;
  o->stalled                = false;
}

void db_observer_reset(struct db_observer_s *o,
                       int64_t x_mdeg, uint64_t t_us)
{
  o->x_mdeg          = x_mdeg;
  o->t_us            = t_us;
  o->primed          = true;
  o->v_est_mdegps    = 0;
  o->stall_streak_ms = 0;
  o->stalled         = false;
}

void db_observer_update_sample(struct db_observer_s *o,
                               int64_t x_mdeg, uint64_t t_us,
                               uint32_t applied_duty_abs)
{
  if (!o->primed || t_us <= o->t_us)
    {
      /* First sample primes the state without producing an estimate. */
      o->x_mdeg = x_mdeg;
      o->t_us   = t_us;
      o->primed = true;
      return;
    }

  uint64_t dt_us = t_us - o->t_us;
  if (dt_us == 0)
    {
      return;
    }

  int64_t dx_mdeg = x_mdeg - o->x_mdeg;
  /* v_meas = dx / dt (mdeg / s).  Compute in int64 to keep precision. */
  int64_t v_meas64 = (dx_mdeg * US_PER_S) / (int64_t)dt_us;
  if (v_meas64 >  INT32_MAX) v_meas64 = INT32_MAX;
  if (v_meas64 <  INT32_MIN) v_meas64 = INT32_MIN;
  int32_t v_meas = (int32_t)v_meas64;

  o->v_est_mdegps = lp_step(o->v_est_mdegps, v_meas, o->alpha_q15);
  o->x_mdeg       = x_mdeg;

  uint32_t dt_ms = (uint32_t)((dt_us + 500) / 1000);
  o->t_us         = t_us;
  update_stall(o, dt_ms, applied_duty_abs);
}

void db_observer_idle_tick(struct db_observer_s *o, uint32_t dt_ms,
                           uint32_t applied_duty_abs)
{
  if (!o->primed)
    {
      return;
    }
  update_stall(o, dt_ms, applied_duty_abs);
}
