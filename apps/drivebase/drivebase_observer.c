/****************************************************************************
 * apps/drivebase/drivebase_observer.c
 *
 * Sliding-window slope velocity estimator + stall detector.
 *
 * Each call to update_sample() pushes one (t, x) pair into a ring of
 * DB_OBSERVER_RING_DEPTH slots (~64 ms at 1 kHz publish rate, plenty
 * of head room for our 5 ms tick).  The velocity estimate is the slope
 * between the newest entry and the oldest still inside `window_us`.
 * That denominator is real time, not sample count, so the estimator
 * keeps working even if encoder updates arrive irregularly (the
 * upper-half sensor framework can drop intermediate samples under
 * load — we always reach back at least `window_us`).
 *
 * Stall detection is unchanged from the IIR-LP first cut: |v_est| <
 * threshold AND |duty| > threshold continuously for window_ms ⇒ stalled.
 ****************************************************************************/

#include <nuttx/config.h>

#include "drivebase_observer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define US_PER_S                ((int64_t)1000000)
#define DB_OBSERVER_DEFAULT_MS  30

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int32_t v_slope_mdegps(const struct db_observer_s *o)
{
  if (o->count < 2)
    {
      return 0;
    }

  uint8_t newest = (uint8_t)((o->head + DB_OBSERVER_RING_DEPTH - 1u) %
                              DB_OBSERVER_RING_DEPTH);
  uint64_t t_new = o->ring[newest].t_us;
  uint64_t deadline = (t_new > o->window_us) ? (t_new - o->window_us) : 0;

  /* Walk backwards from `newest` to find the oldest entry whose
   * timestamp is still ≥ deadline.  count entries are valid; we
   * include up to all of them.
   */

  uint8_t pick = newest;
  for (uint8_t i = 1; i < o->count; i++)
    {
      uint8_t idx = (uint8_t)((newest + DB_OBSERVER_RING_DEPTH - i) %
                              DB_OBSERVER_RING_DEPTH);
      if (o->ring[idx].t_us < deadline)
        {
          break;
        }
      pick = idx;
    }

  if (pick == newest)
    {
      return 0;            /* only the newest is in window */
    }

  int64_t dt = (int64_t)(t_new - o->ring[pick].t_us);
  if (dt <= 0)
    {
      return 0;
    }
  int64_t dx = o->ring[newest].x_mdeg - o->ring[pick].x_mdeg;
  int64_t v  = (dx * US_PER_S) / dt;
  if (v >  INT32_MAX) v = INT32_MAX;
  if (v <  INT32_MIN) v = INT32_MIN;
  return (int32_t)v;
}

static int32_t abs_i32(int32_t v) { return v < 0 ? -v : v; }

static void update_stall(struct db_observer_s *o, uint32_t dt_ms,
                         uint32_t applied_duty_abs)
{
  uint32_t v_abs = (uint32_t)abs_i32(o->v_est_mdegps);
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
                      uint32_t window_ms)
{
  o->head                   = 0;
  o->count                  = 0;
  o->v_est_mdegps           = 0;
  o->window_us              =
    (uint64_t)(window_ms ? window_ms : DB_OBSERVER_DEFAULT_MS) * 1000u;
  o->primed                 = false;
  o->stall_low_speed_mdegps = low_speed_mdegps;
  o->stall_min_duty         = min_duty;
  o->stall_window_ms        = stall_window_ms;
  o->stall_streak_ms        = 0;
  o->stalled                = false;
}

void db_observer_reset(struct db_observer_s *o,
                       int64_t x_mdeg, uint64_t t_us)
{
  o->head                   = 0;
  o->count                  = 0;
  o->v_est_mdegps           = 0;
  o->primed                 = true;
  o->stall_streak_ms        = 0;
  o->stalled                = false;

  o->ring[0].t_us           = t_us;
  o->ring[0].x_mdeg         = x_mdeg;
  o->head                   = 1;
  o->count                  = 1;
}

void db_observer_update_sample(struct db_observer_s *o,
                               int64_t x_mdeg, uint64_t t_us,
                               uint32_t applied_duty_abs)
{
  if (!o->primed)
    {
      o->ring[0].t_us   = t_us;
      o->ring[0].x_mdeg = x_mdeg;
      o->head           = 1;
      o->count          = 1;
      o->primed         = true;
      return;
    }

  /* Reject backward-time samples (only happens at first sample after
   * a SELECT mode change when the device firmware re-starts publishing).
   */

  uint8_t prev = (uint8_t)((o->head + DB_OBSERVER_RING_DEPTH - 1u) %
                            DB_OBSERVER_RING_DEPTH);
  if (t_us <= o->ring[prev].t_us)
    {
      return;
    }

  o->ring[o->head].t_us   = t_us;
  o->ring[o->head].x_mdeg = x_mdeg;
  o->head                 = (uint8_t)((o->head + 1u) %
                                       DB_OBSERVER_RING_DEPTH);
  if (o->count < DB_OBSERVER_RING_DEPTH)
    {
      o->count++;
    }

  o->v_est_mdegps = v_slope_mdegps(o);

  uint32_t dt_ms = (uint32_t)((t_us - o->ring[prev].t_us + 500) / 1000);
  update_stall(o, dt_ms, applied_duty_abs);
}

void db_observer_idle_tick(struct db_observer_s *o, uint32_t dt_ms,
                           uint32_t applied_duty_abs)
{
  if (!o->primed)
    {
      return;
    }

  /* Recompute the slope: as wall-clock advances, older entries fall
   * outside the window.  In the limit (motor stopped, no new samples
   * for `window_us`) the slope estimate decays to 0 because every
   * pair has dx ≈ 0.
   */

  o->v_est_mdegps = v_slope_mdegps(o);
  update_stall(o, dt_ms, applied_duty_abs);
}
