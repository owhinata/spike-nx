/****************************************************************************
 * apps/drivebase/drivebase_observer.c
 *
 * Adaptive-window slope velocity estimator + stall detector +
 * zero-speed hysteresis (Issue #136).
 *
 * Each call to update_sample() pushes one (t, x) pair into a ring of
 * DB_OBSERVER_RING_DEPTH slots (~128 ms at 1 kHz publish rate).  The
 * velocity estimate walks back from the newest sample until both
 * (a) the time span has reached `min_window_us` and (b) the
 * accumulated |Δx| has reached MIN_LSB_COUNT × LSB.  At high speeds
 * this leaves the window at the minimum (low lag); at low speeds it
 * auto-extends so dx stays above the 1-LSB quantization noise floor.
 *
 * Zero-speed hysteresis snaps |v_raw| < V_HYST_LOW to zero and
 * requires |v_raw| > V_HYST_HIGH before releasing.  Suppresses the
 * ±17 dps sign-flip chatter around rest.
 *
 * Stall detection: |v_est| < threshold AND |duty| > threshold
 * continuously for stall_window_ms ⇒ stalled.  Uses the post-
 * hysteresis estimate so a snapped-to-zero motor under duty is
 * correctly flagged as stalled.
 ****************************************************************************/

#include <nuttx/config.h>

#include "drivebase_observer.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define US_PER_S                ((int64_t)1000000)
#define DB_OBSERVER_DEFAULT_MS  30

/* Adaptive-window slope criteria (Issue #136).  LSB is the encoder
 * quantum (1 deg = 1000 mdeg for LUMP MODE 2 POS).  Walk back from
 * `newest` until accumulated |Δx| ≥ MIN_LSB_COUNT × LSB AND the time
 * span has reached `min_window_us` — whichever requires more walking.
 * At high speeds this leaves the window at the minimum (low latency);
 * at low speeds it auto-extends to keep the dx large enough for
 * meaningful SNR.
 */

#define DB_OBSERVER_LSB_MDEG          1000
#define DB_OBSERVER_MIN_LSB_COUNT     3

/* Zero-speed hysteresis (Issue #136).  Snap v_est to 0 when |v_raw| <
 * V_HYST_LOW.  Stay snapped until |v_raw| > V_HYST_HIGH.  Suppresses
 * the ±17 dps slope chatter that 1-LSB quantization produces around
 * zero motion.
 */

#define DB_OBSERVER_V_HYST_LOW_MDEGPS  30000
#define DB_OBSERVER_V_HYST_HIGH_MDEGPS 60000

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int64_t abs_i64(int64_t v) { return v < 0 ? -v : v; }

/* Compute the raw adaptive-window slope (no hysteresis applied). */

static int32_t v_slope_raw_mdegps(const struct db_observer_s *o)
{
  if (o->count < 2)
    {
      return 0;
    }

  uint8_t newest = (uint8_t)((o->head + DB_OBSERVER_RING_DEPTH - 1u) %
                              DB_OBSERVER_RING_DEPTH);
  uint64_t t_new      = o->ring[newest].t_us;
  int64_t  x_new      = o->ring[newest].x_mdeg;
  const int64_t lsb_thr =
    (int64_t)DB_OBSERVER_MIN_LSB_COUNT * DB_OBSERVER_LSB_MDEG;

  /* Walk back from newest.  Stop once both criteria are met:
   *   (a) time span ≥ min_window_us
   *   (b) |Δx| ≥ MIN_LSB_COUNT × LSB
   * If we exhaust the ring before either criterion is met, fall back
   * to the oldest available entry (best effort).
   */

  uint8_t pick = newest;
  for (uint8_t i = 1; i < o->count; i++)
    {
      uint8_t idx = (uint8_t)((newest + DB_OBSERVER_RING_DEPTH - i) %
                              DB_OBSERVER_RING_DEPTH);
      pick = idx;
      uint64_t elapsed_us = t_new - o->ring[idx].t_us;
      int64_t  dx_abs     = abs_i64(x_new - o->ring[idx].x_mdeg);
      if (elapsed_us >= o->min_window_us && dx_abs >= lsb_thr)
        {
          break;
        }
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
  int64_t dx = x_new - o->ring[pick].x_mdeg;
  int64_t v  = (dx * US_PER_S) / dt;
  if (v >  INT32_MAX) v = INT32_MAX;
  if (v <  INT32_MIN) v = INT32_MIN;
  return (int32_t)v;
}

/* Apply zero-speed hysteresis: snap small |v_raw| to 0, then require
 * |v_raw| to exceed the upper threshold before releasing.  Avoids the
 * sign-flip chatter the quantized slope produces near rest.
 */

static int32_t apply_zero_hysteresis(struct db_observer_s *o,
                                     int32_t v_raw_mdegps)
{
  uint32_t v_abs = (uint32_t)((v_raw_mdegps < 0) ? -v_raw_mdegps
                                                 :  v_raw_mdegps);
  if (o->snapped_to_zero)
    {
      if (v_abs > DB_OBSERVER_V_HYST_HIGH_MDEGPS)
        {
          o->snapped_to_zero = false;
          return v_raw_mdegps;
        }
      return 0;
    }
  if (v_abs < DB_OBSERVER_V_HYST_LOW_MDEGPS)
    {
      o->snapped_to_zero = true;
      return 0;
    }
  return v_raw_mdegps;
}

static int32_t v_slope_mdegps(struct db_observer_s *o)
{
  int32_t v_raw = v_slope_raw_mdegps(o);
  return apply_zero_hysteresis(o, v_raw);
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
  o->min_window_us          =
    (uint64_t)(window_ms ? window_ms : DB_OBSERVER_DEFAULT_MS) * 1000u;
  o->primed                 = false;
  o->snapped_to_zero        = true;
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
  o->snapped_to_zero        = true;
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
   * outside the adaptive window.  In the limit (motor stopped, no
   * new samples for ≥ min_window_us) every pair has dx ≈ 0 so the
   * slope decays to 0; the hysteresis then snaps and latches it.
   */

  o->v_est_mdegps = v_slope_mdegps(o);
  update_stall(o, dt_ms, applied_duty_abs);
}
