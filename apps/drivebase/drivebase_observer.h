/****************************************************************************
 * apps/drivebase/drivebase_observer.h
 *
 * Speed observer + stall detector for the per-motor servo (Issue #77
 * commit #6.5).  The first cut was a per-sample IIR low-pass on
 * (Δx / Δt), but at 1-deg encoder resolution × 1 kHz LUMP publish a
 * single-pair velocity estimate is either 0 or ±1000 deg/s — pure
 * quantization noise that the LP filter cannot recover from no matter
 * how aggressive α is.
 *
 * The replacement keeps a ring of recent (timestamp, position) pairs
 * spanning a configurable window (default ≈ 30 ms).  Velocity = slope
 * between the newest and the oldest entry still in the window.  At
 * 100 deg/s motor motion the 30 ms span covers 3 deg of encoder change
 * — well above the 1-deg quantization, so the slope reads the real
 * velocity within ~5 % rather than ±200 % per-sample noise.
 *
 * Pybricks uses a fuller Luenberger observer with motor electrical
 * state.  We can swap that in later behind the same API if bench
 * measurements show the slope estimator is not enough.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_OBSERVER_H
#define __APPS_DRIVEBASE_DRIVEBASE_OBSERVER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_OBSERVER_RING_DEPTH   64
                              /* covers 64 ms at 1 kHz publish rate;     */
                              /* tick consumes 1-5 entries per 5 ms tick */

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_observer_sample_s
{
  uint64_t t_us;
  int64_t  x_mdeg;
};

struct db_observer_s
{
  /* Ring of recent samples.  Producer = update_sample() pushes to the
   * head; oldest entries fall off as the window slides forward.
   */

  struct db_observer_sample_s ring[DB_OBSERVER_RING_DEPTH];
  uint8_t                     head;     /* next write slot                */
  uint8_t                     count;    /* live entries (≤ DEPTH)         */

  /* Derived state */

  int32_t                     v_est_mdegps;
  uint64_t                    window_us;       /* slope window           */
  bool                        primed;

  /* Stall detection */

  uint32_t                    stall_low_speed_mdegps;
  uint32_t                    stall_min_duty;
  uint32_t                    stall_window_ms;
  uint32_t                    stall_streak_ms;
  bool                        stalled;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void db_observer_init(struct db_observer_s *o,
                      uint32_t low_speed_mdegps,
                      uint32_t min_duty,
                      uint32_t stall_window_ms,
                      uint32_t window_ms);

void db_observer_reset(struct db_observer_s *o,
                       int64_t x_mdeg, uint64_t t_us);

void db_observer_update_sample(struct db_observer_s *o,
                               int64_t x_mdeg, uint64_t t_us,
                               uint32_t applied_duty_abs);

void db_observer_idle_tick(struct db_observer_s *o, uint32_t dt_ms,
                           uint32_t applied_duty_abs);

static inline int32_t db_observer_v(const struct db_observer_s *o)
{
  return o->v_est_mdegps;
}

static inline bool db_observer_is_stalled(const struct db_observer_s *o)
{
  return o->stalled;
}

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_OBSERVER_H */
