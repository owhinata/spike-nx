/****************************************************************************
 * apps/drivebase/drivebase_observer.h
 *
 * Speed observer + stall detector for the per-motor servo.
 *
 * History.  Iteration 1 (Issue #77 commit #6.5) used a per-sample IIR
 * low-pass on (Δx / Δt); at 1-deg encoder resolution × 1 kHz LUMP
 * publish a single-pair velocity estimate is either 0 or ±1000 dps —
 * pure quantization noise.  Iteration 2 replaced that with a fixed
 * 30 ms sliding-window slope: at 100 dps the 30 ms span covers 3 deg
 * of encoder change, so the slope reads ~5 % vs ±200 % per-sample
 * noise.  Iteration 3 (Issue #136) made the window adaptive — walk
 * back until accumulated |Δx| ≥ MIN_LSB_COUNT × LSB and elapsed
 * ≥ min_window_us — so low-speed accuracy is no longer bounded by
 * a single fixed window choice.  A zero-speed hysteresis was added on
 * top to suppress sign-flip chatter near rest.
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

#define DB_OBSERVER_RING_DEPTH   128
                              /* covers 128 ms at 1 kHz publish rate.    */
                              /* Larger ring (was 64) lets the adaptive  */
                              /* slope window grow to ~128 ms at low     */
                              /* speeds while still satisfying the 3-LSB */
                              /* dx criterion — see Issue #136.          */

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
  uint64_t                    min_window_us;   /* lower bound on the     */
                                               /* adaptive slope window  */
                                               /* (Issue #136)           */
  bool                        primed;
  bool                        snapped_to_zero; /* zero-speed hysteresis: */
                                               /* true while |v_raw| has */
                                               /* not yet exceeded the   */
                                               /* release threshold      */

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
