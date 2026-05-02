/****************************************************************************
 * apps/drivebase/drivebase_observer.h
 *
 * Speed observer + stall detector for the per-motor servo (Issue #77
 * commit #5).  Inputs are the (timestamp, position) samples produced
 * by drivebase_motor_drain(); outputs are the smoothed speed estimate
 * the PID consumes and a stall flag the completion logic uses.
 *
 * The estimator is a first-order IIR low-pass on (Δx / Δt).  pybricks
 * uses a richer Luenberger observer with motor electrical-side state
 * (`pbio/src/observer.c`), but the encoder is good enough on the
 * Medium Motor that an LP filter at α ≈ 0.4 already gives < 5 deg/s
 * RMS noise.  We can swap in the Luenberger version later behind the
 * same API if bench measurements show the LP isn't enough.
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
 * Types
 ****************************************************************************/

struct db_observer_s
{
  /* Last consumed encoder sample */

  int64_t  x_mdeg;             /* cumulative angle in milli-deg          */
  uint64_t t_us;
  bool     primed;             /* false until first update_sample()      */

  /* Filter state */

  int32_t  v_est_mdegps;
  uint32_t alpha_q15;          /* IIR coefficient × 32768                */

  /* Stall detection (consecutive ms with low speed AND high duty) */

  uint32_t stall_low_speed_mdegps;
  uint32_t stall_min_duty;
  uint32_t stall_window_ms;
  uint32_t stall_streak_ms;
  bool     stalled;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void db_observer_init(struct db_observer_s *o,
                      uint32_t low_speed_mdegps,
                      uint32_t min_duty,
                      uint32_t stall_window_ms,
                      uint32_t alpha_q15);

void db_observer_reset(struct db_observer_s *o,
                       int64_t x_mdeg, uint64_t t_us);

/* Feed one fresh encoder sample.  Updates v_est_mdegps and stall_streak.
 * `applied_duty` is the magnitude of the most-recent PWM duty (0..10000)
 * — used together with v_est for the stall heuristic.
 */

void db_observer_update_sample(struct db_observer_s *o,
                               int64_t x_mdeg, uint64_t t_us,
                               uint32_t applied_duty_abs);

/* Tick variant: no fresh sample arrived this tick.  Bumps stall_streak
 * if the current state still satisfies the low-speed-with-duty
 * condition; does *not* update the velocity estimate (we keep the
 * previous one rather than decay artificially).
 */

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
