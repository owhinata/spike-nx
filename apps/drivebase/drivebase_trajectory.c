/****************************************************************************
 * apps/drivebase/drivebase_trajectory.c
 *
 * Trapezoidal motion profile generator.  Pure integer math throughout
 * — no floats — so it stays cheap on Cortex-M4 with the FPU available
 * but unused on the daemon's RT path.
 *
 * Overflow analysis (Issue #77 commit #5):
 *
 *   v_peak  ≤ 1.5e6 mdeg/s   (≈ 1500 deg/s, well above SPIKE motor
 *                              limit ≈ 800 deg/s)
 *   accel   ≤ 1.0e7 mdeg/s²
 *   ramp_dt ≤ v_peak / accel ≈ 250 ms = 250 000 µs
 *   ramp_dt²= 6.25e10
 *   accel * ramp_dt² = 6.25e17  → /2e12 → ~312 500 mdeg, fits int64
 *
 * Cruise distance is computed as v * dt, no squaring required.  All
 * intermediate products fit comfortably in int64.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include "drivebase_trajectory.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define US_PER_S   ((int64_t)1000000)
#define US2_PER_S2 ((int64_t)US_PER_S * US_PER_S)

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

/* Δx caused by a constant velocity for Δt µs:  Δx = v * Δt / 1e6 */

static inline int64_t dx_from_v(int32_t v_mdegps, uint64_t dt_us)
{
  return ((int64_t)v_mdegps * (int64_t)dt_us) / US_PER_S;
}

/* Δx caused by a constant acceleration starting from rest:
 *   Δx = a * Δt² / 2 / 1e12
 */

static inline int64_t dx_from_a(int32_t a_mdegps2, uint64_t dt_us)
{
  int64_t dt = (int64_t)dt_us;
  return ((int64_t)a_mdegps2 * dt * dt) / (2 * US2_PER_S2);
}

/* Velocity reached after Δt of constant acceleration from rest. */

static inline int32_t v_from_a(int32_t a_mdegps2, uint64_t dt_us)
{
  return (int32_t)(((int64_t)a_mdegps2 * (int64_t)dt_us) / US_PER_S);
}

/* Time to ramp from rest to v at acceleration a (us, rounded down). */

static inline uint64_t dt_to_reach_v(int32_t v_mdegps, int32_t a_mdegps2)
{
  if (a_mdegps2 <= 0)
    {
      return 0;
    }
  return ((uint64_t)v_mdegps * (uint64_t)US_PER_S) / (uint32_t)a_mdegps2;
}

/* Triangular-profile peak speed when ramp_dist_total > available
 * distance.  Solves v² = 2 * d_total * (a*d) / (a+d) by integer math.
 * Returns the magnitude of the peak speed in mdeg/s.
 */

static int32_t triangle_peak(uint64_t d_total_mdeg,
                             int32_t a, int32_t d)
{
  /* v² = 2 * d * a * d / (a+d).  Compute under int64 then sqrt.       */
  uint64_t num = 2u * d_total_mdeg * (uint64_t)a * (uint64_t)d;
  uint64_t den = (uint64_t)a + (uint64_t)d;
  if (den == 0)
    {
      return 0;
    }
  uint64_t v2 = num / den;

  /* Integer sqrt (Newton-Raphson, monotonic).  v fits in 32-bit since
   * v² < 2^32 for any realistic SPIKE drive.
   */

  uint64_t r = 0;
  uint64_t hi = v2 < (1ULL << 31) ? (1ULL << 31) : (1ULL << 32);
  uint64_t lo = 0;
  while (lo + 1 < hi)
    {
      uint64_t mid = (lo + hi) / 2;
      if (mid * mid <= v2)
        {
          lo = mid;
          r  = mid;
        }
      else
        {
          hi = mid;
        }
    }
  return (int32_t)r;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_trajectory_init_position(struct db_trajectory_s *tr,
                                 uint64_t t0_us,
                                 int64_t x0_mdeg, int64_t x1_mdeg,
                                 int32_t v_peak_mdegps,
                                 int32_t accel_mdegps2,
                                 int32_t decel_mdegps2)
{
  tr->t0_us         = t0_us;
  tr->x0_mdeg       = x0_mdeg;
  tr->x1_mdeg       = x1_mdeg;
  tr->infinite      = false;
  tr->accel_mdegps2 = accel_mdegps2 > 0 ? accel_mdegps2 : 1;
  tr->decel_mdegps2 = decel_mdegps2 > 0 ? decel_mdegps2 : tr->accel_mdegps2;

  int64_t delta = x1_mdeg - x0_mdeg;
  if (delta == 0 || v_peak_mdegps <= 0)
    {
      tr->v_peak_mdegps     = 0;
      tr->direction         = 0;
      tr->accel_dt_us       = 0;
      tr->cruise_dt_us      = 0;
      tr->decel_dt_us       = 0;
      tr->total_dt_us       = 0;
      tr->x_accel_end_mdeg  = x0_mdeg;
      tr->x_cruise_end_mdeg = x0_mdeg;
      return;
    }

  tr->direction = (delta > 0) ? 1 : -1;
  uint64_t d_abs = (uint64_t)(delta * tr->direction);

  /* Idealised distance covered during accel + decel ramps at v_peak. */

  uint64_t d_accel = (uint64_t)v_peak_mdegps * v_peak_mdegps /
                     (2u * (uint32_t)tr->accel_mdegps2);
  uint64_t d_decel = (uint64_t)v_peak_mdegps * v_peak_mdegps /
                     (2u * (uint32_t)tr->decel_mdegps2);

  if (d_accel + d_decel >= d_abs)
    {
      /* Triangular: never reach v_peak, cruise = 0. */

      int32_t v_tri = triangle_peak(d_abs, tr->accel_mdegps2,
                                    tr->decel_mdegps2);
      tr->v_peak_mdegps = v_tri;
      tr->accel_dt_us   = dt_to_reach_v(v_tri, tr->accel_mdegps2);
      tr->cruise_dt_us  = 0;
      tr->decel_dt_us   = dt_to_reach_v(v_tri, tr->decel_mdegps2);
    }
  else
    {
      tr->v_peak_mdegps = v_peak_mdegps;
      tr->accel_dt_us   = dt_to_reach_v(v_peak_mdegps, tr->accel_mdegps2);
      tr->decel_dt_us   = dt_to_reach_v(v_peak_mdegps, tr->decel_mdegps2);
      uint64_t d_cruise = d_abs - d_accel - d_decel;
      tr->cruise_dt_us  = (uint64_t)(d_cruise * US_PER_S) /
                          (uint32_t)v_peak_mdegps;
    }
  tr->total_dt_us       = tr->accel_dt_us + tr->cruise_dt_us +
                          tr->decel_dt_us;

  /* Position at end of accel segment, end of cruise segment.       */

  int64_t x_off_accel = (int64_t)dx_from_a(tr->accel_mdegps2,
                                           tr->accel_dt_us);
  int64_t x_off_cruise = x_off_accel +
                         dx_from_v(tr->v_peak_mdegps,
                                   tr->cruise_dt_us);

  tr->x_accel_end_mdeg  = x0_mdeg + (int64_t)tr->direction * x_off_accel;
  tr->x_cruise_end_mdeg = x0_mdeg + (int64_t)tr->direction * x_off_cruise;
}

void db_trajectory_init_forever(struct db_trajectory_s *tr,
                                uint64_t t0_us,
                                int64_t x0_mdeg,
                                int32_t v_target_mdegps,
                                int32_t accel_mdegps2)
{
  tr->t0_us             = t0_us;
  tr->x0_mdeg           = x0_mdeg;
  tr->x1_mdeg           = x0_mdeg;
  tr->infinite          = true;
  tr->accel_mdegps2     = accel_mdegps2 > 0 ? accel_mdegps2 : 1;
  tr->decel_mdegps2     = tr->accel_mdegps2;
  int32_t v_abs         = v_target_mdegps >= 0 ? v_target_mdegps :
                                                 -v_target_mdegps;
  tr->v_peak_mdegps     = v_abs;
  tr->direction         = (v_target_mdegps > 0) ? 1 :
                          (v_target_mdegps < 0) ? -1 : 0;
  tr->accel_dt_us       = dt_to_reach_v(v_abs, tr->accel_mdegps2);
  tr->cruise_dt_us      = 0;             /* unused for infinite        */
  tr->decel_dt_us       = 0;
  tr->total_dt_us       = 0;             /* unused                     */
  tr->x_accel_end_mdeg  = x0_mdeg + (int64_t)tr->direction *
                                    dx_from_a(tr->accel_mdegps2,
                                              tr->accel_dt_us);
  tr->x_cruise_end_mdeg = tr->x_accel_end_mdeg;
}

void db_trajectory_retarget_forever(struct db_trajectory_s *tr,
                                    uint64_t now_us,
                                    int32_t v_target_mdegps,
                                    int32_t accel_mdegps2)
{
  if (!tr->infinite)
    {
      return;
    }

  /* Snapshot current reference state so we can re-anchor (t0, x0)
   * such that the new ramp passes through this point at now_us.
   */

  struct db_trajectory_ref_s ref;
  db_trajectory_get_reference(tr, now_us, &ref);

  int32_t  new_accel    = accel_mdegps2 > 0 ? accel_mdegps2 : 1;
  int32_t  new_v_abs    = v_target_mdegps >= 0 ? v_target_mdegps :
                                                 -v_target_mdegps;
  int8_t   new_direction = (v_target_mdegps > 0) ? 1 :
                           (v_target_mdegps < 0) ? -1 : 0;
  uint64_t new_accel_dt = dt_to_reach_v(new_v_abs, new_accel);

  uint64_t new_t0 = now_us;
  int64_t  new_x0 = ref.x_mdeg;

  if (new_direction != 0)
    {
      /* Project current signed velocity onto the new direction. */

      int32_t v_in_new = ref.v_mdegps * (int32_t)new_direction;

      if (v_in_new >= (int32_t)new_v_abs)
        {
          /* Already at or above target along the new direction —
           * anchor so we land in cruise at now_us.
           */

          new_t0 = now_us - new_accel_dt;
          int64_t dx_full = dx_from_a(new_accel, new_accel_dt);
          new_x0 = ref.x_mdeg - (int64_t)new_direction * dx_full;
        }
      else if (v_in_new > 0)
        {
          /* In accel phase, currently at v_in_new along new direction —
           * anchor so the ramp from rest passes through v_in_new at
           * now_us, then continues toward the new peak.
           */

          uint64_t dt_into = dt_to_reach_v(v_in_new, new_accel);
          new_t0 = now_us - dt_into;
          int64_t dx_partial = dx_from_a(new_accel, dt_into);
          new_x0 = ref.x_mdeg - (int64_t)new_direction * dx_partial;
        }
      /* else: wrong direction or zero — restart from rest at ref.x   */
    }

  tr->t0_us             = new_t0;
  tr->x0_mdeg           = new_x0;
  tr->accel_mdegps2     = new_accel;
  tr->decel_mdegps2     = new_accel;
  tr->v_peak_mdegps     = new_v_abs;
  tr->direction         = new_direction;
  tr->accel_dt_us       = new_accel_dt;
  tr->x_accel_end_mdeg  = new_x0 + (int64_t)new_direction *
                                   dx_from_a(new_accel, new_accel_dt);
  tr->x_cruise_end_mdeg = tr->x_accel_end_mdeg;
}

void db_trajectory_get_reference(const struct db_trajectory_s *tr,
                                 uint64_t t_us,
                                 struct db_trajectory_ref_s *ref)
{
  if (t_us < tr->t0_us)
    {
      ref->x_mdeg    = tr->x0_mdeg;
      ref->v_mdegps  = 0;
      ref->a_mdegps2 = 0;
      ref->done      = false;
      return;
    }

  uint64_t dt = t_us - tr->t0_us;

  /* No-op trajectory (zero distance / zero velocity). */

  if (tr->direction == 0)
    {
      ref->x_mdeg    = tr->x0_mdeg;
      ref->v_mdegps  = 0;
      ref->a_mdegps2 = 0;
      ref->done      = !tr->infinite;
      return;
    }

  /* Phase 1: accel.  Goes up to accel_dt_us; for the infinite case
   * there is no following cruise segment — instead we hold v_peak
   * forever after the accel completes.
   */

  if (dt < tr->accel_dt_us)
    {
      int32_t v_abs   = v_from_a(tr->accel_mdegps2, dt);
      int64_t dx_abs  = dx_from_a(tr->accel_mdegps2, dt);
      ref->v_mdegps   = (int32_t)tr->direction * v_abs;
      ref->a_mdegps2  = (int32_t)tr->direction * tr->accel_mdegps2;
      ref->x_mdeg     = tr->x0_mdeg + (int64_t)tr->direction * dx_abs;
      ref->done       = false;
      return;
    }

  if (tr->infinite)
    {
      uint64_t dt_after_accel = dt - tr->accel_dt_us;
      int64_t dx_abs = (int64_t)tr->v_peak_mdegps *
                       (int64_t)dt_after_accel / US_PER_S;
      ref->v_mdegps  = (int32_t)tr->direction * tr->v_peak_mdegps;
      ref->a_mdegps2 = 0;
      ref->x_mdeg    = tr->x_accel_end_mdeg +
                       (int64_t)tr->direction * dx_abs;
      ref->done      = false;
      return;
    }

  /* Phase 2: cruise (finite trajectories only). */

  uint64_t cruise_end_dt = tr->accel_dt_us + tr->cruise_dt_us;
  if (dt < cruise_end_dt)
    {
      uint64_t dt_in_cruise = dt - tr->accel_dt_us;
      int64_t  dx_abs       = (int64_t)tr->v_peak_mdegps *
                              (int64_t)dt_in_cruise / US_PER_S;
      ref->v_mdegps  = (int32_t)tr->direction * tr->v_peak_mdegps;
      ref->a_mdegps2 = 0;
      ref->x_mdeg    = tr->x_accel_end_mdeg +
                       (int64_t)tr->direction * dx_abs;
      ref->done      = false;
      return;
    }

  /* Phase 3: decel. */

  if (dt < tr->total_dt_us)
    {
      uint64_t dt_in_decel = dt - cruise_end_dt;
      int32_t  v_abs       = tr->v_peak_mdegps -
                             v_from_a(tr->decel_mdegps2, dt_in_decel);
      if (v_abs < 0) v_abs = 0;
      int64_t dx_abs = (int64_t)tr->v_peak_mdegps *
                       (int64_t)dt_in_decel / US_PER_S -
                       dx_from_a(tr->decel_mdegps2, dt_in_decel);
      ref->v_mdegps   = (int32_t)tr->direction * v_abs;
      ref->a_mdegps2  = -(int32_t)tr->direction * tr->decel_mdegps2;
      ref->x_mdeg     = tr->x_cruise_end_mdeg +
                        (int64_t)tr->direction * dx_abs;
      ref->done       = false;
      return;
    }

  /* Past the decel boundary: latch endpoint, mark done. */

  ref->x_mdeg     = tr->x1_mdeg;
  ref->v_mdegps   = 0;
  ref->a_mdegps2  = 0;
  ref->done       = true;
}

bool db_trajectory_is_done(const struct db_trajectory_s *tr,
                           uint64_t t_us)
{
  if (tr->infinite || tr->direction == 0)
    {
      return false;
    }
  return (t_us >= tr->t0_us + tr->total_dt_us);
}
