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

#include <errno.h>
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
 *
 * Overflow note (Issue #144 Step 0): the naive `2 * d_total * a * d`
 * factor reaches ~1.7e22 at realistic upper bounds (d_total ~2.6e8 mdeg
 * for a 10 m straight, a=d ~5.76e6 mdeg/s² with default 4× ramp), which
 * exceeds UINT64_MAX (~1.8e19).  We compute the smaller factor
 * `k = 2*a*d/(a+d)` first, then `v² = d_total * k`.  The triangular
 * branch invariant in db_trajectory_init_position guarantees
 *   d_total <= v_max² / k     (i.e. v² <= v_max²)
 * so d_total*k fits in int64 (≤ INT32_MAX² ≈ 4.6e18).  k_num itself
 * (2*a*d) is bounded by 2*INT32_MAX² (≈ 9.2e18), which fits uint64.
 * A defensive runtime guard catches caller invariant violations.
 */

static int32_t triangle_peak(uint64_t d_total_mdeg,
                             int32_t a, int32_t d)
{
  if (a <= 0 || d <= 0)
    {
      return 0;
    }
  uint64_t den = (uint64_t)a + (uint64_t)d;
  if (den == 0)
    {
      return 0;
    }
  uint64_t k_num = 2ULL * (uint64_t)a * (uint64_t)d;
  uint64_t k     = k_num / den;
  if (k == 0)
    {
      return 0;
    }
  if (d_total_mdeg > UINT64_MAX / k)
    {
      /* Defensive: caller violated the triangular-branch invariant.
       * Returning the int32 ceiling lets callers recover (saturates).
       */

      return INT32_MAX;
    }
  uint64_t v2 = d_total_mdeg * k;

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

/****************************************************************************
 * Trajectory stretch (Issue #144 Phase 4 C)
 *
 * Slow a trapezoidal trajectory down to a target total duration while
 * preserving the displacement |x1 - x0| and the accel/decel ratio.
 * Used by db_drivebase_drive_curve so distance and heading trajectories
 * finish at the same wall-clock time, eliminating the post-heading
 * tangent that current curves produce at large radii.
 *
 * pybricks reference: lib/pbio/src/trajectory.c:pbio_trajectory_stretch.
 * That version is parameterised by absolute time stamps (t1/t2/t3) and
 * solves the trapezoid analytically.  Our parameterisation uses dt
 * segments and explicit boundary positions, so we solve numerically:
 * binary search v_peak for compute_total_dt(v) ≈ T_target, then force
 * total_dt exact by setting cruise_dt = T_target - accel_dt - decel_dt.
 * The displacement residual lands in the cruise segment and is absorbed
 * by the existing end-time clamp in db_trajectory_get_reference.
 ****************************************************************************/

/* Total trajectory duration as a function of v_peak, with fixed a, d
 * and target displacement D_abs.  Used as the search target by
 * db_trajectory_stretch_to_total.  Monotonically non-increasing in v.
 */

static uint64_t stretch_total_dt(int32_t v, int32_t a, int32_t d,
                                 uint64_t D_abs)
{
  if (v <= 0)
    {
      return UINT64_MAX;
    }
  uint64_t t_a = dt_to_reach_v(v, a);
  uint64_t t_d = dt_to_reach_v(v, d);
  uint64_t d_a = dx_from_a(a, t_a);
  uint64_t d_d = dx_from_a(d, t_d);
  if (d_a + d_d >= D_abs)
    {
      /* Triangle branch: v cannot actually be reached given D_abs.
       * Total degenerates to the ramp duration alone.
       */

      return t_a + t_d;
    }
  uint64_t d_cruise = D_abs - d_a - d_d;
  uint64_t t_c      = (d_cruise * US_PER_S) / (uint32_t)v;
  return t_a + t_c + t_d;
}

int db_trajectory_stretch_to_total(struct db_trajectory_s *tr,
                                   uint64_t T_target_us)
{
  if (tr == NULL)                         return -EINVAL;
  if (tr->infinite)                       return -EINVAL;
  if (tr->direction == 0)                 return -EINVAL;
  if (tr->total_dt_us == 0)               return -EINVAL;
  if (T_target_us <= tr->total_dt_us)     return -EINVAL;

  int64_t D_abs64 = (int64_t)tr->direction *
                    (tr->x1_mdeg - tr->x0_mdeg);
  if (D_abs64 <= 0)                       return -EINVAL;
  uint64_t D_abs  = (uint64_t)D_abs64;

  int32_t a = tr->accel_mdegps2;
  int32_t d = tr->decel_mdegps2;
  if (a <= 0 || d <= 0)                   return -EINVAL;

  /* Binary search for v_peak ∈ [1, current_v_peak].  total(v) is
   * monotonically non-increasing in v: higher v -> shorter cruise ->
   * shorter total.  Floor-based integer arithmetic in stretch_total_dt
   * can introduce 1-µs plateaus, so we keep the best candidate seen
   * during the search and probe a neighbourhood at the end.
   */

  int32_t lo = 1;
  int32_t hi = tr->v_peak_mdegps;
  if (hi < 1)                             return -ERANGE;

  int32_t  best     = lo;
  uint64_t best_err = UINT64_MAX;

  while (lo + 1 < hi)
    {
      int32_t mid = (int32_t)(((int64_t)lo + hi) / 2);
      uint64_t total = stretch_total_dt(mid, a, d, D_abs);

      uint64_t err = (total >= T_target_us)
                   ? (total - T_target_us)
                   : (T_target_us - total);
      if (err < best_err) { best_err = err; best = mid; }

      if (total > T_target_us)
        {
          lo = mid;       /* still too slow, raise v */
        }
      else
        {
          hi = mid;       /* fast enough, lower v   */
        }
    }

  /* Wider neighbourhood probe to handle integer plateaus.  ±32 covers
   * the typical floor-rounding plateau width; if the trajectory limits
   * are pathological the helper still returns the best found, with the
   * residual reflected in cruise_dt below.
   */

  int32_t pmin = (best > 32) ? (best - 32) : 1;
  int32_t pmax = (best < tr->v_peak_mdegps - 32)
               ? (best + 32) : tr->v_peak_mdegps;

  /* Loop counter is int64 so the trailing probe++ at pmax==INT32_MAX
   * cannot signed-overflow even on pathological limit inputs.
   */

  for (int64_t probe = pmin; probe <= pmax; probe++)
    {
      uint64_t total = stretch_total_dt((int32_t)probe, a, d, D_abs);
      uint64_t err = (total >= T_target_us)
                   ? (total - T_target_us)
                   : (T_target_us - total);
      if (err < best_err) { best_err = err; best = (int32_t)probe; }
    }

  int32_t v_new = best;
  if (v_new <= 0)                         return -ERANGE;

  uint64_t t_a = dt_to_reach_v(v_new, a);
  uint64_t t_d = dt_to_reach_v(v_new, d);

  /* Sanity: t_a + t_d must fit within T_target_us.  Only reachable if
   * the caller fed a pathological limit pair the search couldn't solve.
   * Leave tr untouched and report -ERANGE; caller decides whether to
   * fall back gracefully.
   */

  if (t_a + t_d >= T_target_us)           return -ERANGE;

  uint64_t t_c = T_target_us - t_a - t_d;

  /* Snapshot original fields so we can roll back if the post-mutation
   * residual gate trips below.  Only the fields we're about to write
   * need preserving — x0/x1/t0/direction/infinite/accel/decel are
   * untouched per the precondition contract.
   */

  int32_t  orig_v_peak       = tr->v_peak_mdegps;
  uint64_t orig_accel_dt     = tr->accel_dt_us;
  uint64_t orig_cruise_dt    = tr->cruise_dt_us;
  uint64_t orig_decel_dt     = tr->decel_dt_us;
  uint64_t orig_total_dt     = tr->total_dt_us;
  int64_t  orig_x_accel_end  = tr->x_accel_end_mdeg;
  int64_t  orig_x_cruise_end = tr->x_cruise_end_mdeg;

  /* Commit candidate state. */

  tr->v_peak_mdegps     = v_new;
  tr->accel_dt_us       = t_a;
  tr->cruise_dt_us      = t_c;
  tr->decel_dt_us       = t_d;
  tr->total_dt_us       = T_target_us;

  int64_t x_off_accel  = (int64_t)dx_from_a(a, t_a);
  int64_t x_off_cruise = x_off_accel + dx_from_v(v_new, t_c);
  tr->x_accel_end_mdeg  = tr->x0_mdeg +
                          (int64_t)tr->direction * x_off_accel;
  tr->x_cruise_end_mdeg = tr->x0_mdeg +
                          (int64_t)tr->direction * x_off_cruise;

  /* Residual displacement gate (Issue #144 C2): sample the trajectory
   * one µs before the terminal clamp.  Floor rounding in cruise_dt_us
   * leaves a small displacement gap; v3 plan sets the bound at
   * 100 mdeg (= 0.1° of motor angle), well below the kp-derived
   * pos_tolerance (8000 mdeg @ kp_pos=50, Issue #140).  If the gap
   * exceeds the bound we roll back so callers see a clean -ERANGE
   * and can fall back to the unstretched trajectory.
   */

  struct db_trajectory_ref_s ref_end;
  db_trajectory_get_reference(tr, T_target_us - 1, &ref_end);
  int64_t residual = ref_end.x_mdeg - tr->x1_mdeg;
  if (residual < 0)
    {
      residual = -residual;
    }
  if (residual > 100)
    {
      tr->v_peak_mdegps     = orig_v_peak;
      tr->accel_dt_us       = orig_accel_dt;
      tr->cruise_dt_us      = orig_cruise_dt;
      tr->decel_dt_us       = orig_decel_dt;
      tr->total_dt_us       = orig_total_dt;
      tr->x_accel_end_mdeg  = orig_x_accel_end;
      tr->x_cruise_end_mdeg = orig_x_cruise_end;
      return -ERANGE;
    }

  return 0;
}
