/****************************************************************************
 * apps/drivebase/drivebase_trajectory.h
 *
 * Trapezoidal motion profile generator for the drivebase daemon
 * (Issue #77).  Produces (position, velocity, acceleration) reference
 * triples that the per-motor servo (commit #6) tracks and the
 * drivebase aggregator (commit #7) splits between left and right.
 *
 * Units throughout:
 *   - time:         µs (uint64_t, to match the daemon's clock)
 *   - position:     milli-deg of motor angle (int64_t)
 *   - velocity:     milli-deg/s (int32_t, signed)
 *   - acceleration: milli-deg/s² (int32_t, signed)
 *
 * pybricks reference: lib/pbio/src/trajectory.c.  We keep the
 * three-segment shape but trade the millisecond ticks-per-segment
 * scheme for direct µs absolute timestamps so the daemon does not
 * need pbdrv_clock_get_100us().
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_TRAJECTORY_H
#define __APPS_DRIVEBASE_DRIVEBASE_TRAJECTORY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_trajectory_s
{
  uint64_t t0_us;            /* time origin                               */

  int64_t  x0_mdeg;          /* start position                            */
  int64_t  x1_mdeg;          /* finite-target end position (unused if     */
                             /* infinite)                                 */

  int32_t  v_peak_mdegps;    /* magnitude of cruise velocity              */
  int32_t  accel_mdegps2;
  int32_t  decel_mdegps2;
  int8_t   direction;        /* +1 / -1, set from sign(x1-x0)             */

  bool     infinite;         /* drive_forever — no decel / no end         */

  /* Pre-computed segment boundaries (relative to t0_us).  Zero if the
   * segment doesn't exist (e.g. triangular profile has cruise_dt=0).
   */

  uint64_t accel_dt_us;      /* duration of acceleration segment          */
  uint64_t cruise_dt_us;
  uint64_t decel_dt_us;
  uint64_t total_dt_us;      /* accel + cruise + decel                    */

  int64_t  x_accel_end_mdeg; /* position at end of accel segment          */
  int64_t  x_cruise_end_mdeg;
};

struct db_trajectory_ref_s
{
  int64_t  x_mdeg;
  int32_t  v_mdegps;
  int32_t  a_mdegps2;
  bool     done;             /* finite trajectory has reached end         */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Plan a finite move from x0 to x1 using a trapezoidal profile bounded
 * by v_peak / accel / decel.  If the move is too short to reach v_peak,
 * the profile degenerates to a triangle and v_peak is rescaled to the
 * peak the move actually achieves.
 */

void db_trajectory_init_position(struct db_trajectory_s *tr,
                                 uint64_t t0_us,
                                 int64_t x0_mdeg, int64_t x1_mdeg,
                                 int32_t v_peak_mdegps,
                                 int32_t accel_mdegps2,
                                 int32_t decel_mdegps2);

/* Plan an infinite move at signed v_target_mdegps starting from x0.
 * Acceleration ramps up from x0's current velocity (assumed 0) to
 * v_target at `accel_mdegps2`.  No decel, no end.  drive_forever in
 * pybricks parlance.
 */

void db_trajectory_init_forever(struct db_trajectory_s *tr,
                                uint64_t t0_us,
                                int64_t x0_mdeg,
                                int32_t v_target_mdegps,
                                int32_t accel_mdegps2);

void db_trajectory_get_reference(const struct db_trajectory_s *tr,
                                 uint64_t t_us,
                                 struct db_trajectory_ref_s *ref);

/* Convenience: returns true once the trajectory has run past
 * total_dt_us (always false for infinite).
 */

bool db_trajectory_is_done(const struct db_trajectory_s *tr,
                           uint64_t t_us);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_TRAJECTORY_H */
