/****************************************************************************
 * apps/drivebase/drivebase_angle.h
 *
 * Continuous-angle accumulator + unit conversions for the drivebase
 * daemon (Issue #77).  Inputs are signed int32 deg from LUMP mode 2
 * (POS), promoted to int64 to avoid concerns about long-running
 * accumulation.  pybricks pbio uses a {rotations, millidegrees} pair
 * for the same purpose; we use a single int64 in milli-deg because
 * SPIKE Medium Motor only ever feeds us int32 deg from one source and
 * the cleaner type avoids sum/diff helpers.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_ANGLE_H
#define __APPS_DRIVEBASE_DRIVEBASE_ANGLE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Inline conversions (mdeg <-> mm)
 *
 * mm = (deg / 360) * pi * wheel_diameter
 *    = mdeg / 1000 * pi * wheel_d_um / 1000 / 360
 *    = mdeg * pi * wheel_d_um / 360,000,000
 *
 * Implementation uses int64 throughout to stay accurate for the
 * 5.9 M revolutions int32 deg can represent.  The diameter argument
 * is in micrometers (0.001 mm) so userspace can express sub-mm wheel
 * sizes (e.g. 17.6 mm = 17600 um) without an ABI hit.  A single-shot
 * mdeg * 355 * d_um multiply would overflow int64 in the worst case
 * (2.1e12 * 355 * 5e5 ~= 3.7e20), so the math is split into two steps:
 * first absorb pi (mdeg * 355 / 113 ~= mdeg * pi, +-0.5 ulp), then
 * multiply by d_um and divide by 360,000,000.  Worst-case intermediate
 * <= 6.6e12 * 5e5 = 3.3e18 which fits in int64 with 3x margin.
 *
 * Truncation is toward zero (negative values round symmetrically).
 ****************************************************************************/

#define DB_PI_NUM   355  /* π ≈ 355/113, error < 9e-8                     */
#define DB_PI_DEN   113

/* Promote one frame's signed int32 deg into int64 milli-deg. */

static inline int64_t db_angle_deg_to_mdeg(int32_t deg)
{
  return (int64_t)deg * 1000;
}

static inline int32_t db_angle_mdeg_to_deg(int64_t mdeg)
{
  if (mdeg >= 0)
    {
      return (int32_t)((mdeg + 500) / 1000);
    }
  return (int32_t)((mdeg - 500) / 1000);
}

/* Two-step round-to-nearest division for signed int64.  Handles the
 * symmetric rounding that `(num + den/2) / den` only gives for num >= 0.
 */

static inline int64_t db_angle_div_round(int64_t num, int64_t den)
{
  if (num >= 0)
    {
      return (num + den / 2) / den;
    }
  return (num - den / 2) / den;
}

static inline int32_t db_angle_mdeg_to_mm(int64_t mdeg, uint32_t wheel_d_um)
{
  int64_t pi_mdeg = db_angle_div_round(mdeg * DB_PI_NUM, DB_PI_DEN);
  int64_t num     = pi_mdeg * (int64_t)wheel_d_um;
  return (int32_t)db_angle_div_round(num, 360000000LL);
}

static inline int64_t db_angle_mm_to_mdeg(int32_t mm, uint32_t wheel_d_um)
{
  if (wheel_d_um == 0)
    {
      return 0;
    }
  int64_t pi_mm = db_angle_div_round((int64_t)mm * 360000000LL,
                                     (int64_t)wheel_d_um);
  return db_angle_div_round(pi_mm * DB_PI_DEN, DB_PI_NUM);
}

/* mm/s <-> mdeg/s share the same math. */

static inline int32_t db_angle_mdegps_to_mmps(int32_t mdegps,
                                              uint32_t wheel_d_um)
{
  return db_angle_mdeg_to_mm((int64_t)mdegps, wheel_d_um);
}

static inline int32_t db_angle_mmps_to_mdegps(int32_t mmps,
                                              uint32_t wheel_d_um)
{
  int64_t mdeg = db_angle_mm_to_mdeg(mmps, wheel_d_um);
  if (mdeg >  INT32_MAX) return INT32_MAX;
  if (mdeg <  INT32_MIN) return INT32_MIN;
  return (int32_t)mdeg;
}

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_ANGLE_H */
