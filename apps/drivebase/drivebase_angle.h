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
 *    = mdeg / 1000 * pi * wheel_diameter / 360
 *
 * Implementation uses int64 throughout to stay accurate for the
 * 5.9 M revolutions int32 deg can represent.  Truncation is toward
 * zero (negative values round symmetrically).
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

/* mm = mdeg * π * d_mm / (1000 * 360) */

static inline int32_t db_angle_mdeg_to_mm(int64_t mdeg, uint32_t wheel_d_mm)
{
  int64_t num = mdeg * DB_PI_NUM * (int64_t)wheel_d_mm;
  int64_t den = (int64_t)1000 * 360 * DB_PI_DEN;
  if (num >= 0)
    {
      return (int32_t)((num + den / 2) / den);
    }
  return (int32_t)((num - den / 2) / den);
}

static inline int64_t db_angle_mm_to_mdeg(int32_t mm, uint32_t wheel_d_mm)
{
  int64_t num = (int64_t)mm * 1000 * 360 * DB_PI_DEN;
  int64_t den = (int64_t)DB_PI_NUM * (int64_t)wheel_d_mm;
  if (num >= 0)
    {
      return (num + den / 2) / den;
    }
  return (num - den / 2) / den;
}

/* mm/s <-> mdeg/s share the same math. */

static inline int32_t db_angle_mdegps_to_mmps(int32_t mdegps,
                                              uint32_t wheel_d_mm)
{
  return db_angle_mdeg_to_mm((int64_t)mdegps, wheel_d_mm);
}

static inline int32_t db_angle_mmps_to_mdegps(int32_t mmps,
                                              uint32_t wheel_d_mm)
{
  int64_t mdeg = db_angle_mm_to_mdeg(mmps, wheel_d_mm);
  if (mdeg >  INT32_MAX) return INT32_MAX;
  if (mdeg <  INT32_MIN) return INT32_MIN;
  return (int32_t)mdeg;
}

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_ANGLE_H */
