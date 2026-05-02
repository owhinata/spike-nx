/****************************************************************************
 * apps/drivebase/drivebase_internal.h
 *
 * Daemon-internal types and helpers shared across the drivebase
 * userspace modules (Issue #77).  Not exported beyond apps/drivebase/.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_INTERNAL_H
#define __APPS_DRIVEBASE_DRIVEBASE_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/* L/R identifier for the two-motor drivebase.  L is bound to
 * /dev/uorb/sensor_motor_l (odd port = B/D/F), R to sensor_motor_r
 * (even port = A/C/E).  No other configuration is supported in this
 * Issue (see plan §"対応構成").
 */

enum db_side_e
{
  DB_SIDE_LEFT  = 0,
  DB_SIDE_RIGHT = 1,
  DB_SIDE_NUM   = 2,
};

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_INTERNAL_H */
