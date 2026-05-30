/****************************************************************************
 * apps/capture/include/capture_schema_linetrace_lap_run.h
 *
 * Schema for a per-tick line-tracing "lap" trace recorded by the
 * `linetrace` PID daemon (Issue #166, parent #163 LQG roadmap).  Each
 * tick the daemon snapshots the color sensor intensity, the intensity
 * setpoint, the turn rate it commanded to the drivebase, and the
 * achieved heading / turn rate / drive speed read back from
 * DRIVEBASE_GET_STATE.  The trace feeds P0c's offline c/bias-drift
 * fitter.
 *
 * Wire layout: 19 bytes / record (packed; see the table in
 * docs/{ja,en}/development/capture-schemas.md).
 *
 * The `edge` byte tags which edge of the line the lap was driven on:
 *   0 = UNKNOWN/UNSET (P0b always writes this — the PID daemon has no
 *       edge concept yet), 1 = LEFT, 2 = RIGHT (reserved for P1a).  The
 *   offline P0c fitter must NOT treat edge==0 as a real edge; the
 *   operator supplies the c-sign out of band for P0b captures.
 ****************************************************************************/

#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_LINETRACE_LAP_RUN_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_LINETRACE_LAP_RUN_H

#include <stddef.h>
#include <stdint.h>

#include "capture.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct capture_linetrace_lap_run_record_s
{
  uint32_t ts_us;          /* CLOCK_BOOTTIME low 32 bits, us since arm   */
  uint16_t intensity;      /* color MODE 5 channel 4, 0..1024            */
  uint16_t target;         /* intensity setpoint at this tick            */
  int16_t  turn_cmd_dps;   /* turn rate commanded to drivebase (FOREVER) */
  int32_t  heading_mdeg;   /* drivebase angle_mdeg (gyro-fused heading)  */
  int16_t  turn_rate_dps;  /* achieved turn rate from GET_STATE          */
  int16_t  speed_mmps;     /* achieved drive speed from GET_STATE        */
  uint8_t  edge;           /* 0 = unset (P0b); LEFT/RIGHT in P1a         */
} __attribute__((packed));

_Static_assert(sizeof(struct capture_linetrace_lap_run_record_s) == 19,
               "capture_linetrace_lap_run_record_s wire size");
_Static_assert(offsetof(struct capture_linetrace_lap_run_record_s,
                        ts_us) == 0,
               "ts_us must be the first field");

extern const struct capture_schema_s g_capture_schema_linetrace_lap_run;

#ifdef __cplusplus
}
#endif

#endif /* __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_LINETRACE_LAP_RUN_H */
