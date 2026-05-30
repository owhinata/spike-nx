/****************************************************************************
 * apps/capture/schemas_linetrace_lap_run.c
 *
 * `capture_schema_t` instance for the `linetrace cap` lap trace
 * (Issue #166).  One record per PID-daemon loop tick (100 Hz default).
 ****************************************************************************/

#include <nuttx/config.h>

#include "capture.h"
#include "capture_field.h"
#include "capture_format.h"
#include "capture_schema_linetrace_lap_run.h"
#include "capture_schema_init.h"

const struct capture_schema_s g_capture_schema_linetrace_lap_run =
{
  .magic        = 0x0012,
  .rate_hz_hint = 100,           /* PID daemon default loop is 100 Hz */
  .record_size  = sizeof(struct capture_linetrace_lap_run_record_s),
  .field_count  = 8,
  .name         = "linetrace_lap_run",
  .fields       =
  {
    CAPTURE_FIELD_INIT(linetrace_lap_run, ts_us,         u32, "us",   0),
    CAPTURE_FIELD_INIT(linetrace_lap_run, intensity,     u16, "raw",  0),
    CAPTURE_FIELD_INIT(linetrace_lap_run, target,        u16, "raw",  0),
    CAPTURE_FIELD_INIT(linetrace_lap_run, turn_cmd_dps,  i16, "dps",  0),
    CAPTURE_FIELD_INIT(linetrace_lap_run, heading_mdeg,  i32, "mdeg", 0),
    CAPTURE_FIELD_INIT(linetrace_lap_run, turn_rate_dps, i16, "dps",  0),
    CAPTURE_FIELD_INIT(linetrace_lap_run, speed_mmps,    i16, "mmps", 0),
    CAPTURE_FIELD_INIT(linetrace_lap_run, edge,          u8,  "",     0),
  },
};
