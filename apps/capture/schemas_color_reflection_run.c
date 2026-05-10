/****************************************************************************
 * apps/capture/schemas_color_reflection_run.c
 *
 * `capture_schema_t` instance for `sensor color capture` while the
 * Color sensor is in MODE 1 (Reflection).  Records the LUMP sample
 * timestamp and the reflection percent value.
 ****************************************************************************/

#include <nuttx/config.h>

#include "capture.h"
#include "capture_field.h"
#include "capture_format.h"
#include "capture_schema_color_reflection_run.h"
#include "capture_schema_init.h"

const struct capture_schema_s g_capture_schema_color_reflection_run =
{
  .magic        = 0x0010,
  .rate_hz_hint = 100,
  .record_size  = sizeof(struct capture_color_reflection_run_record_s),
  .field_count  = 2,
  .name         = "color_reflection_run",
  .fields       =
  {
    CAPTURE_FIELD_INIT(color_reflection_run, ts_us,          u32, "us", 0),
    CAPTURE_FIELD_INIT(color_reflection_run, reflection_pct, u8,  "%",  0),
  },
};
