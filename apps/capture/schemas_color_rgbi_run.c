/****************************************************************************
 * apps/capture/schemas_color_rgbi_run.c
 *
 * `capture_schema_t` instance for `sensor color capture` while the
 * Color sensor is in MODE 5 (RGB+I).
 ****************************************************************************/

#include <nuttx/config.h>

#include "capture.h"
#include "capture_field.h"
#include "capture_format.h"
#include "capture_schema_color_rgbi_run.h"
#include "capture_schema_init.h"

const struct capture_schema_s g_capture_schema_color_rgbi_run =
{
  .magic        = 0x0011,
  .rate_hz_hint = 100,
  .record_size  = sizeof(struct capture_color_rgbi_run_record_s),
  .field_count  = 5,
  .name         = "color_rgbi_run",
  .fields       =
  {
    CAPTURE_FIELD_INIT(color_rgbi_run, ts_us,     u32, "us",  0),
    CAPTURE_FIELD_INIT(color_rgbi_run, red,       u16, "raw", 0),
    CAPTURE_FIELD_INIT(color_rgbi_run, green,     u16, "raw", 0),
    CAPTURE_FIELD_INIT(color_rgbi_run, blue,      u16, "raw", 0),
    CAPTURE_FIELD_INIT(color_rgbi_run, intensity, u16, "raw", 0),
  },
};
