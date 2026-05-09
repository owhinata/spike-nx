/****************************************************************************
 * apps/capture/include/capture_schema_color_rgbi_run.h
 *
 * Sample schema for `sensor color capture` while the LEGO Color sensor
 * is in MODE 5 (RGB+I, 0..255 per channel).
 *
 * Wire layout: 12 bytes / record (u32 + i32 + 4*u8, packed).  Same
 * `distance_mm` reservation as color_reflection_run.
 ****************************************************************************/

#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_COLOR_RGBI_RUN_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_COLOR_RGBI_RUN_H

#include <stddef.h>
#include <stdint.h>

#include "capture.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct capture_color_rgbi_run_record_s
{
  uint32_t ts_us;
  int32_t  distance_mm;
  uint8_t  red;
  uint8_t  green;
  uint8_t  blue;
  uint8_t  intensity;
} __attribute__((packed));

_Static_assert(sizeof(struct capture_color_rgbi_run_record_s) == 12,
               "capture_color_rgbi_run_record_s wire size");
_Static_assert(offsetof(struct capture_color_rgbi_run_record_s, ts_us) == 0,
               "ts_us must be the first field");

extern const struct capture_schema_s g_capture_schema_color_rgbi_run;

#ifdef __cplusplus
}
#endif

#endif /* __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_COLOR_RGBI_RUN_H */
