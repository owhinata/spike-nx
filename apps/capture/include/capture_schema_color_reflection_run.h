/****************************************************************************
 * apps/capture/include/capture_schema_color_reflection_run.h
 *
 * Sample schema for `sensor color capture` while the LEGO Color sensor
 * is in MODE 1 (Reflection).  Rate hint 100 Hz matches the lump_sample
 * uORB topic.
 *
 * Wire layout: 9 bytes / record (u32 + i32 + u8, packed).  `distance_mm`
 * is reserved for a future revision that wires drivebase encoder state
 * in; v1 always logs zero so the C# viewer sees the field but plotting
 * libraries can ignore it.
 ****************************************************************************/

#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_COLOR_REFLECTION_RUN_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_COLOR_REFLECTION_RUN_H

#include <stddef.h>
#include <stdint.h>

#include "capture.h"

#ifdef __cplusplus
extern "C"
{
#endif

struct capture_color_reflection_run_record_s
{
  uint32_t ts_us;
  int32_t  distance_mm;
  uint8_t  reflection_pct;
} __attribute__((packed));

_Static_assert(sizeof(struct capture_color_reflection_run_record_s) == 9,
               "capture_color_reflection_run_record_s wire size");
_Static_assert(offsetof(struct capture_color_reflection_run_record_s,
                        ts_us) == 0,
               "ts_us must be the first field");

extern const struct capture_schema_s g_capture_schema_color_reflection_run;

#ifdef __cplusplus
}
#endif

#endif /* __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_COLOR_REFLECTION_RUN_H */
