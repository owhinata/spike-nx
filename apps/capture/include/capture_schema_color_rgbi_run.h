/****************************************************************************
 * apps/capture/include/capture_schema_color_rgbi_run.h
 *
 * Sample schema for `sensor color capture` while the LEGO Color sensor
 * is in MODE 5 (RGB+I).  The sensor reports four INT16 values per
 * sample with a typical raw range of 0..1024, so each channel needs
 * 16 bits.
 *
 * Wire layout: 12 bytes / record (u32 + 4*u16, packed).
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
  uint16_t red;
  uint16_t green;
  uint16_t blue;
  uint16_t intensity;
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
