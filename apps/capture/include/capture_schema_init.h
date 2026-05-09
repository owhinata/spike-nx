/****************************************************************************
 * apps/capture/include/capture_schema_init.h
 *
 * Helper macros for filling out a `capture_schema_s` const initializer
 * (Issue #122).  The on-wire metadata mirrors the packed C record
 * struct produced by the X-macro pass in capture_field.h; this header
 * keeps the two in sync by deriving offset/size with offsetof/sizeof
 * so the metadata cannot drift from the layout.
 *
 * Usage (typical schema-init .c):
 *
 *   const struct capture_schema_s g_capture_schema_<name> = {
 *     .magic        = 0x0010,
 *     .rate_hz_hint = 100,
 *     .record_size  = sizeof(struct capture_<name>_record_s),
 *     .field_count  = N,
 *     .name         = "<name>",
 *     .fields       = {
 *       CAPTURE_FIELD_INIT(<name>, ts_us,         u32, "us", 0),
 *       CAPTURE_FIELD_INIT(<name>, reflection_pct, u8, "%",  0),
 *       ...
 *     },
 *   };
 ****************************************************************************/

#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_INIT_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_INIT_H

#include <stddef.h>

#include "capture.h"
#include "capture_field.h"
#include "capture_format.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_FIELD_INIT(schema_name, field, type_tok, unit_str, scale)  \
    {                                                                       \
      .name        = #field,                                                \
      .type        = CAPTURE_TYPE_TAG_ ## type_tok,                         \
      .offset      = (uint8_t)offsetof(                                     \
                       struct capture_ ## schema_name ## _record_s, field), \
      .size        = (uint8_t)CAPTURE_TYPE_SZ_ ## type_tok,                 \
      .scale_log10 = (int8_t)(scale),                                       \
      .unit        = unit_str,                                              \
    }

#endif /* __APPS_CAPTURE_INCLUDE_CAPTURE_SCHEMA_INIT_H */
