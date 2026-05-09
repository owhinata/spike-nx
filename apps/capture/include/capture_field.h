/****************************************************************************
 * apps/capture/include/capture_field.h
 *
 * Field-type token mappings used by schema declarations (Issue #122).
 * The mappings are split from capture.h so a future X-macro / codegen
 * pass can re-include just this file with `CAPTURE_TOOL_GEN` defined
 * (see tools/gen_pc_schema.py).
 *
 * Token taxonomy:
 *   u8 / i8 / u16 / i16 / u32 / i32 / u64 / i64 / f32 / f64
 *
 * Each token has three derived macros:
 *   CAPTURE_TYPE_C_<tok>     - the C type used in the packed record
 *   CAPTURE_TYPE_TAG_<tok>   - the on-wire numeric tag in the .cap header
 *   CAPTURE_TYPE_SZ_<tok>    - the on-wire size in bytes
 *
 * Schema source files use these via the CAPTURE_FIELD_INIT() helper in
 * capture_schema_init.h, which assembles a `capture_field_desc_s`
 * initializer with offset / size derived from the C struct.
 ****************************************************************************/

#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_FIELD_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_FIELD_H

#include <stdint.h>

#include "capture_format.h"

/****************************************************************************
 * Pre-processor Definitions: token -> C type
 ****************************************************************************/

#define CAPTURE_TYPE_C_u8    uint8_t
#define CAPTURE_TYPE_C_i8    int8_t
#define CAPTURE_TYPE_C_u16   uint16_t
#define CAPTURE_TYPE_C_i16   int16_t
#define CAPTURE_TYPE_C_u32   uint32_t
#define CAPTURE_TYPE_C_i32   int32_t
#define CAPTURE_TYPE_C_u64   uint64_t
#define CAPTURE_TYPE_C_i64   int64_t
#define CAPTURE_TYPE_C_f32   float
#define CAPTURE_TYPE_C_f64   double

/****************************************************************************
 * Pre-processor Definitions: token -> wire tag
 ****************************************************************************/

#define CAPTURE_TYPE_TAG_u8    CAPTURE_TYPE_U8
#define CAPTURE_TYPE_TAG_i8    CAPTURE_TYPE_I8
#define CAPTURE_TYPE_TAG_u16   CAPTURE_TYPE_U16
#define CAPTURE_TYPE_TAG_i16   CAPTURE_TYPE_I16
#define CAPTURE_TYPE_TAG_u32   CAPTURE_TYPE_U32
#define CAPTURE_TYPE_TAG_i32   CAPTURE_TYPE_I32
#define CAPTURE_TYPE_TAG_u64   CAPTURE_TYPE_U64
#define CAPTURE_TYPE_TAG_i64   CAPTURE_TYPE_I64
#define CAPTURE_TYPE_TAG_f32   CAPTURE_TYPE_F32
#define CAPTURE_TYPE_TAG_f64   CAPTURE_TYPE_F64

/****************************************************************************
 * Pre-processor Definitions: token -> wire size (bytes)
 ****************************************************************************/

#define CAPTURE_TYPE_SZ_u8     1
#define CAPTURE_TYPE_SZ_i8     1
#define CAPTURE_TYPE_SZ_u16    2
#define CAPTURE_TYPE_SZ_i16    2
#define CAPTURE_TYPE_SZ_u32    4
#define CAPTURE_TYPE_SZ_i32    4
#define CAPTURE_TYPE_SZ_u64    8
#define CAPTURE_TYPE_SZ_i64    8
#define CAPTURE_TYPE_SZ_f32    4
#define CAPTURE_TYPE_SZ_f64    8

#endif /* __APPS_CAPTURE_INCLUDE_CAPTURE_FIELD_H */
