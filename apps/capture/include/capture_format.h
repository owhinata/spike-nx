/****************************************************************************
 * apps/capture/include/capture_format.h
 *
 * On-the-wire / on-disk layout of a `.cap` file (Issue #122).  Every
 * byte that travels through /dev/btcap and lands in the C# CaptureViewer
 * starts with this header (64 B fixed) immediately followed by an array
 * of `capture_field_desc_s` (48 B each) and the raw record payload.
 *
 * Portability rules:
 *   - Fixed-width integer types only.  No `int`, no `long`, no
 *     pointers, no `time_t`.
 *   - All multi-byte fields are little-endian on the wire.  The hub
 *     architecture is little-endian by definition (Cortex-M4) so
 *     `__attribute__((packed))` plus `_Static_assert` is enough.
 *   - String fields are null-padded but not necessarily null-terminated
 *     when full.  Readers must respect the length.
 ****************************************************************************/

#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_FORMAT_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_FORMAT_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_FILE_MAGIC          0x42504143u   /* "CAPB" little-endian   */
#define CAPTURE_FILE_VERSION        1u

#define CAPTURE_NAME_MAX            32
#define CAPTURE_FIELD_NAME_MAX      16
#define CAPTURE_FIELD_UNIT_MAX      16

/* Wire type tags.  Match the `u8`/`i8`/... tokens accepted by the
 * `CAPTURE_FIELD()` X-macro (see capture_field.h).
 */

#define CAPTURE_TYPE_U8             0
#define CAPTURE_TYPE_I8             1
#define CAPTURE_TYPE_U16            2
#define CAPTURE_TYPE_I16            3
#define CAPTURE_TYPE_U32            4
#define CAPTURE_TYPE_I32            5
#define CAPTURE_TYPE_U64            6
#define CAPTURE_TYPE_I64            7
#define CAPTURE_TYPE_F32            8
#define CAPTURE_TYPE_F64            9

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Per-field descriptor.  48 bytes; 8-byte aligned for the unit string. */

struct capture_field_desc_s
{
  char     name[CAPTURE_FIELD_NAME_MAX];
  uint8_t  type;
  uint8_t  offset;
  uint8_t  size;
  int8_t   scale_log10;
  char     unit[CAPTURE_FIELD_UNIT_MAX];
  uint8_t  reserved[12];
} __attribute__((packed));

/* File / session header.  64 bytes; designed so fixed-size readers can
 * mmap or fread() this block without knowing the field array length.
 */

struct capture_file_header_s
{
  uint32_t magic;             /* CAPTURE_FILE_MAGIC                       */
  uint16_t version;           /* CAPTURE_FILE_VERSION                     */
  uint16_t schema_magic;      /* per-schema sentinel set by CAPTURE_SCHEMA*/
  uint64_t start_ts_us;       /* CLOCK_BOOTTIME at REGISTER time          */
  uint32_t record_size;       /* bytes per record                         */
  uint32_t record_count;      /* total records in the payload that follow */
  char     schema_name[CAPTURE_NAME_MAX];
  uint8_t  field_count;
  uint8_t  reserved[7];
} __attribute__((packed));

_Static_assert(sizeof(struct capture_field_desc_s) == 48,
               "capture_field_desc_s wire size");
_Static_assert(sizeof(struct capture_file_header_s) == 64,
               "capture_file_header_s wire size");

/* Layout invariants the reader relies on. */

_Static_assert(offsetof(struct capture_file_header_s, start_ts_us)  ==  8,
               "capture_file_header_s.start_ts_us offset");
_Static_assert(offsetof(struct capture_file_header_s, record_size)  == 16,
               "capture_file_header_s.record_size offset");
_Static_assert(offsetof(struct capture_file_header_s, record_count) == 20,
               "capture_file_header_s.record_count offset");
_Static_assert(offsetof(struct capture_file_header_s, schema_name)  == 24,
               "capture_file_header_s.schema_name offset");
_Static_assert(offsetof(struct capture_file_header_s, field_count)  == 56,
               "capture_file_header_s.field_count offset");

#ifdef __cplusplus
}
#endif

#endif /* __APPS_CAPTURE_INCLUDE_CAPTURE_FORMAT_H */
