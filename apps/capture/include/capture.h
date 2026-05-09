/****************************************************************************
 * apps/capture/include/capture.h
 *
 * Public API of the userspace capture library (Issue #122).  Four
 * functions wrap a `/dev/btcap` session:
 *
 *   capture_init   open + REGISTER_SESSION + stage .cap header in priv
 *                  data; non-blocking
 *   capture_write  push bytes through the chardev (header on the first
 *                  call, then caller payload); blocks until the BT-side
 *                  reader engages, returns -ECANCELED on signal/abort
 *   capture_deinit FINALIZE_SESSION + drain to IDLE + close; blocks
 *   capture_abort  ABORT_SESSION + close, never blocks
 *
 * The library is timeout-free by design: a writer waiting for the
 * reader (btsensor MODE CAPTURE) is intended to be killed by the user
 * with `kill <pid>` if they no longer want the capture.  The kernel
 * release fop reclaims the chardev session when the writer task exits
 * (see boards/spike-prime-hub/src/stm32_btcap_chardev.c).
 *
 * The handle struct is exposed so callers can place it on the stack;
 * its layout is part of the in-process ABI but the fields are private
 * (touch them only through the API).
 ****************************************************************************/

#ifndef __APPS_CAPTURE_INCLUDE_CAPTURE_H
#define __APPS_CAPTURE_INCLUDE_CAPTURE_H

#include <stddef.h>
#include <stdint.h>

#include "capture_format.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum number of fields a single schema may declare.  Bounded by the
 * 8-bit `field_count` in capture_file_header_s.  Sized so a fully
 * populated `capture_handle_t` lives on the apps/sensor 2 KiB task
 * stack with room to spare for the per-call lump_sample_s buffers and
 * the regular function-call overhead.
 *
 *   8 × 48 B (field descs) + 64 B (file header) + small ≈ 480 B,
 *   leaving > 1.5 KiB for the rest of the `do_capture` frame.
 */

#define CAPTURE_MAX_FIELDS          8

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Caller-facing schema.  Built statically (one per .h via the X-macro,
 * see capture_field.h).  `fields` mirrors the on-wire descriptor layout
 * so capture_init can copy it into the file header without re-encoding.
 */

struct capture_schema_s
{
  uint16_t magic;                                    /* schema_magic   */
  uint16_t rate_hz_hint;                             /* informational  */
  uint8_t  record_size;                              /* bytes / record */
  uint8_t  field_count;
  char     name[CAPTURE_NAME_MAX];
  struct capture_field_desc_s fields[CAPTURE_MAX_FIELDS];
};

typedef struct capture_schema_s capture_schema_t;

/* Capture session handle.  Stack-allocate this in the caller; the
 * library never heap-allocates on its behalf.
 */

struct capture_handle_s
{
  int                                  fd;            /* /dev/btcap fd  */
  uint8_t                              header_sent;   /* 1 after .cap   */
                                                      /* header push    */
  uint8_t                              field_count;
  uint16_t                             reserved;
  uint64_t                             total_bytes;   /* expected total */
  uint64_t                             bytes_written; /* progress       */
  const struct capture_schema_s       *schema;
  /* Pre-built file header + field descriptors, sent verbatim on the
   * first write().  Storing it here avoids a second malloc / heap
   * touch in the export path.
   */

  struct capture_file_header_s         file_header;
  struct capture_field_desc_s          field_descs[CAPTURE_MAX_FIELDS];
};

typedef struct capture_handle_s capture_handle_t;

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: capture_init
 *
 * Description:
 *   Open /dev/btcap and register a session against the given schema.
 *   Stages the .cap header (64 B + field_count * 48 B) inside the
 *   handle without writing it to the kernel ring; the first
 *   capture_write() call flushes the header before any caller payload.
 *
 *   This call does not block: the writer-side wait happens in
 *   capture_write().  REGISTER_SESSION fails with -EBUSY if another
 *   session is in flight on /dev/btcap; it never depends on the BT
 *   link being up or on btsensor's MODE.
 *
 * Returned Value:
 *   0 on success.  -EBUSY if /dev/btcap is non-IDLE.  -EINVAL for
 *   malformed schema (NULL, oversized field_count, etc.).  -EACCES
 *   if /dev/btcap could not be opened O_WRONLY.
 ****************************************************************************/

int capture_init(FAR capture_handle_t *h,
                 FAR const capture_schema_t *schema,
                 uint32_t record_count);

/****************************************************************************
 * Name: capture_write
 *
 * Description:
 *   Push `bytes` octets of caller payload through /dev/btcap.  On the
 *   first call, the staged .cap header is sent first; callers do not
 *   need to know about it.  Internally chunks into ring-sized writes
 *   and blocks per chunk while the reader catches up.
 *
 * Returned Value:
 *   0 once every byte has been written.  -ECANCELED if the session was
 *   aborted (peer closed the chardev, ABORT_SESSION ioctl, SIGINT-style
 *   wait interruption).  -EINVAL if the session was already finalized
 *   or never registered.
 ****************************************************************************/

int capture_write(FAR capture_handle_t *h,
                  FAR const void *data, size_t bytes);

/****************************************************************************
 * Name: capture_deinit
 *
 * Description:
 *   Send FINALIZE_SESSION, then poll QUERY_STATE until the kernel
 *   reports IDLE (i.e. the BT-side reader has drained the ring).
 *   Closes the fd in every exit path.  No timeout.
 *
 * Returned Value:
 *   0 on a clean drain.  -ECANCELED if the session was aborted while
 *   we were still waiting.
 ****************************************************************************/

int capture_deinit(FAR capture_handle_t *h);

/****************************************************************************
 * Name: capture_abort
 *
 * Description:
 *   Issue ABORT_SESSION and close the fd.  Idempotent and never
 *   blocks; safe to call from error-handling goto paths.
 *
 * Returned Value:
 *   0 on success, negated errno on a low-level failure.
 ****************************************************************************/

int capture_abort(FAR capture_handle_t *h);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_CAPTURE_INCLUDE_CAPTURE_H */
