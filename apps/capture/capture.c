/****************************************************************************
 * apps/capture/capture.c
 *
 * Implementation of the four-function capture API (Issue #122).  Wraps
 * the kernel-resident /dev/btcap chardev: open + REGISTER, push raw
 * bytes (with the .cap header on the first call), FINALIZE + drain to
 * IDLE, and abort.  See apps/capture/include/capture.h for contract
 * details.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board_btcap.h>

#include "capture.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CAPTURE_DRAIN_POLL_US       50000  /* 50 ms                          */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Push `buf`/`len` through `fd` looping over partial writes (the kernel
 * chardev returns whatever fits in its ring on each call when there is
 * back-pressure).  Returns 0 on success or a negated errno (-ECANCELED
 * if the session was torn down, -EINTR-style codes otherwise — treat as
 * abort because we have no timeout to fall back on).
 */

static int capture_write_all(int fd, FAR const void *buf, size_t len)
{
  FAR const uint8_t *p = (FAR const uint8_t *)buf;
  size_t remaining = len;

  while (remaining > 0)
    {
      ssize_t n = write(fd, p, remaining);
      if (n > 0)
        {
          p += n;
          remaining -= n;
          continue;
        }

      if (n == 0)
        {
          /* Should not happen with a non-empty buffer; treat as abort
           * to avoid spinning.
           */

          return -ECANCELED;
        }

      int err = errno;
      if (err == EINTR || err == ECANCELED)
        {
          return -ECANCELED;
        }

      return -err;
    }

  return 0;
}

static uint8_t capture_type_size(uint8_t type)
{
  switch (type)
    {
      case CAPTURE_TYPE_U8:
      case CAPTURE_TYPE_I8:
        return 1;

      case CAPTURE_TYPE_U16:
      case CAPTURE_TYPE_I16:
        return 2;

      case CAPTURE_TYPE_U32:
      case CAPTURE_TYPE_I32:
      case CAPTURE_TYPE_F32:
        return 4;

      case CAPTURE_TYPE_U64:
      case CAPTURE_TYPE_I64:
      case CAPTURE_TYPE_F64:
        return 8;

      default:
        return 0;
    }
}

/* Stage the .cap file header + field descriptor array into the handle.
 * Builds offsets/sizes from the schema's field type tags so the schema
 * declaration stays a single source of truth.
 */

static int capture_build_header(FAR capture_handle_t *h,
                                FAR const capture_schema_t *schema,
                                uint32_t record_count)
{
  if (schema->field_count == 0 ||
      schema->field_count > CAPTURE_MAX_FIELDS)
    {
      return -EINVAL;
    }

  uint32_t expect = 0;
  for (uint8_t i = 0; i < schema->field_count; i++)
    {
      uint8_t sz = capture_type_size(schema->fields[i].type);
      if (sz == 0)
        {
          return -EINVAL;
        }

      h->field_descs[i] = schema->fields[i];

      /* Backfill offset/size from the type tag.  Schema declarations
       * are allowed to leave these at 0; the lib computes them so the
       * wire metadata matches the packed C struct that the X-macro
       * pipeline produced.
       */

      h->field_descs[i].offset = (uint8_t)expect;
      h->field_descs[i].size   = sz;
      expect += sz;
    }

  if (expect != schema->record_size)
    {
      return -EINVAL;
    }

  /* CLOCK_BOOTTIME at the moment of session registration. */

  struct timespec ts;
  clock_gettime(CLOCK_BOOTTIME, &ts);
  uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ull
                  + (uint64_t)ts.tv_nsec / 1000ull;

  memset(&h->file_header, 0, sizeof(h->file_header));
  h->file_header.magic        = CAPTURE_FILE_MAGIC;
  h->file_header.version      = CAPTURE_FILE_VERSION;
  h->file_header.schema_magic = schema->magic;
  h->file_header.start_ts_us  = now_us;
  h->file_header.record_size  = schema->record_size;
  h->file_header.record_count = record_count;
  strncpy(h->file_header.schema_name, schema->name,
          CAPTURE_NAME_MAX);
  h->file_header.field_count  = schema->field_count;

  h->field_count = schema->field_count;

  uint64_t total_bytes = (uint64_t)sizeof(struct capture_file_header_s)
                       + (uint64_t)schema->field_count
                         * sizeof(struct capture_field_desc_s)
                       + (uint64_t)record_count * schema->record_size;
  h->total_bytes = total_bytes;
  h->bytes_written = 0;
  h->header_sent = 0;
  h->schema = schema;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int capture_init(FAR capture_handle_t *h,
                 FAR const capture_schema_t *schema,
                 uint32_t record_count)
{
  if (h == NULL || schema == NULL)
    {
      return -EINVAL;
    }

  memset(h, 0, sizeof(*h));
  h->fd = -1;

  int rc = capture_build_header(h, schema, record_count);
  if (rc < 0)
    {
      return rc;
    }

  /* total_bytes for the BT-side framing equals the bytes the library
   * will push (header + descriptors + payload).
   */

  if (h->total_bytes > UINT32_MAX)
    {
      return -E2BIG;
    }

  int fd = open(BTCAP_DEVPATH, O_WRONLY);
  if (fd < 0)
    {
      return -errno;
    }

  struct btcap_session_meta_s meta = {0};
  meta.schema_magic = schema->magic;
  meta.total_bytes  = (uint32_t)h->total_bytes;
  strncpy(meta.name, schema->name, BTCAP_NAME_MAX);

  if (ioctl(fd, BTCAPIOC_REGISTER_SESSION, (unsigned long)&meta) < 0)
    {
      int err = errno;
      close(fd);
      return -err;
    }

  h->fd = fd;
  return 0;
}

int capture_write(FAR capture_handle_t *h,
                  FAR const void *data, size_t bytes)
{
  if (h == NULL || h->fd < 0)
    {
      return -EINVAL;
    }

  /* Flush the staged header on the first call. */

  if (!h->header_sent)
    {
      int rc = capture_write_all(h->fd, &h->file_header,
                                 sizeof(h->file_header));
      if (rc < 0)
        {
          return rc;
        }

      rc = capture_write_all(h->fd, h->field_descs,
                             (size_t)h->field_count
                             * sizeof(struct capture_field_desc_s));
      if (rc < 0)
        {
          return rc;
        }

      h->header_sent = 1;
      h->bytes_written += sizeof(h->file_header)
                       + (uint64_t)h->field_count
                         * sizeof(struct capture_field_desc_s);
    }

  if (bytes == 0)
    {
      return 0;
    }

  if (data == NULL)
    {
      return -EINVAL;
    }

  int rc = capture_write_all(h->fd, data, bytes);
  if (rc == 0)
    {
      h->bytes_written += bytes;
    }

  return rc;
}

int capture_deinit(FAR capture_handle_t *h)
{
  if (h == NULL)
    {
      return -EINVAL;
    }

  if (h->fd < 0)
    {
      return 0;
    }

  int rc = 0;

  if (h->header_sent)
    {
      if (ioctl(h->fd, BTCAPIOC_FINALIZE_SESSION, 0) < 0)
        {
          rc = -errno;
        }
    }

  /* Poll until the chardev returns to IDLE.  No timeout: if the BT-side
   * reader never engages or stalls, the user is expected to kill us
   * and let the kernel release fop reclaim the session.
   */

  if (rc == 0)
    {
      for (; ; )
        {
          int32_t state = -1;
          if (ioctl(h->fd, BTCAPIOC_QUERY_STATE,
                    (unsigned long)&state) < 0)
            {
              rc = -errno;
              break;
            }

          if (state == BTCAP_STATE_IDLE)
            {
              break;
            }

          if (state == BTCAP_STATE_ABORTED)
            {
              rc = -ECANCELED;
              break;
            }

          /* Sleep ~50 ms via poll() instead of usleep() so the wait is
           * reliably interrupted by SIGINT/SIGTERM.  In NuttX usleep is
           * implemented via timed semaphores that are not always
           * interruptible by signals; poll() always returns -EINTR
           * when a signal handler runs.  We poll on the chardev fd
           * with no events of interest — the call is purely a
           * sleep-with-EINTR primitive here.
           */

          struct pollfd pfd = { .fd = h->fd, .events = 0 };
          int prc = poll(&pfd, 1, CAPTURE_DRAIN_POLL_US / 1000);
          if (prc < 0)
            {
              if (errno == EINTR)
                {
                  rc = -ECANCELED;
                  break;
                }

              rc = -errno;
              break;
            }
        }
    }

  close(h->fd);
  h->fd = -1;
  return rc;
}

int capture_abort(FAR capture_handle_t *h)
{
  if (h == NULL)
    {
      return -EINVAL;
    }

  if (h->fd < 0)
    {
      return 0;
    }

  int rc = 0;
  if (ioctl(h->fd, BTCAPIOC_ABORT_SESSION, 0) < 0)
    {
      rc = -errno;
    }

  close(h->fd);
  h->fd = -1;
  return rc;
}
