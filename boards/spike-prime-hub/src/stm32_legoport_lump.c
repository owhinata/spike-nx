/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport_lump.c
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2018-2023 The Pybricks Authors
 *
 * LUMP UART protocol engine for SPIKE Prime Hub I/O ports (Issue #43).
 * Based on `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c`, ported from
 * Contiki protothreads to one NuttX kthread per port.
 *
 * Phase 2 scope (this commit): SYNC -> INFO -> ACK state machine.  After
 * a successful sync the kthread sends 100 ms `SYS_NACK` keepalive in a
 * minimal loop so the device stays alive long enough for `lump info <N>`
 * to inspect the populated `lump_device_info_s`.  DATA frame parsing,
 * mode selection, and `lump_send_data()` are deferred to Phase 3.
 *
 * Threading: one kthread per port (`SCHED_PRIORITY_DEFAULT`, 2 KB stack).
 * All six are created at boot in `stm32_legoport_lump_register()` and
 * sit blocked on a per-port wakeup sem.  The DCM handoff callback stores
 * the per-port `legoport_pin_s` pointer and posts the sem; the kthread
 * resumes from there, configures GPIO AF, opens the USART, and runs the
 * pybricks-derived sync sequence.
 *
 * On error / disconnect the kthread closes the USART, calls
 * `stm32_legoport_release_uart()` (which clears the DCM's CB pointer),
 * immediately re-registers the CB, and sleeps for an exponentially
 * lengthening backoff (100 ms -> 1 s -> 5 s -> 30 s) before re-blocking
 * on the wakeup sem for the next handoff.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>

#include <arch/board/board.h>
#include <arch/board/board_legoport.h>
#include <arch/board/board_lump.h>

#include "stm32.h"

#include "spike_prime_hub.h"
#include "stm32_legoport_uart_hw.h"

#ifdef CONFIG_LEGO_LUMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LUMP wire format constants — see `pybricks/lib/lego/lego_uart.h`.
 * Reproduced here so we don't depend on pybricks headers.  Bit
 * positions match RM0430 §28 USART irrespective of host MCU.
 */

#define LUMP_MSG_TYPE_MASK    0xC0u
#define LUMP_MSG_TYPE_SYS     0x00u
#define LUMP_MSG_TYPE_CMD     0x40u
#define LUMP_MSG_TYPE_INFO    0x80u
#define LUMP_MSG_TYPE_DATA    0xC0u

#define LUMP_MSG_SIZE_MASK    0x38u
#define LUMP_MSG_SIZE_SHIFT   3
#define LUMP_MSG_CMD_MASK     0x07u

#define LUMP_SYS_SYNC         0x00u
#define LUMP_SYS_NACK         0x02u
#define LUMP_SYS_ACK          0x04u

#define LUMP_CMD_TYPE         0x00u
#define LUMP_CMD_MODES        0x01u
#define LUMP_CMD_SPEED        0x02u
#define LUMP_CMD_SELECT       0x03u
#define LUMP_CMD_WRITE        0x04u
#define LUMP_CMD_EXT_MODE     0x06u
#define LUMP_CMD_VERSION      0x07u

#define LUMP_INFO_NAME        0x00u
#define LUMP_INFO_RAW         0x01u
#define LUMP_INFO_PCT         0x02u
#define LUMP_INFO_SI          0x03u
#define LUMP_INFO_UNITS       0x04u
#define LUMP_INFO_MAPPING     0x05u
#define LUMP_INFO_FORMAT      0x80u
#define LUMP_INFO_MODE_PLUS_8 0x20u

#define LUMP_TYPE_ID_MIN      29u
#define LUMP_TYPE_ID_MAX      101u

/* Tracks which info frames we've received during SYNC.  Mirrors
 * pybricks `EV3_UART_INFO_FLAG_*`.  We only enforce the four required
 * ones (TYPE, MODES, INFO_NAME, INFO_FORMAT) per device.
 */

#define LUMP_IF_CMD_TYPE      (1u << 0)
#define LUMP_IF_CMD_MODES     (1u << 1)
#define LUMP_IF_CMD_SPEED     (1u << 2)
#define LUMP_IF_CMD_VERSION   (1u << 3)
#define LUMP_IF_INFO_NAME     (1u << 4)
#define LUMP_IF_INFO_FORMAT   (1u << 14)
#define LUMP_IF_REQUIRED      (LUMP_IF_CMD_TYPE | LUMP_IF_CMD_MODES | \
                               LUMP_IF_INFO_NAME | LUMP_IF_INFO_FORMAT)

/* Timing (per pybricks legodev_pup_uart.c constants).
 *  - 250 ms between RX bytes during SYNC/INFO before declaring a stall
 *  - 10 ms between sending ACK and changing baud
 *  - 100 ms keepalive period in DATA state
 */

#define LUMP_IO_TIMEOUT_MS         250u
#define LUMP_ACK_BAUD_DELAY_MS     10u
#define LUMP_KEEPALIVE_MS          100u

/* Baud rate band (pybricks `EV3_UART_SPEED_*`). */

#define LUMP_BAUD_INITIAL          115200u
#define LUMP_BAUD_FALLBACK         2400u
#define LUMP_BAUD_MAX              460800u

/* Backoff schedule (ms) on consecutive sync failures.  Index 5 ==
 * `LUMP_FAULT_BACKOFF` mode (30 s retries).
 */

static const uint32_t g_backoff_ms[] = { 100u, 1000u, 5000u, 30000u, 30000u };
#define LUMP_BACKOFF_MAX           4u

/* Receive scratch buffer.  Worst case: header + 32B payload + checksum +
 * info_type byte + EXT_MODE prefix.  pybricks uses LUMP_MAX_MSG_SIZE+3.
 */

#define LUMP_RX_MSG_BYTES          (32u + 3u)

/* Kthread setup */

#define LUMP_KTHREAD_STACK         2048
#define LUMP_KTHREAD_PRIO          SCHED_PRIORITY_DEFAULT

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum lump_state_e
{
  LUMP_ST_IDLE = 0,    /* waiting for handoff */
  LUMP_ST_SYNCING,     /* sent CMD SPEED, waiting for first CMD TYPE */
  LUMP_ST_INFO,        /* receiving CMD/INFO frames before SYS ACK */
  LUMP_ST_DATA,        /* sync complete, Phase 2 = idle keepalive */
  LUMP_ST_ERR,         /* transient error — go through release/backoff */
};

struct lump_engine_s
{
  uint8_t  port;
  pid_t    kthread;

  /* DCM handoff handshake */

  const struct legoport_pin_s *pins;
  sem_t    wakeup;
  volatile bool handoff_pending;
  bool     dcm_cb_registered;

  /* Engine state machine */

  enum lump_state_e state;
  uint8_t  backoff_step;

  /* SYNC scratch (mirrors pybricks `_pbdrv_legodev_pup_uart_dev_t`) */

  uint32_t info_flags;
  uint8_t  new_mode;
  uint8_t  ext_mode;
  uint32_t new_baud_rate;
  uint8_t  err_count;

  /* RX scratch */

  uint8_t  rx_msg[LUMP_RX_MSG_BYTES];
  size_t   rx_msg_size;

  /* Last-N frame ring, dumped on timeout for diagnostic.  Each entry
   * is the header byte + first payload byte (cmd2) of a successfully
   * received frame.  Saved without a checksum or size — that's already
   * derivable from the header.
   */

#define LUMP_LASTN  8u
  uint8_t  last_hdr[LUMP_LASTN];
  uint8_t  last_cmd2[LUMP_LASTN];
  uint8_t  last_idx;
  uint8_t  last_count;

  /* Public, lock-protected info snapshot */

  mutex_t  info_lock;
  struct lump_device_info_s info;

  /* Stats */

  uint32_t rx_bytes;
  uint32_t tx_bytes;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lump_engine_s g_lump[BOARD_LEGOPORT_COUNT];
static bool g_lump_initialized;

/****************************************************************************
 * Forward Declarations
 ****************************************************************************/

static int  lump_handoff_cb(int port, const struct legoport_pin_s *pins,
                            void *priv);
static int  lump_kthread_entry(int argc, FAR char *argv[]);
static int  lump_run_session(struct lump_engine_s *e);
static int  lump_sync(struct lump_engine_s *e);
static int  lump_recv_msg(struct lump_engine_s *e, uint32_t timeout_ms);
static int  lump_parse_msg(struct lump_engine_s *e);
static int  lump_send_msg(struct lump_engine_s *e, uint8_t header,
                          const uint8_t *payload, size_t payload_len,
                          bool with_checksum);
static int  lump_send_sys(struct lump_engine_s *e, uint8_t sys_byte);
static int  lump_keepalive_loop(struct lump_engine_s *e);
static void lump_reset_session_state(struct lump_engine_s *e);
static uint8_t lump_msg_payload_size(uint8_t header);

/****************************************************************************
 * Helpers
 ****************************************************************************/

/* Decoded payload size in bytes from a LUMP header. */

static uint8_t lump_msg_payload_size(uint8_t header)
{
  uint8_t code = (header & LUMP_MSG_SIZE_MASK) >> LUMP_MSG_SIZE_SHIFT;
  return (uint8_t)(1u << (code & 0x07));
}

/* Wire size of a complete LUMP frame from the header byte alone:
 *   SYS  : 1
 *   CMD  : 1 + payload + 1 (checksum)
 *   INFO : 1 + 1 (info_type) + payload + 1 (checksum)
 *   DATA : 1 + payload + 1 (checksum)
 */

static size_t lump_msg_total_size(uint8_t header)
{
  uint8_t mtype = header & LUMP_MSG_TYPE_MASK;

  if (mtype == LUMP_MSG_TYPE_SYS)
    {
      return 1u;
    }

  size_t total = 1u + lump_msg_payload_size(header) + 1u;
  if (mtype == LUMP_MSG_TYPE_INFO)
    {
      total += 1u;
    }

  return total;
}

static uint8_t lump_xor_checksum(const uint8_t *buf, size_t len)
{
  uint8_t cs = 0xff;
  for (size_t i = 0; i < len; i++)
    {
      cs ^= buf[i];
    }
  return cs;
}

static void lump_reset_session_state(struct lump_engine_s *e)
{
  e->state           = LUMP_ST_IDLE;
  e->info_flags      = 0;
  e->new_mode        = 0;
  e->ext_mode        = 0;
  e->new_baud_rate   = LUMP_BAUD_INITIAL;
  e->err_count       = 0;
  e->rx_msg_size     = 0;

  nxmutex_lock(&e->info_lock);
  memset(&e->info, 0, sizeof(e->info));
  nxmutex_unlock(&e->info_lock);
}

/****************************************************************************
 * DCM handoff callback (DCM HPWORK context, prio 192)
 ****************************************************************************/

/* Called by the DCM (Issue #42) once `UNKNOWN_UART` is confirmed on the
 * port.  Per the contract, this MUST be lightweight — we only stash the
 * pin-descriptor pointer and post the per-port wakeup sem.  The real
 * work runs in the per-port kthread (priority 100, can block).
 *
 * Return OK so DCM transitions the port to LATCHED_UART_OWNED.
 */

static int lump_handoff_cb(int port, const struct legoport_pin_s *pins,
                           void *priv)
{
  struct lump_engine_s *e = (struct lump_engine_s *)priv;

  DEBUGASSERT(port >= 0 && port < BOARD_LEGOPORT_COUNT);
  DEBUGASSERT(e == &g_lump[port]);

  e->pins             = pins;
  e->handoff_pending  = true;

  /* Bump the sem at most once per pending handoff — kthread clears the
   * flag when it processes the request.  Multiple posts from this CB
   * cannot happen because DCM serialises invokes per port.
   */

  nxsem_post(&e->wakeup);
  return OK;
}

/****************************************************************************
 * RX path — read a complete LUMP frame
 ****************************************************************************/

/* Read one frame into `e->rx_msg`.  Sets `e->rx_msg_size` on success.
 * Returns 0 on success, -ETIMEDOUT on byte-level timeout, or other
 * negated errno.  The frame's checksum is validated for non-SYS types.
 */

static int lump_recv_msg(struct lump_engine_s *e, uint32_t timeout_ms)
{
  int ret;

  /* Header byte */

  ret = lump_uart_read_byte(e->port, &e->rx_msg[0], timeout_ms);
  if (ret < 0)
    {
      return ret;
    }
  e->rx_bytes++;

  size_t total = lump_msg_total_size(e->rx_msg[0]);
  if (total > sizeof(e->rx_msg))
    {
      return -EBADMSG;
    }

  for (size_t i = 1; i < total; i++)
    {
      ret = lump_uart_read_byte(e->port, &e->rx_msg[i], timeout_ms);
      if (ret < 0)
        {
          return ret;
        }
      e->rx_bytes++;
    }

  e->rx_msg_size = total;

  /* SYS frames have no checksum.  Other types verify XOR of header +
   * payload (+ info_type for INFO) against the trailing byte.
   */

  if (total > 1)
    {
      uint8_t cs = lump_xor_checksum(e->rx_msg, total - 1);
      if (cs != e->rx_msg[total - 1])
        {
          return -EILSEQ;
        }
    }

  return OK;
}

/****************************************************************************
 * TX path — assemble + send a frame
 ****************************************************************************/

/* SYS frames are a single byte (no checksum).  Used for SYS_ACK / SYS_NACK. */

static int lump_send_sys(struct lump_engine_s *e, uint8_t sys_byte)
{
  int ret = lump_uart_write(e->port, &sys_byte, 1, LUMP_IO_TIMEOUT_MS);
  if (ret == OK)
    {
      e->tx_bytes += 1;
    }
  return ret;
}

/* CMD / DATA frames: header + payload + xor checksum. */

static int lump_send_msg(struct lump_engine_s *e, uint8_t header,
                         const uint8_t *payload, size_t payload_len,
                         bool with_checksum)
{
  uint8_t buf[1 + 32 + 1];

  if (payload_len > 32)
    {
      return -EINVAL;
    }

  buf[0] = header;
  if (payload_len > 0 && payload != NULL)
    {
      memcpy(&buf[1], payload, payload_len);
    }

  size_t total = 1 + payload_len;

  if (with_checksum)
    {
      uint8_t cs = lump_xor_checksum(buf, total);
      buf[total] = cs;
      total += 1;
    }

  int ret = lump_uart_write(e->port, buf, total, LUMP_IO_TIMEOUT_MS);
  if (ret == OK)
    {
      e->tx_bytes += total;
    }
  return ret;
}

/* Encode a 4-byte little-endian uint32 (used for CMD SPEED payload). */

static void lump_put_u32_le(uint8_t *out, uint32_t v)
{
  out[0] = (uint8_t)(v & 0xff);
  out[1] = (uint8_t)((v >> 8) & 0xff);
  out[2] = (uint8_t)((v >> 16) & 0xff);
  out[3] = (uint8_t)((v >> 24) & 0xff);
}

static uint32_t lump_get_u32_le(const uint8_t *in)
{
  return (uint32_t)in[0] |
         ((uint32_t)in[1] << 8) |
         ((uint32_t)in[2] << 16) |
         ((uint32_t)in[3] << 24);
}

/* Header byte for a CMD frame.  Picks the smallest payload-size encoding
 * that fits.  Returns the encoded size class for the caller to pad.
 */

static uint8_t lump_cmd_header(uint8_t cmd, size_t payload_len,
                               size_t *padded_len_out)
{
  uint8_t size_code;
  size_t padded;

  if (payload_len <= 1)        { size_code = 0u; padded = 1; }
  else if (payload_len <= 2)   { size_code = 1u; padded = 2; }
  else if (payload_len <= 4)   { size_code = 2u; padded = 4; }
  else if (payload_len <= 8)   { size_code = 3u; padded = 8; }
  else if (payload_len <= 16)  { size_code = 4u; padded = 16; }
  else                         { size_code = 5u; padded = 32; }

  *padded_len_out = padded;
  return (uint8_t)(LUMP_MSG_TYPE_CMD |
                   ((size_code << LUMP_MSG_SIZE_SHIFT) & LUMP_MSG_SIZE_MASK) |
                   (cmd & LUMP_MSG_CMD_MASK));
}

/****************************************************************************
 * Frame parser — consume e->rx_msg and update engine state
 ****************************************************************************/

/* Returns 0 on a valid frame (state may transition), -EBADMSG to drop
 * the frame and continue, or -EILSEQ to abort the session.  Mirrors the
 * pybricks `pbdrv_legodev_pup_uart_parse_msg()` behaviour.
 */

static int lump_parse_msg(struct lump_engine_s *e)
{
  if (e->rx_msg_size == 0)
    {
      return -EBADMSG;
    }

  uint8_t header = e->rx_msg[0];
  uint8_t mtype  = header & LUMP_MSG_TYPE_MASK;
  uint8_t cmd    = header & LUMP_MSG_CMD_MASK;
  uint8_t mode   = cmd;
  uint8_t cmd2   = (e->rx_msg_size >= 2) ? e->rx_msg[1] : 0;

  if (mtype == LUMP_MSG_TYPE_INFO && (cmd2 & LUMP_INFO_MODE_PLUS_8))
    {
      mode += 8;
      cmd2 &= (uint8_t)~LUMP_INFO_MODE_PLUS_8;
    }
  else
    {
      mode += e->ext_mode;
    }

  if (mode >= LUMP_MAX_MODES)
    {
      /* We only carry mode_info[8] in the public struct; modes >= 8 are
       * silently dropped.  All the SPIKE/Technic/BOOST devices we plan
       * to support fit in this window.
       */
      return OK;
    }

  switch (mtype)
    {
      case LUMP_MSG_TYPE_SYS:
        if (cmd == LUMP_SYS_ACK)
          {
            if ((e->info_flags & LUMP_IF_REQUIRED) != LUMP_IF_REQUIRED)
              {
                syslog(LOG_WARNING,
                       "lump: port %c: ACK before required INFO (flags=%lx)\n",
                       'A' + e->port, (unsigned long)e->info_flags);
                return -EILSEQ;
              }

            nxmutex_lock(&e->info_lock);
            e->info.current_mode = e->new_mode;
            nxmutex_unlock(&e->info_lock);

            /* Pybricks transitions to STATUS_ACK at this point; we use
             * the same boundary to break out of the INFO loop and send
             * our SYS_ACK reply.
             */
          }
        break;

      case LUMP_MSG_TYPE_CMD:
        switch (cmd)
          {
            case LUMP_CMD_TYPE:
              {
                uint8_t type_id = e->rx_msg[1];
                if (type_id < LUMP_TYPE_ID_MIN ||
                    type_id > LUMP_TYPE_ID_MAX)
                  {
                    return -EBADMSG;
                  }
                e->info_flags |= LUMP_IF_CMD_TYPE;
                nxmutex_lock(&e->info_lock);
                e->info.type_id = type_id;
                nxmutex_unlock(&e->info_lock);
                break;
              }

            case LUMP_CMD_MODES:
              {
                if (e->info_flags & LUMP_IF_CMD_MODES)
                  {
                    return -EILSEQ;
                  }
                if (cmd2 > 7)  /* LUMP_MAX_MODE */
                  {
                    return -EILSEQ;
                  }
                uint8_t num_modes = (uint8_t)(cmd2 + 1);
                if (e->rx_msg_size > 5)
                  {
                    /* Powered Up extended-mode count in rx_msg[3] */
                    num_modes = (uint8_t)(e->rx_msg[3] + 1);
                  }

                if (num_modes > LUMP_MAX_MODES)
                  {
                    /* Cap at our compile-time max; the engine still
                     * SYNCs successfully, only modes >= MAX are unseen.
                     */
                    num_modes = LUMP_MAX_MODES;
                  }

                e->info_flags |= LUMP_IF_CMD_MODES;
                e->new_mode    = (uint8_t)(num_modes - 1);

                nxmutex_lock(&e->info_lock);
                e->info.num_modes = num_modes;
                nxmutex_unlock(&e->info_lock);
                break;
              }

            case LUMP_CMD_SPEED:
              {
                if (e->info_flags & LUMP_IF_CMD_SPEED)
                  {
                    return -EILSEQ;
                  }
                uint32_t speed = lump_get_u32_le(&e->rx_msg[1]);
                if (speed < LUMP_BAUD_FALLBACK || speed > LUMP_BAUD_MAX)
                  {
                    return -EILSEQ;
                  }
                e->info_flags    |= LUMP_IF_CMD_SPEED;
                e->new_baud_rate  = speed;
                break;
              }

            case LUMP_CMD_VERSION:
              {
                if (e->info_flags & LUMP_IF_CMD_VERSION)
                  {
                    return -EILSEQ;
                  }
                uint32_t fw = lump_get_u32_le(&e->rx_msg[1]);
                uint32_t hw = lump_get_u32_le(&e->rx_msg[5]);
                e->info_flags |= LUMP_IF_CMD_VERSION;

                nxmutex_lock(&e->info_lock);
                e->info.fw_version = fw;
                e->info.hw_version = hw;
                nxmutex_unlock(&e->info_lock);
                break;
              }

            case LUMP_CMD_EXT_MODE:
              e->ext_mode = e->rx_msg[1];
              break;

            case LUMP_CMD_WRITE:
              /* Not used during SYNC; ignore.  Phase 3 will route DATA
               * mode-write payloads via lump_send_data().
               */
              break;

            default:
              return -EBADMSG;
          }
        break;

      case LUMP_MSG_TYPE_INFO:
        switch (cmd2)
          {
            case LUMP_INFO_NAME:
              {
                /* Name starts at rx_msg[2] and is up to LUMP_MAX_NAME_LEN
                 * bytes; rx_msg[size-1] is the (already-validated)
                 * checksum.  Overwrite checksum with NUL for a safe
                 * C string.
                 */
                e->rx_msg[e->rx_msg_size - 1] = 0;
                const char *name = (const char *)&e->rx_msg[2];

                /* Reject ASCII outside printable letters (pybricks parity). */

                if (e->rx_msg[2] < 'A' || e->rx_msg[2] > 'z')
                  {
                    return -EBADMSG;
                  }

                e->info_flags |= LUMP_IF_INFO_NAME;
                e->new_mode    = mode;

                nxmutex_lock(&e->info_lock);
                strncpy(e->info.modes[mode].name, name, LUMP_MAX_NAME_LEN);
                e->info.modes[mode].name[LUMP_MAX_NAME_LEN] = 0;
                nxmutex_unlock(&e->info_lock);
                break;
              }

            case LUMP_INFO_RAW:
            case LUMP_INFO_PCT:
            case LUMP_INFO_SI:
              /* 8 bytes: two LE float32 (min, max).  We store them
               * verbatim; the public struct already lays them out as
               * `float`.  Phase 2 keeps these optional — devices skip
               * them sometimes.
               */
              if (e->rx_msg_size >= 11 && mode < LUMP_MAX_MODES)
                {
                  uint32_t a = lump_get_u32_le(&e->rx_msg[2]);
                  uint32_t b = lump_get_u32_le(&e->rx_msg[6]);
                  float fa, fb;
                  memcpy(&fa, &a, sizeof(fa));
                  memcpy(&fb, &b, sizeof(fb));

                  nxmutex_lock(&e->info_lock);
                  if (cmd2 == LUMP_INFO_RAW)
                    {
                      e->info.modes[mode].raw_min = fa;
                      e->info.modes[mode].raw_max = fb;
                    }
                  else if (cmd2 == LUMP_INFO_PCT)
                    {
                      e->info.modes[mode].pct_min = fa;
                      e->info.modes[mode].pct_max = fb;
                    }
                  else
                    {
                      e->info.modes[mode].si_min = fa;
                      e->info.modes[mode].si_max = fb;
                    }
                  nxmutex_unlock(&e->info_lock);
                }
              break;

            case LUMP_INFO_UNITS:
              {
                /* Up to 4-char units string.  rx_msg[size-1] is checksum. */
                size_t copy = e->rx_msg_size > 3 ? e->rx_msg_size - 3 : 0;
                if (copy > LUMP_MAX_UNITS_LEN)
                  {
                    copy = LUMP_MAX_UNITS_LEN;
                  }
                nxmutex_lock(&e->info_lock);
                memcpy(e->info.modes[mode].units, &e->rx_msg[2], copy);
                e->info.modes[mode].units[copy] = 0;
                nxmutex_unlock(&e->info_lock);
                break;
              }

            case LUMP_INFO_MAPPING:
              if (e->rx_msg_size >= 5)
                {
                  nxmutex_lock(&e->info_lock);
                  e->info.modes[mode].writable = (e->rx_msg[3] != 0);
                  nxmutex_unlock(&e->info_lock);
                }
              break;

            case LUMP_INFO_FORMAT:
              {
                if (e->new_mode != mode)
                  {
                    return -EILSEQ;
                  }
                if (e->rx_msg_size < 7)
                  {
                    return -EILSEQ;
                  }
                if ((e->info_flags & LUMP_IF_REQUIRED) !=
                    (LUMP_IF_REQUIRED & ~LUMP_IF_INFO_FORMAT))
                  {
                    /* Need TYPE/MODES/NAME before FORMAT can complete a
                     * mode.  Treat as protocol error (pybricks parity).
                     */
                    return -EILSEQ;
                  }

                e->info_flags |= LUMP_IF_INFO_FORMAT;

                nxmutex_lock(&e->info_lock);
                e->info.modes[mode].num_values = e->rx_msg[2];
                e->info.modes[mode].data_type  = e->rx_msg[3];
                nxmutex_unlock(&e->info_lock);

                if (e->new_mode > 0)
                  {
                    e->new_mode--;
                    /* Clear NAME / FORMAT bits for the next mode round-
                     * trip.  Pybricks line 436 does this: subsequent
                     * INFO_NAME re-sets them before INFO_FORMAT closes.
                     */
                    e->info_flags &= ~(LUMP_IF_INFO_NAME |
                                       LUMP_IF_INFO_FORMAT);
                  }
                break;
              }

            default:
              /* Unknown INFO subtype — silently drop (pybricks-style). */
              break;
          }
        break;

      case LUMP_MSG_TYPE_DATA:
        /* Phase 2: discard.  Phase 3 will route to on_data callback. */
        break;
    }

  return OK;
}

/****************************************************************************
 * SYNC sequence (pybricks `synchronize_thread` port)
 ****************************************************************************/

static int lump_sync(struct lump_engine_s *e)
{
  int ret;

  e->state = LUMP_ST_SYNCING;

  /* 1. Send `CMD SPEED 115200` to ask the device to talk fast. */

  uint8_t speed_payload[4];
  size_t  padded;
  uint8_t header = lump_cmd_header(LUMP_CMD_SPEED, 4, &padded);

  lump_put_u32_le(speed_payload, LUMP_BAUD_INITIAL);

  ret = lump_send_msg(e, header, speed_payload, padded, true);
  if (ret < 0)
    {
      return ret;
    }

  /* 2. Probe for ACK at fast baud.  Pybricks waits 10 ms for any byte;
   * if it isn't ACK or we time out, drop to 2400 baud (EV3 mode).
   */

  uint8_t probe;
  ret = lump_uart_read_byte(e->port, &probe, 10);
  if (ret == -ETIMEDOUT || (ret == OK && probe != LUMP_SYS_ACK))
    {
      ret = lump_uart_set_baud(e->port, LUMP_BAUD_FALLBACK);
      if (ret < 0)
        {
          return ret;
        }

      /* Discard any bytes captured at 115200 while the device was
       * actually streaming at 2400 — they would corrupt the header
       * resync below otherwise.
       */

      lump_uart_flush_rx(e->port);
    }
  else if (ret < 0)
    {
      return ret;
    }

  /* 3+4. Resync to a valid `CMD TYPE` frame.  Pybricks structure: the
   *      inner header search has no bound — keep reading until we see
   *      the magic byte.  After that, read type_id + checksum, validate
   *      ID range + checksum.  On bad ID/checksum, increment a session-
   *      wide error counter and restart the header search.  Bail after
   *      10 consecutive bad IDs.
   */

  e->err_count = 0;

  for (;;)
    {
      /* Header search loop — no count, must see CMD_TYPE byte. */

      for (;;)
        {
          uint8_t hdr;
          ret = lump_uart_read_byte(e->port, &hdr, LUMP_IO_TIMEOUT_MS);
          if (ret == -ETIMEDOUT)
            {
              continue;  /* keep waiting; device may still be syncing */
            }
          if (ret < 0)
            {
              return ret;
            }
          e->rx_bytes++;

          if (hdr == (LUMP_MSG_TYPE_CMD | LUMP_CMD_TYPE))
            {
              e->rx_msg[0] = hdr;
              break;
            }
          /* otherwise: keep looking */
        }

      /* Read type_id + checksum. */

      for (size_t i = 1; i < 3; i++)
        {
          ret = lump_uart_read_byte(e->port, &e->rx_msg[i],
                                    LUMP_IO_TIMEOUT_MS);
          if (ret < 0)
            {
              return ret;
            }
          e->rx_bytes++;
        }
      e->rx_msg_size = 3;

      bool bad_id = (e->rx_msg[1] < LUMP_TYPE_ID_MIN ||
                     e->rx_msg[1] > LUMP_TYPE_ID_MAX);
      uint8_t cs  = lump_xor_checksum(e->rx_msg, 2);
      bool bad_cs = (cs != e->rx_msg[2]);

      if (bad_id || bad_cs)
        {
          if (++e->err_count > 10)
            {
              return -EILSEQ;
            }
          continue;  /* re-enter header search */
        }

      /* Good frame — parse and exit the resync loop. */

      ret = lump_parse_msg(e);
      if (ret < 0)
        {
          return ret;
        }
      break;
    }

  syslog(LOG_INFO, "lump: port %c: type_id=%u\n",
         'A' + e->port, e->info.type_id);

  /* 5. INFO loop.  Read frames until SYS_ACK or error. */

  e->state      = LUMP_ST_INFO;
  e->last_idx   = 0;
  e->last_count = 0;

  for (;;)
    {
      ret = lump_recv_msg(e, LUMP_IO_TIMEOUT_MS);
      if (ret == -EBADMSG || ret == -EILSEQ)
        {
          continue;  /* skip malformed frame */
        }
      if (ret < 0)
        {
          /* Dump the last-N frame ring for forensics. */

          char trace[80];
          size_t off = 0;
          uint8_t n = e->last_count < LUMP_LASTN ?
                      e->last_count : LUMP_LASTN;
          uint8_t start = (uint8_t)((e->last_idx + LUMP_LASTN - n) %
                                    LUMP_LASTN);
          for (uint8_t i = 0; i < n; i++)
            {
              uint8_t k = (uint8_t)((start + i) % LUMP_LASTN);
              int w = snprintf(trace + off, sizeof(trace) - off,
                               "%02x:%02x ",
                               e->last_hdr[k], e->last_cmd2[k]);
              if (w <= 0 || (size_t)w >= sizeof(trace) - off) break;
              off += (size_t)w;
            }

          syslog(LOG_WARNING,
                 "lump: port %c: INFO timeout flags=0x%lx new_mode=%u "
                 "num_modes=%u rx=%lu ore=%u last=[%s]\n",
                 'A' + e->port, (unsigned long)e->info_flags,
                 e->new_mode, (unsigned)e->info.num_modes,
                 (unsigned long)e->rx_bytes,
                 (unsigned)lump_uart_get_ore_count(e->port),
                 trace);
          return ret;
        }

      /* Record last-N frame for forensics. */

      e->last_hdr[e->last_idx]  = e->rx_msg[0];
      e->last_cmd2[e->last_idx] = (uint8_t)
          (e->rx_msg_size >= 2 ? e->rx_msg[1] : 0);
      e->last_idx = (uint8_t)((e->last_idx + 1) % LUMP_LASTN);
      if (e->last_count < LUMP_LASTN)
        {
          e->last_count++;
        }

      /* Detect SYS_ACK before parse so we can break the loop. */

      if (e->rx_msg[0] == (LUMP_MSG_TYPE_SYS | LUMP_SYS_ACK))
        {
          if ((e->info_flags & LUMP_IF_REQUIRED) != LUMP_IF_REQUIRED)
            {
              syslog(LOG_WARNING,
                     "lump: port %c: ACK before required INFO "
                     "(flags=0x%lx)\n",
                     'A' + e->port, (unsigned long)e->info_flags);
              return -EILSEQ;
            }
          break;
        }

      ret = lump_parse_msg(e);
      if (ret == -EILSEQ)
        {
          syslog(LOG_WARNING,
                 "lump: port %c: parse err on hdr=0x%02x size=%u\n",
                 'A' + e->port, e->rx_msg[0],
                 (unsigned)e->rx_msg_size);
          return ret;
        }
      /* -EBADMSG = drop frame, continue */
    }

  /* 6. Reply with SYS_ACK, wait 10 ms, switch to negotiated baud. */

  ret = lump_send_sys(e, LUMP_SYS_ACK);
  if (ret < 0)
    {
      return ret;
    }

  nxsig_usleep(LUMP_ACK_BAUD_DELAY_MS * 1000);

  if (e->new_baud_rate != LUMP_BAUD_INITIAL)
    {
      ret = lump_uart_set_baud(e->port, e->new_baud_rate);
      if (ret < 0)
        {
          return ret;
        }
    }

  /* 7. Mark synced.  Phase 2 enters keepalive idle; Phase 3 will run
   * the DATA state machine here instead.
   */

  nxmutex_lock(&e->info_lock);
  e->info.flags |= LUMP_FLAG_SYNCED;
  e->info.baud   = e->new_baud_rate;
  nxmutex_unlock(&e->info_lock);

  e->state = LUMP_ST_DATA;

  syslog(LOG_INFO,
         "lump: port %c: SYNCED type=%u modes=%u baud=%lu\n",
         'A' + e->port, e->info.type_id,
         (unsigned)e->info.num_modes,
         (unsigned long)e->info.baud);

  return OK;
}

/****************************************************************************
 * Keepalive loop — Phase 2 minimal: SYS_NACK every 100 ms, discard RX
 ****************************************************************************/

static int lump_keepalive_loop(struct lump_engine_s *e)
{
  /* Drain the ring buffer to clear any stale bytes from sync. */

  lump_uart_flush_rx(e->port);

  for (;;)
    {
      int ret = lump_send_sys(e, LUMP_SYS_NACK);
      if (ret < 0)
        {
          return ret;
        }

      nxsig_usleep(LUMP_KEEPALIVE_MS * 1000);

      /* Discard accumulated RX bytes so the ring doesn't permanently
       * fill (Phase 3 will parse them properly).
       */

      lump_uart_flush_rx(e->port);

      /* Check for an external "stop" event — currently the only way
       * out is `nxsem_trywait()` against the wakeup sem.  Phase 3 will
       * branch on EVT_STALL / EVT_FAULT_RECOVER bitmap entries.
       */

      if (nxsem_trywait(&e->wakeup) == OK)
        {
          /* External wake — exit keepalive and let session restart. */

          return -EINTR;
        }
    }
}

/****************************************************************************
 * Per-session driver — one handoff -> sync -> keepalive -> release cycle
 ****************************************************************************/

static int lump_run_session(struct lump_engine_s *e)
{
  int ret;
  const struct legoport_pin_s *pins = e->pins;

  /* Switch pins to UART AF.  The DCM left them in GPIO scan state. */

  stm32_configgpio(pins->uart_buf_lo);   /* enable line buffer */
  stm32_configgpio(pins->uart_tx_af);    /* TX -> USART AF */
  stm32_configgpio(pins->uart_rx_af);    /* RX -> USART AF */

  /* Open USART at LPF2 baud — the engine drops to 2400 internally if
   * the device doesn't acknowledge.
   */

  ret = lump_uart_open(e->port, LUMP_BAUD_INITIAL);
  if (ret < 0)
    {
      return ret;
    }

  ret = lump_sync(e);
  if (ret < 0)
    {
      return ret;
    }

  /* Phase 2: just keep the device alive long enough for `lump info N`
   * to show the populated info struct.  Phase 3 replaces this with a
   * full DATA state machine.
   */

  return lump_keepalive_loop(e);
}

/****************************************************************************
 * Per-port kthread main loop
 ****************************************************************************/

static int lump_kthread_entry(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      return -EINVAL;
    }

  int port = atoi(argv[1]);
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct lump_engine_s *e = &g_lump[port];

  for (;;)
    {
      /* 1. Wait for handoff (sem posted by DCM CB). */

      int ret = nxsem_wait_uninterruptible(&e->wakeup);
      if (ret < 0)
        {
          continue;
        }

      if (!e->handoff_pending)
        {
          /* Spurious wake (e.g., legacy posts).  Loop. */
          continue;
        }
      e->handoff_pending = false;

      /* 2. Run a session: open USART, sync, keepalive until error. */

      lump_reset_session_state(e);
      ret = lump_run_session(e);

      /* 3. Tear down: close USART, give pins back to DCM, re-register
       *    the handoff so the next plug-in cycles us back here.
       */

      lump_uart_close(e->port);
      stm32_legoport_release_uart(e->port);

      nxmutex_lock(&e->info_lock);
      e->info.flags = 0;  /* clear SYNCED, etc. */
      nxmutex_unlock(&e->info_lock);

      stm32_legoport_register_uart_handoff(e->port, lump_handoff_cb, e);
      e->dcm_cb_registered = true;

      /* 4. Backoff before accepting the next handoff (capped exp). */

      uint32_t sleep_ms = g_backoff_ms[e->backoff_step];
      if (ret == OK || ret == -EINTR)
        {
          /* Clean exit (e.g., explicit reset).  Reset backoff. */
          e->backoff_step = 0;
          sleep_ms        = g_backoff_ms[0];
        }
      else
        {
          syslog(LOG_WARNING,
                 "lump: port %c session ended ret=%d step=%u sleep=%lu ms\n",
                 'A' + e->port, ret, e->backoff_step,
                 (unsigned long)sleep_ms);

          if (e->backoff_step < LUMP_BACKOFF_MAX)
            {
              e->backoff_step++;
            }
        }

      nxsig_usleep(sleep_ms * 1000);
    }

  return 0;
}

/****************************************************************************
 * Public API (board_lump.h)
 ****************************************************************************/

int lump_get_info(int port, struct lump_device_info_s *out)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT || out == NULL)
    {
      return -EINVAL;
    }

  struct lump_engine_s *e = &g_lump[port];

  nxmutex_lock(&e->info_lock);
  if ((e->info.flags & LUMP_FLAG_SYNCED) == 0)
    {
      nxmutex_unlock(&e->info_lock);
      return -EAGAIN;
    }
  memcpy(out, &e->info, sizeof(*out));
  nxmutex_unlock(&e->info_lock);

  return OK;
}

int lump_get_status(int port, uint8_t *flags_out,
                    uint32_t *rx_bytes_out,
                    uint32_t *tx_bytes_out)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct lump_engine_s *e = &g_lump[port];

  nxmutex_lock(&e->info_lock);
  if (flags_out != NULL)
    {
      *flags_out = e->info.flags;
    }
  nxmutex_unlock(&e->info_lock);

  if (rx_bytes_out != NULL)
    {
      *rx_bytes_out = e->rx_bytes;
    }
  if (tx_bytes_out != NULL)
    {
      *tx_bytes_out = e->tx_bytes;
    }

  return OK;
}

/* Phase 2: lump_attach/detach/select_mode/send_data still stubbed.
 * Phase 3 will replace these with real implementations.
 */

int lump_attach(int port, const struct lump_callbacks_s *cb)
{
  UNUSED(port);
  UNUSED(cb);
  return -ENOSYS;
}

int lump_detach(int port)
{
  UNUSED(port);
  return -ENOSYS;
}

int lump_select_mode(int port, uint8_t mode)
{
  UNUSED(port);
  UNUSED(mode);
  return -ENOSYS;
}

int lump_send_data(int port, uint8_t mode, const uint8_t *buf, size_t len)
{
  UNUSED(port);
  UNUSED(mode);
  UNUSED(buf);
  UNUSED(len);
  return -ENOSYS;
}

/****************************************************************************
 * Engine bring-up (called from stm32_bringup.c)
 ****************************************************************************/

int stm32_legoport_lump_register(void)
{
  if (g_lump_initialized)
    {
      return -EALREADY;
    }

  for (int p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      struct lump_engine_s *e = &g_lump[p];

      memset(e, 0, sizeof(*e));
      e->port = (uint8_t)p;
      nxsem_init(&e->wakeup, 0, 0);
      nxmutex_init(&e->info_lock);

      /* Pre-create the per-port kthread.  It blocks immediately on
       * `wakeup`; the DCM CB is what unblocks it (or `lump reset`
       * later).  Stack is allocated once at boot per the plan note.
       */

      char name[16];
      char arg_port[4];
      char *argv[2];

      snprintf(name, sizeof(name), "lump-%c", 'A' + p);
      snprintf(arg_port, sizeof(arg_port), "%d", p);
      argv[0] = arg_port;
      argv[1] = NULL;

      e->kthread = kthread_create(name,
                                  LUMP_KTHREAD_PRIO,
                                  LUMP_KTHREAD_STACK,
                                  lump_kthread_entry,
                                  argv);
      if (e->kthread < 0)
        {
          syslog(LOG_ERR,
                 "lump: kthread_create(port %c) failed: %d\n",
                 'A' + p, (int)e->kthread);
          return (int)e->kthread;
        }

      /* Register the DCM handoff CB.  When the port detects
       * UNKNOWN_UART, this CB fires and posts e->wakeup.
       */

      int ret = stm32_legoport_register_uart_handoff(p, lump_handoff_cb, e);
      if (ret < 0)
        {
          syslog(LOG_ERR,
                 "lump: register_uart_handoff(port %c) failed: %d\n",
                 'A' + p, ret);
          return ret;
        }
      e->dcm_cb_registered = true;
    }

  g_lump_initialized = true;

  syslog(LOG_INFO,
         "lump: 6 kthreads pre-created, DCM handoff registered\n");
  return OK;
}

#endif /* CONFIG_LEGO_LUMP */
