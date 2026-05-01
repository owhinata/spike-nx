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
#include <nuttx/sched.h>
#include <nuttx/semaphore.h>
#include <nuttx/signal.h>
#include <nuttx/wdog.h>

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

#define LUMP_DATA_RECV_SLICE_MS    20u
#define LUMP_DATA_MISS_LIMIT       6u  /* 6 missed keepalives = ~600 ms */

/* Per-port watchdog timeout.  Worst case for one DATA-loop iteration is
 * the recv_msg slice (~20 ms) + a full padded frame (35 bytes × 20 ms
 * timeout each at slow line = ~700 ms) + keepalive TX (~50 ms) + TX
 * drain.  2 seconds gives a comfortable margin.
 */

#define LUMP_WATCHDOG_MS           2000u

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

  /* Phase 3: DATA-state plumbing */

  /* Pending TX requests posted by user-space ioctls / kernel API.  All
   * three guarded by `tx_lock`; the kthread drains them between byte
   * reads in the DATA loop.
   */

  mutex_t  tx_lock;
  bool     pending_select;
  uint8_t  pending_select_mode;
  bool     pending_send;
  uint8_t  pending_send_mode;
  uint8_t  pending_send_buf[LUMP_MAX_PAYLOAD];
  uint8_t  pending_send_len;

  /* Per-port DATA-frame ring served by `LEGOPORT_LUMP_POLL_DATA` ioctl
   * (used by `port lump watch <N>`).  The kthread is the producer;
   * `lump_pop_data_frame()` is the consumer.  Drops oldest on overflow.
   */

  mutex_t  dq_lock;
  struct lump_data_frame_s dq[LUMP_DATA_QUEUE];
  uint8_t  dq_head;
  uint8_t  dq_tail;
  uint8_t  dq_count;
  uint32_t dq_dropped;

  /* `lump_attach()` callback set, served by the kthread (lock-released
   * before fire to avoid deadlock — same-port re-entry is rejected by
   * `in_callback`).
   */

  mutex_t  cb_lock;
  bool     cb_attached;
  bool     in_callback;
  struct lump_callbacks_s cb;

  /* Phase 4: per-port watchdog supervisor.  Pets on each DATA-loop
   * iteration; if the kthread stalls > LUMP_WATCHDOG_MS, the wdog
   * callback (IRQ context) sets `wdog_stall` + posts the wakeup sem,
   * and the kthread context handles release/register/backoff (no
   * release_uart from IRQ context per the plan).
   */

  struct wdog_s wdog;
  volatile bool wdog_stall;

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
static int  lump_data_loop(struct lump_engine_s *e);
static void lump_reset_session_state(struct lump_engine_s *e);
static uint8_t lump_msg_payload_size(uint8_t header);

/* H-bridge HAL (boards/spike-prime-hub/src/stm32_legoport_pwm.c) — used
 * by the supply-rail auto-drive on SYNC and by the reset path on
 * disconnect / ERR.  See pybricks `legodev_pup_uart.c:894-900` /
 * `:690-694` for the equivalent calls.
 */

int stm32_legoport_pwm_pin_supply(int idx, int sign);
int stm32_legoport_pwm_unpin(int idx);

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

  /* Drop any SUPPLY rail this port may have been holding for an
   * active sensor (Color / Ultrasonic) — pybricks
   * `legodev_pup_uart.c:690-694` calls `pbdrv_motor_driver_coast()`
   * from `pbdrv_legodev_pup_uart_reset` for the same reason.
   * Calling unconditionally is safe: the HAL clears the pinned flag
   * and coasts; if the port was not pinned, this is just an extra
   * coast on an already-coasted port.
   */

  stm32_legoport_pwm_unpin(e->port);

  nxmutex_lock(&e->info_lock);
  memset(&e->info, 0, sizeof(e->info));
  nxmutex_unlock(&e->info_lock);

  /* Drop any stale TX requests + DATA queue from the previous session.
   * Callbacks survive — `lump_attach` registration outlives sessions
   * (a re-sync re-fires `on_sync` on next entry to DATA loop... well,
   * Phase 3 doesn't re-fire `on_sync` after re-sync; consumers track
   * the SYNCED flag transition themselves via lump_get_info polling
   * if they care.  #44/#45 motor/sensor drivers will likely just
   * re-attach after detach-on-disconnect.).
   */

  nxmutex_lock(&e->tx_lock);
  e->pending_select   = false;
  e->pending_send     = false;
  e->pending_send_len = 0;
  nxmutex_unlock(&e->tx_lock);

  nxmutex_lock(&e->dq_lock);
  e->dq_head    = 0;
  e->dq_tail    = 0;
  e->dq_count   = 0;
  e->dq_dropped = 0;
  nxmutex_unlock(&e->dq_lock);
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

                /* Mode capability flags (FLAGS0 byte).
                 *
                 * Per pybricks `legodev_pup_uart.c:460-465` and the LUMP
                 * spec at `lego_uart.h:323-327`: when the mode name is a
                 * "short name" (≤ 5 chars + NUL padding so it fits in
                 * bytes 2..7) AND the frame is large enough to include
                 * the 6-byte capability tail (bytes 8..13), byte 8 holds
                 * the per-mode FLAGS0 byte.  Older devices that don't
                 * announce capabilities send the same INFO_NAME frame
                 * but stop after the name — those carry no FLAGS0 and
                 * the field stays 0.
                 *
                 * `LUMP_MAX_SHORT_NAME_LEN`= 5 here matches pybricks
                 * `LUMP_MAX_SHORT_NAME_SIZE` (the spec value).  The size
                 * threshold `> 11` matches pybricks
                 * `> LUMP_MAX_NAME_SIZE` (the wire LUMP_MAX_NAME_SIZE,
                 * NOT spike-nx's struct-buffer LUMP_MAX_NAME_LEN).
                 */
                #define LUMP_MAX_SHORT_NAME_LEN  5
                #define LUMP_NAME_FRAME_WITH_FLAGS_MIN_SIZE  (11 + 1)

                size_t name_len = strlen(name);
                uint8_t mode_flags = 0;
                if (name_len <= LUMP_MAX_SHORT_NAME_LEN &&
                    e->rx_msg_size > LUMP_NAME_FRAME_WITH_FLAGS_MIN_SIZE)
                  {
                    mode_flags = e->rx_msg[8];
                  }

                nxmutex_lock(&e->info_lock);
                strncpy(e->info.modes[mode].name, name, LUMP_MAX_NAME_LEN);
                e->info.modes[mode].name[LUMP_MAX_NAME_LEN] = 0;
                e->info.modes[mode].mode_flags = mode_flags;

                /* Per pybricks `legodev_pup_uart.c:472`:
                 * "Although capabilities are sent per mode, we apply
                 * them to the whole device".  OR every mode's FLAGS0
                 * into the device-level capability_flags.
                 */
                e->info.capability_flags |= mode_flags;
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
        {
          /* Mode is the lower 3 bits of the header plus `ext_mode`
           * (CMD EXT_MODE adder).  Powered Up devices issue
           * `CMD EXT_MODE 0` / `CMD EXT_MODE 8` before each DATA group
           * to multiplex modes >= 8.
           */

          uint8_t data_mode = (uint8_t)((header & LUMP_MSG_CMD_MASK) +
                                        e->ext_mode);
          if (data_mode >= LUMP_MAX_MODES)
            {
              break;  /* unknown mode — drop silently */
            }

          uint8_t payload = lump_msg_payload_size(header);
          if (payload > LUMP_MAX_PAYLOAD)
            {
              break;
            }

          /* Update `current_mode` only on edge.  Cheap mutex round-trip
           * but only fires when the mode changes — DATA frames keep
           * coming at high rate.
           */

          nxmutex_lock(&e->info_lock);
          if (e->info.current_mode != data_mode)
            {
              e->info.current_mode = data_mode;
            }
          e->info.flags |= LUMP_FLAG_DATA_OK;
          nxmutex_unlock(&e->info_lock);

          /* Push into the user-facing data queue (drop oldest if full). */

          nxmutex_lock(&e->dq_lock);
          if (e->dq_count == LUMP_DATA_QUEUE)
            {
              e->dq_tail = (uint8_t)((e->dq_tail + 1) % LUMP_DATA_QUEUE);
              e->dq_count--;
              e->dq_dropped++;
            }
          struct lump_data_frame_s *slot = &e->dq[e->dq_head];
          slot->mode = data_mode;
          slot->len  = payload;
          memset(slot->reserved, 0, sizeof(slot->reserved));
          memcpy(slot->data, &e->rx_msg[1], payload);
          if (payload < LUMP_MAX_PAYLOAD)
            {
              memset(slot->data + payload, 0,
                     LUMP_MAX_PAYLOAD - payload);
            }
          e->dq_head = (uint8_t)((e->dq_head + 1) % LUMP_DATA_QUEUE);
          e->dq_count++;
          nxmutex_unlock(&e->dq_lock);

          /* Fire `on_data` callback.  Snapshot the cb under tx_lock so
           * we can safely call it after dropping the lock — `lump_attach`
           * uses the same lock to install / clear callbacks.
           */

          nxmutex_lock(&e->cb_lock);
          lump_on_data_t cb_local = e->cb_attached ? e->cb.on_data : NULL;
          void          *priv     = e->cb_attached ? e->cb.priv    : NULL;
          if (cb_local != NULL)
            {
              e->in_callback = true;
            }
          nxmutex_unlock(&e->cb_lock);

          if (cb_local != NULL)
            {
              cb_local(e->port, data_mode, &e->rx_msg[1], payload, priv);
              nxmutex_lock(&e->cb_lock);
              e->in_callback = false;
              nxmutex_unlock(&e->cb_lock);
            }
          break;
        }
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
  uint8_t default_mode  = e->info.current_mode;
  uint8_t num_modes     = e->info.num_modes;
  uint8_t cap_flags     = e->info.capability_flags;
  nxmutex_unlock(&e->info_lock);

  e->state = LUMP_ST_DATA;

  /* Supply rail auto-drive — mirrors pybricks
   * `legodev_pup_uart.c:894-900`.  Devices that announced
   * NEEDS_SUPPLY_PIN1 (Color / Ultrasonic) need pin1 held HIGH so the
   * H-bridge bus rail powers their LEDs / IR / photodiode circuitry;
   * the H-bridge layer translates that into "REV at MAX_DUTY".
   * NEEDS_SUPPLY_PIN2 is the mirror image (Technic Color Light Matrix).
   * Stays pinned (rejecting userspace SET_DUTY / BRAKE) until the
   * session ends — see `lump_reset_session_state()` for the unpin
   * path.
   */

  if (cap_flags & LUMP_CAP_NEEDS_SUPPLY_PIN1)
    {
      stm32_legoport_pwm_pin_supply(e->port, -1);
    }
  else if (cap_flags & LUMP_CAP_NEEDS_SUPPLY_PIN2)
    {
      stm32_legoport_pwm_pin_supply(e->port, +1);
    }

  syslog(LOG_INFO,
         "lump: port %c: SYNCED type=%u modes=%u baud=%lu\n",
         'A' + e->port, e->info.type_id,
         (unsigned)e->info.num_modes,
         (unsigned long)e->info.baud);

  /* Issue #81: auto-enqueue a CMD SELECT for the device's default
   * (current) mode after SYNC.  Mirrors the pybricks reference
   * (`pbio/drv/legodev/legodev_pup_uart.c:903`); without this, any
   * device whose default mode does not auto-emit DATA gets torn
   * down by the 600 ms no-DATA watchdog before any consumer can
   * issue a SELECT.  All SPIKE-family devices observed to date
   * auto-stream, so this is defensive parity.  The data loop drains
   * `pending_select` on the next TX window.
   */

  if (default_mode < num_modes)
    {
      nxmutex_lock(&e->tx_lock);
      e->pending_select       = true;
      e->pending_select_mode  = default_mode;
      nxmutex_unlock(&e->tx_lock);
    }

  /* Fire `on_sync` so an attached consumer (motor / sensor driver) can
   * hydrate its info cache and emit a connect sentinel on every fresh
   * SYNCING -> DATA transition (boot, re-sync after backoff, etc.).
   * Lock-released-then-fire pattern matches `on_data` / `on_error` so
   * same-port API re-entry from inside the callback is rejected via
   * `in_callback`.
   */
  {
    nxmutex_lock(&e->cb_lock);
    lump_on_sync_t sync_cb   = e->cb_attached ? e->cb.on_sync : NULL;
    void          *sync_priv = e->cb_attached ? e->cb.priv    : NULL;
    if (sync_cb != NULL)
      {
        e->in_callback = true;
      }
    nxmutex_unlock(&e->cb_lock);

    if (sync_cb != NULL)
      {
        struct lump_device_info_s snap;
        nxmutex_lock(&e->info_lock);
        memcpy(&snap, &e->info, sizeof(snap));
        nxmutex_unlock(&e->info_lock);

        sync_cb(e->port, &snap, sync_priv);

        nxmutex_lock(&e->cb_lock);
        e->in_callback = false;
        nxmutex_unlock(&e->cb_lock);
      }
  }

  return OK;
}

/****************************************************************************
 * Watchdog supervisor — Phase 4
 *
 * Pets per DATA-loop iteration; on stall, fires from the system timer
 * IRQ context and just sets a flag + posts the wakeup sem so the
 * kthread can run release/register/backoff in thread context.
 ****************************************************************************/

static void lump_wdog_handler(wdparm_t arg)
{
  struct lump_engine_s *e = (struct lump_engine_s *)arg;
  e->wdog_stall = true;
  nxsem_post(&e->wakeup);
}

static inline void lump_wdog_pet(struct lump_engine_s *e)
{
  wd_start(&e->wdog, MSEC2TICK(LUMP_WATCHDOG_MS),
           lump_wdog_handler, (wdparm_t)e);
}

static inline void lump_wdog_cancel(struct lump_engine_s *e)
{
  wd_cancel(&e->wdog);
}

/****************************************************************************
 * DATA loop — Phase 3: receive DATA frames, send keepalive, drain TX queue
 ****************************************************************************/

/* Send `CMD EXT_MODE` to multiplex mode-byte values past 7.  Required
 * before each `CMD SELECT` / DATA TX on Powered Up devices, even when
 * the target mode is < 8.
 */

static int lump_send_ext_mode(struct lump_engine_s *e, uint8_t mode)
{
  uint8_t adder = (mode > 7) ? 8 : 0;
  uint8_t hdr   = (uint8_t)(LUMP_MSG_TYPE_CMD | LUMP_CMD_EXT_MODE);
  /* size_code 0 → 1-byte payload */
  return lump_send_msg(e, hdr, &adder, 1, true);
}

/* Send `CMD SELECT <mode_lo>`.  Caller is responsible for first sending
 * `CMD EXT_MODE` if the mode is >= 8.
 */

static int lump_send_select(struct lump_engine_s *e, uint8_t mode)
{
  uint8_t hdr     = (uint8_t)(LUMP_MSG_TYPE_CMD | LUMP_CMD_SELECT);
  uint8_t mode_lo = (uint8_t)(mode & LUMP_MSG_CMD_MASK);
  return lump_send_msg(e, hdr, &mode_lo, 1, true);
}

/* Send a DATA frame for a writable mode.  Pads payload to the smallest
 * power-of-2 size that fits (1/2/4/8/16/32 bytes) and OR's the mode
 * (mod 8) into the header CMD field.  Caller has already issued
 * `CMD EXT_MODE` for `mode >= 8`.
 */

static int lump_send_data_frame(struct lump_engine_s *e, uint8_t mode,
                                const uint8_t *buf, size_t len)
{
  size_t  padded;
  uint8_t hdr_cmd_byte = (uint8_t)(mode & LUMP_MSG_CMD_MASK);
  uint8_t header;
  uint8_t size_code;

  if (len == 0 || len > LUMP_MAX_PAYLOAD)
    {
      return -EINVAL;
    }

  if (len <= 1)        { size_code = 0u; padded = 1; }
  else if (len <= 2)   { size_code = 1u; padded = 2; }
  else if (len <= 4)   { size_code = 2u; padded = 4; }
  else if (len <= 8)   { size_code = 3u; padded = 8; }
  else if (len <= 16)  { size_code = 4u; padded = 16; }
  else                 { size_code = 5u; padded = 32; }

  header = (uint8_t)(LUMP_MSG_TYPE_DATA |
                     ((size_code << LUMP_MSG_SIZE_SHIFT) &
                      LUMP_MSG_SIZE_MASK) |
                     hdr_cmd_byte);

  /* Build padded payload (zero-fill the tail). */

  uint8_t pad[LUMP_MAX_PAYLOAD];
  memcpy(pad, buf, len);
  if (padded > len)
    {
      memset(pad + len, 0, padded - len);
    }

  return lump_send_msg(e, header, pad, padded, true);
}

/* Drain pending TX requests posted by `lump_select_mode` / `lump_send_data`.
 * Called from the kthread context only, so we can take `tx_lock`, copy
 * the request, drop the lock, then issue the TX without blocking the
 * user-side ioctl.
 */

static int lump_drain_tx_requests(struct lump_engine_s *e)
{
  bool    do_select = false;
  bool    do_send   = false;
  uint8_t sel_mode  = 0;
  uint8_t send_mode = 0;
  uint8_t send_buf[LUMP_MAX_PAYLOAD];
  uint8_t send_len  = 0;

  nxmutex_lock(&e->tx_lock);
  if (e->pending_select)
    {
      do_select          = true;
      sel_mode           = e->pending_select_mode;
      e->pending_select  = false;
    }
  if (e->pending_send)
    {
      do_send            = true;
      send_mode          = e->pending_send_mode;
      send_len           = e->pending_send_len;
      memcpy(send_buf, e->pending_send_buf, send_len);
      e->pending_send    = false;
    }
  nxmutex_unlock(&e->tx_lock);

  if (do_select)
    {
      int ret = lump_send_ext_mode(e, sel_mode);
      if (ret < 0)
        {
          return ret;
        }
      ret = lump_send_select(e, sel_mode);
      if (ret < 0)
        {
          return ret;
        }
    }

  if (do_send)
    {
      int ret = lump_send_ext_mode(e, send_mode);
      if (ret < 0)
        {
          return ret;
        }
      ret = lump_send_data_frame(e, send_mode, send_buf, send_len);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

/* (DATA-loop / watchdog constants moved to the file-scope #define block
 * near the top of the file so the wdog handler can reference them.)
 */

static int lump_data_loop(struct lump_engine_s *e)
{
  /* Drain any tail bytes from sync (SYS_ACK we sent might trigger a
   * brief device echo / pause) before the steady-state loop.
   */

  lump_uart_flush_rx(e->port);

  clock_t  last_keepalive = clock_systime_ticks();
  uint8_t  miss_count     = 0;
  bool     data_rec       = false;

  /* Phase 4: arm the per-port watchdog and pet on each iteration. */

  e->wdog_stall = false;
  lump_wdog_pet(e);

  for (;;)
    {
      /* Stall detected by wdog (IRQ ctx) — exit the loop so the
       * kthread can run release/register/backoff.
       */

      if (e->wdog_stall)
        {
          syslog(LOG_WARNING,
                 "lump: port %c: watchdog stall, resetting session\n",
                 'A' + e->port);
          lump_wdog_cancel(e);
          return -ETIMEDOUT;
        }

      /* Try to read one frame within a short slice — we want the loop
       * to come around to the keepalive / TX-drain checks at least
       * every ~20 ms even if no DATA frames are arriving.
       */

      int ret = lump_recv_msg(e, LUMP_DATA_RECV_SLICE_MS);
      if (ret == OK)
        {
          if ((e->rx_msg[0] & LUMP_MSG_TYPE_MASK) == LUMP_MSG_TYPE_DATA)
            {
              data_rec = true;
            }
          (void)lump_parse_msg(e);
        }
      /* -EBADMSG / -EILSEQ / -ETIMEDOUT all fall through to keepalive. */

      /* Keepalive cadence. */

      clock_t now = clock_systime_ticks();
      if ((sclock_t)(now - last_keepalive) >=
          (sclock_t)MSEC2TICK(LUMP_KEEPALIVE_MS))
        {
          if (data_rec)
            {
              miss_count = 0;
            }
          else if (++miss_count > LUMP_DATA_MISS_LIMIT)
            {
              syslog(LOG_INFO,
                     "lump: port %c: no DATA for %lu ms, disconnecting\n",
                     'A' + e->port,
                     (unsigned long)(LUMP_DATA_MISS_LIMIT *
                                     LUMP_KEEPALIVE_MS));
              nxmutex_lock(&e->info_lock);
              e->info.flags &= (uint8_t)~LUMP_FLAG_DATA_OK;
              nxmutex_unlock(&e->info_lock);
              lump_wdog_cancel(e);
              return -ETIMEDOUT;
            }

          int wret = lump_send_sys(e, LUMP_SYS_NACK);
          if (wret < 0)
            {
              lump_wdog_cancel(e);
              return wret;
            }
          last_keepalive = now;
          data_rec       = false;
        }

      /* Drain user-posted TX requests. */

      ret = lump_drain_tx_requests(e);
      if (ret < 0)
        {
          lump_wdog_cancel(e);
          return ret;
        }

      /* External shutdown signal? */

      if (nxsem_trywait(&e->wakeup) == OK)
        {
          /* Could be wdog stall (set flag), genuine reset, or stale
           * post — wdog flag check at loop top handles the wdog case.
           */

          if (e->wdog_stall)
            {
              continue;
            }
          lump_wdog_cancel(e);
          return -EINTR;
        }

      /* Pet the watchdog before going around again. */

      lump_wdog_pet(e);
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

  /* Phase 3: full DATA state machine — receive DATA frames, send
   * 100 ms `SYS_NACK` keepalive, drain pending TX from user-space.
   */

  return lump_data_loop(e);
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
       *    Mark the engine state as ERR so `lump status` reflects the
       *    transient post-session period instead of stale DATA.
       */

      e->state = LUMP_ST_ERR;

      lump_uart_close(e->port);
      stm32_legoport_release_uart(e->port);

      nxmutex_lock(&e->info_lock);
      bool was_synced = (e->info.flags & LUMP_FLAG_SYNCED) != 0;
      e->info.flags = 0;  /* clear SYNCED / DATA_OK */
      uint8_t err_mode = e->info.current_mode;
      nxmutex_unlock(&e->info_lock);

      /* Fire `on_error` so an attached consumer (motor / sensor driver)
       * can flush its state and publish a disconnect sentinel.  Fires on
       * every session-ending transition into ERR — sync failure, missed
       * keepalives, watchdog stall — `data == NULL && len == 0` is the
       * agreed disconnect convention.  Lock-released-then-fire matches
       * `on_data` / `on_sync`; same-port API re-entry is gated by
       * `in_callback`.
       */
      {
        nxmutex_lock(&e->cb_lock);
        lump_on_data_t err_cb   = e->cb_attached ? e->cb.on_error : NULL;
        void          *err_priv = e->cb_attached ? e->cb.priv     : NULL;
        if (err_cb != NULL)
          {
            e->in_callback = true;
          }
        nxmutex_unlock(&e->cb_lock);

        if (err_cb != NULL)
          {
            err_cb(e->port, err_mode, NULL, 0, err_priv);

            nxmutex_lock(&e->cb_lock);
            e->in_callback = false;
            nxmutex_unlock(&e->cb_lock);
          }
      }

      stm32_legoport_register_uart_handoff(e->port, lump_handoff_cb, e);
      e->dcm_cb_registered = true;

      /* 4. Backoff before accepting the next handoff (capped exp). */

      uint32_t sleep_ms = g_backoff_ms[e->backoff_step];
      if (ret == OK || ret == -EINTR || was_synced)
        {
          /* Either an explicit clean exit, or a session that reached
           * SYNCED before being torn down (typically a physical
           * disconnect during normal operation — `ret = -110` from the
           * 600 ms no-DATA watchdog).  Neither is a SYNC-phase failure,
           * so reset backoff to 0 (Issue #81).  Without this, every
           * successful plug/unplug cycle would escalate the backoff
           * and a handful of cycles would bury the engine in a 30 s
           * sleep, masking subsequent device insertions.
           */

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

      /* Backoff complete — engine is now idle, waiting for the next
       * handoff sem post.  `lump_reset_session_state` will set IDLE
       * again at the top of the next iteration.
       */

      e->state = LUMP_ST_IDLE;
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

int lump_get_status_full(int port, struct lump_status_full_s *out)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT || out == NULL)
    {
      return -EINVAL;
    }

  struct lump_engine_s *e = &g_lump[port];

  memset(out, 0, sizeof(*out));

  nxmutex_lock(&e->info_lock);
  out->state        = (uint8_t)e->state;
  out->flags        = e->info.flags;
  out->type_id      = e->info.type_id;
  out->current_mode = e->info.current_mode;
  out->baud         = e->info.baud;
  nxmutex_unlock(&e->info_lock);

  out->rx_bytes     = e->rx_bytes;
  out->tx_bytes     = e->tx_bytes;
  out->backoff_step = e->backoff_step;

  nxmutex_lock(&e->dq_lock);
  out->dq_dropped   = e->dq_dropped;
  nxmutex_unlock(&e->dq_lock);

  /* Stack high-water — needs CONFIG_STACK_COLORATION (set in defconfig).
   * `up_check_tcbstack(tcb, full_size)` walks from the stack base up,
   * skipping the still-poisoned bottom and returning the deepest the
   * stack ever grew (bytes used).  Passing 0 returns 0, so we have to
   * give it the full allocation size.
   */

#ifdef CONFIG_STACK_COLORATION
  if (e->kthread > 0)
    {
      FAR struct tcb_s *tcb = nxsched_get_tcb(e->kthread);
      if (tcb != NULL)
        {
          out->stk_size = (uint32_t)tcb->adj_stack_size;
          out->stk_used = (uint32_t)up_check_tcbstack(tcb,
                                                     tcb->adj_stack_size);
        }
    }
#endif

  return OK;
}

int lump_attach(int port, const struct lump_callbacks_s *cb)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT || cb == NULL)
    {
      return -EINVAL;
    }

  struct lump_engine_s *e = &g_lump[port];

  /* Reject same-port re-entry from inside an existing callback —
   * keeps the on_data dispatch path single-threaded.
   */

  nxmutex_lock(&e->cb_lock);
  if (e->in_callback)
    {
      nxmutex_unlock(&e->cb_lock);
      return -EDEADLK;
    }
  if (e->cb_attached)
    {
      nxmutex_unlock(&e->cb_lock);
      return -EBUSY;
    }
  e->cb           = *cb;
  e->cb_attached  = true;
  nxmutex_unlock(&e->cb_lock);

  /* If the engine is already SYNCED, fire `on_sync` synchronously after
   * dropping the cb_lock — same-port API re-entry is forbidden but
   * cross-port and `lump_get_*` are fine.
   */

  if (cb->on_sync != NULL)
    {
      struct lump_device_info_s snap;
      nxmutex_lock(&e->info_lock);
      bool synced = (e->info.flags & LUMP_FLAG_SYNCED) != 0;
      if (synced)
        {
          memcpy(&snap, &e->info, sizeof(snap));
        }
      nxmutex_unlock(&e->info_lock);

      if (synced)
        {
          nxmutex_lock(&e->cb_lock);
          e->in_callback = true;
          nxmutex_unlock(&e->cb_lock);

          cb->on_sync(port, &snap, cb->priv);

          nxmutex_lock(&e->cb_lock);
          e->in_callback = false;
          nxmutex_unlock(&e->cb_lock);
        }
    }

  return OK;
}

int lump_detach(int port)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct lump_engine_s *e = &g_lump[port];

  nxmutex_lock(&e->cb_lock);
  if (e->in_callback)
    {
      nxmutex_unlock(&e->cb_lock);
      return -EDEADLK;
    }
  e->cb_attached  = false;
  memset(&e->cb, 0, sizeof(e->cb));
  nxmutex_unlock(&e->cb_lock);

  return OK;
}

int lump_select_mode(int port, uint8_t mode)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
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
  if (mode >= e->info.num_modes)
    {
      nxmutex_unlock(&e->info_lock);
      return -EINVAL;
    }
  nxmutex_unlock(&e->info_lock);

  nxmutex_lock(&e->tx_lock);
  e->pending_select       = true;
  e->pending_select_mode  = mode;
  nxmutex_unlock(&e->tx_lock);

  return OK;
}

int lump_send_data(int port, uint8_t mode, const uint8_t *buf, size_t len)
{
  bool need_select;

  if (port < 0 || port >= BOARD_LEGOPORT_COUNT ||
      buf == NULL || len == 0 || len > LUMP_MAX_PAYLOAD)
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
  if (mode >= e->info.num_modes)
    {
      nxmutex_unlock(&e->info_lock);
      return -EINVAL;
    }
  if (!e->info.modes[mode].writable)
    {
      nxmutex_unlock(&e->info_lock);
      return -ENOTSUP;
    }
  need_select = (e->info.current_mode != mode);
  nxmutex_unlock(&e->info_lock);

  /* Queue SELECT first when not already on the target mode, then DATA.
   * `lump_drain_tx_requests` honours the order (SELECT before DATA in
   * the same drain pass), mirroring pybricks' send-thread behaviour
   * (see `legodev_pup_uart.c:923` send_thread).  Writable-mode DATA
   * itself bypasses the active SELECT on the sensor side, but pre-
   * SELECTing keeps the engine's `current_mode` aligned with the mode
   * just written — useful for callers that immediately follow up with
   * `LEGOPORT_LUMP_POLL_DATA` or `lump_get_info` snapshots.
   */

  nxmutex_lock(&e->tx_lock);
  if (need_select)
    {
      e->pending_select       = true;
      e->pending_select_mode  = mode;
    }
  e->pending_send       = true;
  e->pending_send_mode  = mode;
  e->pending_send_len   = (uint8_t)len;
  memcpy(e->pending_send_buf, buf, len);
  nxmutex_unlock(&e->tx_lock);

  return OK;
}

/* Pop one DATA frame from the per-port ring, used by the
 * `LEGOPORT_LUMP_POLL_DATA` ioctl.  Returns -EAGAIN if empty.
 */

int lump_pop_data_frame(int port, struct lump_data_frame_s *out)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT || out == NULL)
    {
      return -EINVAL;
    }

  struct lump_engine_s *e = &g_lump[port];

  nxmutex_lock(&e->dq_lock);
  if (e->dq_count == 0)
    {
      nxmutex_unlock(&e->dq_lock);
      return -EAGAIN;
    }
  *out       = e->dq[e->dq_tail];
  e->dq_tail = (uint8_t)((e->dq_tail + 1) % LUMP_DATA_QUEUE);
  e->dq_count--;
  nxmutex_unlock(&e->dq_lock);

  return OK;
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
      nxmutex_init(&e->tx_lock);
      nxmutex_init(&e->dq_lock);
      nxmutex_init(&e->cb_lock);

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
