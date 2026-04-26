/****************************************************************************
 * apps/btsensor/btsensor_tx.c
 *
 * Single RFCOMM send arbiter (Issue #56 Commit B).
 *
 * The previous design had imu_sampler.c own its own ringbuf and
 * RFCOMM_EVENT_CAN_SEND_NOW handler.  Once Commit D adds PC -> Hub
 * commands, replies need to share the send path with the telemetry
 * stream — and the BTstack RFCOMM API only allows one outstanding
 * can-send-now request per channel, so the two paths must be unified.
 *
 * Two queues feed the arbiter:
 *   - Response queue: small, FIFO, drop-prohibited.  Used by the
 *     Commit D command parser to send `OK`/`ERR <reason>` replies.
 *     Filled by btsensor_tx_enqueue_response().
 *   - Frame ring:     larger, FIFO with drop-oldest under back-pressure.
 *     Used by imu_sampler to push IMU telemetry.
 *     Filled by btsensor_tx_try_enqueue_frame().
 *
 * btsensor_tx_on_can_send_now() drains a response first if any are
 * pending, otherwise a frame, then re-arms the can-send-now request if
 * either queue is non-empty.
 *
 * All public functions must run on the BTstack main thread.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <syslog.h>

#include "classic/rfcomm.h"

#include "btsensor_tx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_APP_BTSENSOR_RING_DEPTH
#  define CONFIG_APP_BTSENSOR_RING_DEPTH    8
#endif

#define BTSENSOR_TX_RING_DEPTH              CONFIG_APP_BTSENSOR_RING_DEPTH

/* Response queue depth.  4 is enough to absorb back-to-back command
 * lines while RFCOMM is mid-send; commands are infrequent and short.
 */

#define BTSENSOR_TX_RESPONSE_DEPTH          4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct tx_frame_s
{
  uint8_t  buf[BTSENSOR_TX_FRAME_MAX_SIZE];
  uint16_t len;
};

struct tx_response_s
{
  char     buf[BTSENSOR_TX_RESPONSE_MAX_LEN];
  uint16_t len;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct tx_frame_s    g_ring[BTSENSOR_TX_RING_DEPTH];
static uint8_t              g_ring_head;
static uint8_t              g_ring_tail;

static struct tx_response_s g_resp_ring[BTSENSOR_TX_RESPONSE_DEPTH];
static uint8_t              g_resp_head;
static uint8_t              g_resp_tail;

static uint16_t g_rfcomm_cid;
static bool     g_can_send_pending;

static uint32_t g_frames_sent;
static uint32_t g_frames_dropped;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool ring_empty(void)
{
  return g_ring_head == g_ring_tail;
}

static inline bool ring_full(uint8_t next_head)
{
  return next_head == g_ring_tail;
}

static inline bool resp_empty(void)
{
  return g_resp_head == g_resp_tail;
}

static inline bool resp_full(uint8_t next_head)
{
  return next_head == g_resp_tail;
}

static void request_send_if_needed(void)
{
  if (g_rfcomm_cid == 0 || g_can_send_pending)
    {
      return;
    }

  if (ring_empty() && resp_empty())
    {
      return;
    }

  /* Set pending BEFORE the request call.  rfcomm_request_can_send_now_
   * event() reaches l2cap_notify_channel_can_send() synchronously, which
   * may fire RFCOMM_EVENT_CAN_SEND_NOW and recurse back through
   * btsensor_tx_on_can_send_now() in the same call stack.  Setting the
   * flag first lets the recursive handler clear it correctly so we do
   * not leave a phantom pending state (Issue #54 root cause for the
   * earlier sampler-only path).
   */

  g_can_send_pending = true;
  rfcomm_request_can_send_now_event(g_rfcomm_cid);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btsensor_tx_init(void)
{
  memset(g_ring, 0, sizeof(g_ring));
  memset(g_resp_ring, 0, sizeof(g_resp_ring));
  g_ring_head        = 0;
  g_ring_tail        = 0;
  g_resp_head        = 0;
  g_resp_tail        = 0;
  g_rfcomm_cid       = 0;
  g_can_send_pending = false;
  g_frames_sent      = 0;
  g_frames_dropped   = 0;
  return 0;
}

void btsensor_tx_deinit(void)
{
  /* Best-effort tear-down: drop everything still queued so the next
   * init() starts clean.  No need to send anything — the RFCOMM
   * channel is being torn down by the caller.
   */

  g_ring_head        = 0;
  g_ring_tail        = 0;
  g_resp_head        = 0;
  g_resp_tail        = 0;
  g_rfcomm_cid       = 0;
  g_can_send_pending = false;
}

void btsensor_tx_set_rfcomm_cid(uint16_t cid)
{
  g_rfcomm_cid       = cid;
  g_can_send_pending = false;

  if (cid == 0)
    {
      /* Channel closed: drop pending telemetry so the next session does
       * not start with a backlog of stale frames.  Responses are also
       * dropped (the requester is gone).
       */

      g_ring_head = g_ring_tail;
      g_resp_head = g_resp_tail;
    }
  else
    {
      request_send_if_needed();
    }
}

int btsensor_tx_enqueue_response(const char *line)
{
  size_t len = strnlen(line, BTSENSOR_TX_RESPONSE_MAX_LEN + 1);
  if (len > BTSENSOR_TX_RESPONSE_MAX_LEN)
    {
      return -E2BIG;
    }

  uint8_t next = (g_resp_head + 1) % BTSENSOR_TX_RESPONSE_DEPTH;
  if (resp_full(next))
    {
      return -ENOSPC;
    }

  memcpy(g_resp_ring[g_resp_head].buf, line, len);
  g_resp_ring[g_resp_head].len = (uint16_t)len;
  g_resp_head = next;

  request_send_if_needed();
  return 0;
}

int btsensor_tx_try_enqueue_frame(const uint8_t *buf, size_t len)
{
  if (len == 0 || len > BTSENSOR_TX_FRAME_MAX_SIZE)
    {
      return -E2BIG;
    }

  uint8_t next = (g_ring_head + 1) % BTSENSOR_TX_RING_DEPTH;
  int rc = 0;
  if (ring_full(next))
    {
      /* Drop oldest to favour newer telemetry. */

      g_ring_tail = (g_ring_tail + 1) % BTSENSOR_TX_RING_DEPTH;
      g_frames_dropped++;
      rc = -ENOSPC;
    }

  memcpy(g_ring[g_ring_head].buf, buf, len);
  g_ring[g_ring_head].len = (uint16_t)len;
  g_ring_head = next;

  request_send_if_needed();
  return rc;
}

void btsensor_tx_on_can_send_now(void)
{
  g_can_send_pending = false;

  if (g_rfcomm_cid == 0)
    {
      return;
    }

  /* Responses get priority — they must always make it through. */

  if (!resp_empty())
    {
      struct tx_response_s *r = &g_resp_ring[g_resp_tail];
      uint8_t err = rfcomm_send(g_rfcomm_cid, (uint8_t *)r->buf, r->len);
      if (err == 0)
        {
          g_resp_tail = (g_resp_tail + 1) % BTSENSOR_TX_RESPONSE_DEPTH;
        }
    }
  else if (!ring_empty())
    {
      struct tx_frame_s *f = &g_ring[g_ring_tail];
      uint8_t err = rfcomm_send(g_rfcomm_cid, f->buf, f->len);
      if (err == 0)
        {
          g_ring_tail = (g_ring_tail + 1) % BTSENSOR_TX_RING_DEPTH;
          g_frames_sent++;
        }
    }

  request_send_if_needed();
}

void btsensor_tx_get_stats(uint32_t *frames_sent, uint32_t *frames_dropped)
{
  if (frames_sent != NULL)
    {
      *frames_sent = g_frames_sent;
    }

  if (frames_dropped != NULL)
    {
      *frames_dropped = g_frames_dropped;
    }
}
