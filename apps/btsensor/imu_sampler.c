/****************************************************************************
 * apps/btsensor/imu_sampler.c
 *
 * IMU sampler / RFCOMM streamer.
 *
 * The LSM6DSL uORB driver publishes a single struct sensor_imu on
 * /dev/uorb/sensor_imu0 carrying paired accel + gyro raw int16 LSB
 * values plus an ISR-captured timestamp.  We open the topic non-blocking
 * and register its fd with the btstack run loop as a READ-flagged data
 * source, copy each sample's raw values straight into the wire-format
 * sample slot (no physical-unit conversion), and pack a Kconfig-tunable
 * batch into one RFCOMM frame.  Frames land in a small SPSC ring drained
 * by the RFCOMM can-send-now hand-shake so we never block the run loop
 * thread inside rfcomm_send.
 *
 * Wire format (all little-endian) — unchanged in Commit A; the header
 * gets reshaped (magic / FSR fields / per-sample ts_delta) in Commit E:
 *
 *   struct spp_frame_hdr {
 *     uint16_t magic;          // 0xA55A
 *     uint16_t seq;            // monotonic per frame
 *     uint32_t timestamp_us;   // first sample's hardware timestamp,
 *                              // microseconds since session start
 *                              // (mod 2^32, ~71m35s wrap)
 *     uint16_t sample_rate;    // informational (833 Hz today)
 *     uint8_t  sample_count;   // <= BTSENSOR_FRAME_MAX_SAMPLES (80)
 *     uint8_t  type;           // 0x01 = IMU
 *   };                         // 12 bytes
 *
 *   struct imu_sample {
 *     int16_t ax, ay, az;      // raw LSM6DSL accel LSB, chip frame
 *     int16_t gx, gy, gz;      // raw LSM6DSL gyro  LSB, chip frame
 *   };                         // 12 bytes each
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board_lsm6dsl.h>

#include "btstack_event.h"
#include "btstack_run_loop.h"
#include "classic/rfcomm.h"

#include "imu_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMU_DEVPATH            "/dev/uorb/sensor_imu0"

#ifndef CONFIG_APP_BTSENSOR_BATCH
#  define CONFIG_APP_BTSENSOR_BATCH       8
#endif
#ifndef CONFIG_APP_BTSENSOR_RING_DEPTH
#  define CONFIG_APP_BTSENSOR_RING_DEPTH  8
#endif

/* Compile-time upper bound for the per-frame sample buffer.  The runtime
 * batch size (g_batch) is set from CONFIG_APP_BTSENSOR_BATCH by default
 * and can be overridden with imu_sampler_configure() before init —
 * letting the btsensor entry point accept a "batch" argv positional so
 * different values can be tested without rebuilding the firmware.  Must
 * match the Kconfig range upper bound.
 */

#define BTSENSOR_FRAME_MAX_SAMPLES        80
#define BTSENSOR_RING_DEPTH               CONFIG_APP_BTSENSOR_RING_DEPTH
#define BTSENSOR_FRAME_MAX_SIZE           \
    (12 + 12 * BTSENSOR_FRAME_MAX_SAMPLES)

#define IMU_SAMPLE_RATE_HZ            833

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imu_frame_s
{
  uint8_t  buf[BTSENSOR_FRAME_MAX_SIZE];
  uint16_t len;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static btstack_data_source_t g_imu_ds;

static int g_imu_fd = -1;

/* Batch in progress (assembled sample-by-sample). */

static struct imu_frame_s g_current;
static uint8_t            g_current_count;

/* Hardware timestamp of the first sample in g_current; copied into the
 * frame header at flush time.
 */

static uint32_t           g_current_first_ts;

/* SPSC ring drained by the RFCOMM send path. */

static struct imu_frame_s g_ring[BTSENSOR_RING_DEPTH];
static uint8_t            g_ring_head;
static uint8_t            g_ring_tail;

/* Counters — visible via btsensor_status in a future step. */

static uint32_t g_frames_sent;
static uint32_t g_frames_dropped;

/* Runtime-tunable batch size (samples per RFCOMM frame).  Default
 * comes from Kconfig; overridden by imu_sampler_configure().
 */

static uint8_t  g_batch = CONFIG_APP_BTSENSOR_BATCH;

/* RFCOMM state. */

static uint16_t g_rfcomm_cid;
static uint16_t g_rfcomm_mtu;
static uint16_t g_seq;
static bool     g_can_send_pending;

/* Session-start time in the same time base as the driver's sensor_imu
 * timestamp (low 32 bits of CLOCK_BOOTTIME us).  Per-frame ts_us is
 * computed as `first_sample.timestamp - g_start_us` mod 2^32 so it
 * reflects the actual hardware sample time relative to the session
 * (Issue #55).
 */

static uint32_t g_start_us;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void encode_sample(uint8_t *dst, const struct sensor_imu *s)
{
  /* Raw int16 LSB values — driver already publishes them this way. */

  dst[0]  = (uint8_t)(s->ax & 0xff); dst[1]  = (uint8_t)(s->ax >> 8);
  dst[2]  = (uint8_t)(s->ay & 0xff); dst[3]  = (uint8_t)(s->ay >> 8);
  dst[4]  = (uint8_t)(s->az & 0xff); dst[5]  = (uint8_t)(s->az >> 8);
  dst[6]  = (uint8_t)(s->gx & 0xff); dst[7]  = (uint8_t)(s->gx >> 8);
  dst[8]  = (uint8_t)(s->gy & 0xff); dst[9]  = (uint8_t)(s->gy >> 8);
  dst[10] = (uint8_t)(s->gz & 0xff); dst[11] = (uint8_t)(s->gz >> 8);
}

static void request_send_if_needed(void)
{
  if (g_rfcomm_cid == 0 || g_can_send_pending)
    {
      return;
    }

  if (g_ring_head == g_ring_tail)
    {
      return;
    }

  /* Set pending BEFORE the request call.  rfcomm_request_can_send_now_
   * event() reaches l2cap_notify_channel_can_send() synchronously, which
   * may fire RFCOMM_EVENT_CAN_SEND_NOW and recurse back through
   * imu_sampler_on_can_send_now() in the same call stack.  If the
   * recursive on_can_send_now drains the ring and bails out at its
   * "ring empty" early-return, no further request_can_send_now is
   * issued — and if we set g_can_send_pending=true *after* the call
   * returns, we leave a phantom pending: g_can_send_pending=true while
   * btstack's waiting_for_can_send_now is back to 0.  No further
   * RFCOMM_EVENT_CAN_SEND_NOW will arrive, and every subsequent
   * request_send_if_needed() bails out at the pending check.  Setting
   * pending first lets the recursive on_can_send_now clear it
   * correctly.
   */

  g_can_send_pending = true;
  rfcomm_request_can_send_now_event(g_rfcomm_cid);
}

static void flush_current_frame(void)
{
  if (g_current_count == 0)
    {
      return;
    }

  /* Patch header fields decided at flush time. */

  uint32_t ts = g_current_first_ts - g_start_us; /* mod 2^32 */

  g_current.buf[4]  = (uint8_t)(ts & 0xff);
  g_current.buf[5]  = (uint8_t)((ts >> 8) & 0xff);
  g_current.buf[6]  = (uint8_t)((ts >> 16) & 0xff);
  g_current.buf[7]  = (uint8_t)((ts >> 24) & 0xff);
  g_current.buf[10] = g_current_count;    /* sample_count */
  g_current.len     = 12 + 12 * g_current_count;

  uint8_t next = (g_ring_head + 1) % BTSENSOR_RING_DEPTH;
  if (next == g_ring_tail)
    {
      /* Drop the oldest frame to make room — prefer newer telemetry. */

      g_ring_tail = (g_ring_tail + 1) % BTSENSOR_RING_DEPTH;
      g_frames_dropped++;
    }

  g_ring[g_ring_head] = g_current;
  g_ring_head = next;
  g_current_count = 0;

  request_send_if_needed();
}

static void append_sample(const struct sensor_imu *sample)
{
  if (g_current_count == 0)
    {
      /* First sample: lay down the frame header skeleton.  ts_us and
       * sample_count are patched at flush time by flush_current_frame().
       */

      uint16_t magic     = BTSENSOR_FRAME_MAGIC;
      uint16_t seq       = g_seq++;
      uint16_t rate      = IMU_SAMPLE_RATE_HZ;
      uint8_t  type      = BTSENSOR_FRAME_TYPE_IMU;
      uint8_t  reserved  = 0;

      g_current_first_ts = sample->timestamp;

      g_current.buf[0]  = (uint8_t)(magic & 0xff);
      g_current.buf[1]  = (uint8_t)(magic >> 8);
      g_current.buf[2]  = (uint8_t)(seq & 0xff);
      g_current.buf[3]  = (uint8_t)(seq >> 8);
      g_current.buf[4]  = 0;             /* ts_us — patched at flush */
      g_current.buf[5]  = 0;
      g_current.buf[6]  = 0;
      g_current.buf[7]  = 0;
      g_current.buf[8]  = (uint8_t)(rate & 0xff);
      g_current.buf[9]  = (uint8_t)(rate >> 8);
      g_current.buf[10] = 0;             /* sample_count — patched at flush */
      g_current.buf[11] = (type & 0x7f) | ((reserved & 1) << 7);
    }

  uint8_t *slot = &g_current.buf[12 + 12 * g_current_count];
  encode_sample(slot, sample);
  g_current_count++;

  if (g_current_count >= g_batch)
    {
      flush_current_frame();
    }
}

static void imu_process(btstack_data_source_t *ds,
                        btstack_data_source_callback_type_t type)
{
  (void)type;

  while (1)
    {
      struct sensor_imu s;
      ssize_t n = read(ds->source.fd, &s, sizeof(s));
      if (n != sizeof(s))
        {
          break;
        }

      append_sample(&s);
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imu_sampler_configure(uint8_t batch)
{
  if (batch == 0)
    {
      batch = 1;
    }

  if (batch > BTSENSOR_FRAME_MAX_SAMPLES)
    {
      batch = BTSENSOR_FRAME_MAX_SAMPLES;
    }

  g_batch = batch;
}

int imu_sampler_init(void)
{
  /* CLOCK_BOOTTIME matches the driver's ts_irq (low 32 bits of
   * sensor_get_timestamp() = clock_systime_timespec()), so the modular
   * subtraction `sample.timestamp - g_start_us` yields microseconds
   * since session start.
   */

  struct timespec start;
  clock_gettime(CLOCK_BOOTTIME, &start);
  uint64_t start_us = (uint64_t)start.tv_sec * 1000000ULL +
                      (uint64_t)start.tv_nsec / 1000ULL;
  g_start_us = (uint32_t)start_us;

  g_imu_fd = open(IMU_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (g_imu_fd < 0)
    {
      printf("btsensor: open %s errno %d\n", IMU_DEVPATH, errno);
      return -1;
    }

  btstack_run_loop_set_data_source_fd(&g_imu_ds, g_imu_fd);
  btstack_run_loop_set_data_source_handler(&g_imu_ds, imu_process);
  btstack_run_loop_add_data_source(&g_imu_ds);
  btstack_run_loop_enable_data_source_callbacks(
      &g_imu_ds, DATA_SOURCE_CALLBACK_READ);

  return 0;
}

void imu_sampler_set_rfcomm_cid(uint16_t rfcomm_cid, uint16_t mtu)
{
  g_rfcomm_cid       = rfcomm_cid;
  g_rfcomm_mtu       = mtu;
  g_can_send_pending = false;

  if (rfcomm_cid == 0)
    {
      /* Channel closed: reset batch so the next session starts clean. */

      g_current_count = 0;
      g_ring_head     = g_ring_tail;
      g_seq           = 0;
    }
  else
    {
      /* Channel opened: kick the pipeline if we already have frames. */

      request_send_if_needed();
    }
}

void imu_sampler_on_can_send_now(void)
{
  g_can_send_pending = false;

  if (g_rfcomm_cid == 0 || g_ring_head == g_ring_tail)
    {
      return;
    }

  struct imu_frame_s *fb = &g_ring[g_ring_tail];
  uint8_t ret = rfcomm_send(g_rfcomm_cid, fb->buf, fb->len);
  if (ret == 0)
    {
      g_ring_tail = (g_ring_tail + 1) % BTSENSOR_RING_DEPTH;
      g_frames_sent++;
    }

  /* Request the next can-send-now if there's more in the ring. */

  request_send_if_needed();
}
