/****************************************************************************
 * apps/btsensor/imu_sampler.c
 *
 * IMU sampler / RFCOMM streamer (Issue #52 Step E).
 *
 * The LSM6DS3TR-C driver publishes accel + gyro through uORB at 833 Hz
 * (±8g / ±2000 dps).  We open both feeds non-blocking, register their
 * fds with the btstack run loop as READ-flagged data sources, pair
 * samples as they arrive and pack 16 paired samples per RFCOMM frame.
 * Frames land in a small SPSC ring drained by the RFCOMM can-send-now
 * hand-shake so we never block the run loop thread inside rfcomm_send.
 *
 * Wire format (all little-endian):
 *
 *   struct spp_frame_hdr {
 *     uint16_t magic;          // 0xA55A
 *     uint16_t seq;            // monotonic per frame
 *     uint32_t timestamp_us;   // first sample's hardware timestamp,
 *                              // microseconds since session start
 *     uint16_t sample_rate;    // informational (833 Hz today)
 *     uint8_t  sample_count;   // <= BTSENSOR_FRAME_MAX_SAMPLES (16)
 *     uint8_t  type;           // 0x01 = IMU
 *   };                         // 12 bytes
 *
 *   struct imu_sample {
 *     int16_t ax, ay, az;      // raw LSM6DS3 accel LSB, 0.244 mg/LSB
 *     int16_t gx, gy, gz;      // raw LSM6DS3 gyro  LSB, 0.070 dps/LSB
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

#include <nuttx/uorb.h>

#include "btstack_event.h"
#include "btstack_run_loop.h"
#include "classic/rfcomm.h"

#include "imu_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ACCEL_DEVPATH          "/dev/uorb/sensor_accel0"
#define GYRO_DEVPATH           "/dev/uorb/sensor_gyro0"

#ifndef CONFIG_APP_BTSENSOR_BATCH
#  define CONFIG_APP_BTSENSOR_BATCH       16
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

/* LSM6DS3 scales matching apps/imu/imu_main.c (chip configured to
 * ±8g / ±2000 dps).  Used to re-encode the uORB physical-unit samples
 * back into raw int16 LSBs for the wire format.
 */

#define IMU_ACCEL_SCALE_MG_LSB        0.244f
#define IMU_GYRO_SCALE_DPS_LSB        0.070f
#define IMU_GRAVITY_MS2               9.80665f
#define IMU_RAD_TO_DEG                57.29577951f

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

static btstack_data_source_t g_accel_ds;
static btstack_data_source_t g_gyro_ds;

static int g_accel_fd = -1;
static int g_gyro_fd = -1;

static struct sensor_accel g_last_accel;
static struct sensor_gyro  g_last_gyro;
static bool                g_accel_fresh;
static bool                g_gyro_fresh;

/* Batch in progress (assembled sample-by-sample). */

static struct imu_frame_s g_current;
static uint8_t            g_current_count;

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

/* Session-start time in the same time base as the uORB sensor message
 * timestamp field (clock_systime_timespec → CLOCK_BOOTTIME).  Per-frame
 * ts_us is computed as `first_sample.timestamp - g_start_us` so it
 * reflects the actual hardware sample time, not the moment the Hub
 * drained the FIFO.  See Issue #55.
 */

static uint64_t g_start_us;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int16_t clamp_i16(int32_t v)
{
  if (v > INT16_MAX)
    {
      return INT16_MAX;
    }

  if (v < INT16_MIN)
    {
      return INT16_MIN;
    }

  return (int16_t)v;
}

/* Convert uORB physical units back to the raw LSM6DS3 ADC LSB values. */

static void encode_sample(uint8_t *dst,
                          const struct sensor_accel *a,
                          const struct sensor_gyro *g)
{
  /* uORB accel is m/s^2.  Raw LSB = m/s^2 * 1000 / 9.80665 / 0.244. */

  int16_t ax = clamp_i16((int32_t)(a->x * 1000.0f /
                                   (IMU_GRAVITY_MS2 *
                                    IMU_ACCEL_SCALE_MG_LSB)));
  int16_t ay = clamp_i16((int32_t)(a->y * 1000.0f /
                                   (IMU_GRAVITY_MS2 *
                                    IMU_ACCEL_SCALE_MG_LSB)));
  int16_t az = clamp_i16((int32_t)(a->z * 1000.0f /
                                   (IMU_GRAVITY_MS2 *
                                    IMU_ACCEL_SCALE_MG_LSB)));

  /* uORB gyro is rad/s.  Raw LSB = rad/s * (180/pi) / 0.070. */

  int16_t gx = clamp_i16((int32_t)(g->x * IMU_RAD_TO_DEG /
                                   IMU_GYRO_SCALE_DPS_LSB));
  int16_t gy = clamp_i16((int32_t)(g->y * IMU_RAD_TO_DEG /
                                   IMU_GYRO_SCALE_DPS_LSB));
  int16_t gz = clamp_i16((int32_t)(g->z * IMU_RAD_TO_DEG /
                                   IMU_GYRO_SCALE_DPS_LSB));

  /* Write little-endian. */

  dst[0]  = (uint8_t)(ax & 0xff); dst[1]  = (uint8_t)(ax >> 8);
  dst[2]  = (uint8_t)(ay & 0xff); dst[3]  = (uint8_t)(ay >> 8);
  dst[4]  = (uint8_t)(az & 0xff); dst[5]  = (uint8_t)(az >> 8);
  dst[6]  = (uint8_t)(gx & 0xff); dst[7]  = (uint8_t)(gx >> 8);
  dst[8]  = (uint8_t)(gy & 0xff); dst[9]  = (uint8_t)(gy >> 8);
  dst[10] = (uint8_t)(gz & 0xff); dst[11] = (uint8_t)(gz >> 8);
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

  /* Patch sample_count into the header now that we're done. */

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

static void maybe_build_sample(void)
{
  if (!g_accel_fresh || !g_gyro_fresh)
    {
      return;
    }

  /* Start a new frame header if this is the first sample. */

  if (g_current_count == 0)
    {
      /* Use the first paired sample's hardware timestamp (uORB stamps
       * accel + gyro with the same value at the moment the driver
       * reads the sensor, see boards/spike-prime-hub/src/lsm6dsl_uorb.c).
       * The previous monotonic_us_since_start() recorded the Hub's
       * drain time, which clusters multiple frames to the same value
       * during FIFO bursts (Issue #55).
       */

      uint64_t sample_us = g_last_accel.timestamp;
      uint16_t magic     = BTSENSOR_FRAME_MAGIC;
      uint16_t seq       = g_seq++;
      uint32_t ts        = (uint32_t)(sample_us - g_start_us);
      uint16_t rate      = IMU_SAMPLE_RATE_HZ;
      uint8_t  type      = BTSENSOR_FRAME_TYPE_IMU;
      uint8_t  reserved  = 0;

      g_current.buf[0]  = (uint8_t)(magic & 0xff);
      g_current.buf[1]  = (uint8_t)(magic >> 8);
      g_current.buf[2]  = (uint8_t)(seq & 0xff);
      g_current.buf[3]  = (uint8_t)(seq >> 8);
      g_current.buf[4]  = (uint8_t)(ts & 0xff);
      g_current.buf[5]  = (uint8_t)((ts >> 8) & 0xff);
      g_current.buf[6]  = (uint8_t)((ts >> 16) & 0xff);
      g_current.buf[7]  = (uint8_t)((ts >> 24) & 0xff);
      g_current.buf[8]  = (uint8_t)(rate & 0xff);
      g_current.buf[9]  = (uint8_t)(rate >> 8);
      g_current.buf[10] = 0;             /* sample_count — patched at flush */
      g_current.buf[11] = (type & 0x7f) | ((reserved & 1) << 7);
    }

  uint8_t *slot = &g_current.buf[12 + 12 * g_current_count];
  encode_sample(slot, &g_last_accel, &g_last_gyro);
  g_current_count++;

  g_accel_fresh = false;
  g_gyro_fresh  = false;

  if (g_current_count >= g_batch)
    {
      flush_current_frame();
    }
}

static void accel_process(btstack_data_source_t *ds,
                          btstack_data_source_callback_type_t type)
{
  (void)type;

  while (1)
    {
      struct sensor_accel s;
      ssize_t n = read(ds->source.fd, &s, sizeof(s));
      if (n != sizeof(s))
        {
          break;
        }

      g_last_accel  = s;
      g_accel_fresh = true;
      maybe_build_sample();
    }
}

static void gyro_process(btstack_data_source_t *ds,
                         btstack_data_source_callback_type_t type)
{
  (void)type;

  while (1)
    {
      struct sensor_gyro s;
      ssize_t n = read(ds->source.fd, &s, sizeof(s));
      if (n != sizeof(s))
        {
          break;
        }

      g_last_gyro  = s;
      g_gyro_fresh = true;
      maybe_build_sample();
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
  /* CLOCK_BOOTTIME matches clock_systime_timespec() used by NuttX's
   * sensor_get_timestamp(), so subtracting g_start_us from a sample's
   * uORB timestamp yields microseconds since session start.
   */

  struct timespec start;
  clock_gettime(CLOCK_BOOTTIME, &start);
  g_start_us = (uint64_t)start.tv_sec * 1000000ULL +
               (uint64_t)start.tv_nsec / 1000ULL;

  g_accel_fd = open(ACCEL_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (g_accel_fd < 0)
    {
      printf("btsensor: open %s errno %d\n", ACCEL_DEVPATH, errno);
      return -1;
    }

  g_gyro_fd = open(GYRO_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (g_gyro_fd < 0)
    {
      printf("btsensor: open %s errno %d\n", GYRO_DEVPATH, errno);
      close(g_accel_fd);
      g_accel_fd = -1;
      return -1;
    }

  btstack_run_loop_set_data_source_fd(&g_accel_ds, g_accel_fd);
  btstack_run_loop_set_data_source_handler(&g_accel_ds, accel_process);
  btstack_run_loop_add_data_source(&g_accel_ds);
  btstack_run_loop_enable_data_source_callbacks(
      &g_accel_ds, DATA_SOURCE_CALLBACK_READ);

  btstack_run_loop_set_data_source_fd(&g_gyro_ds, g_gyro_fd);
  btstack_run_loop_set_data_source_handler(&g_gyro_ds, gyro_process);
  btstack_run_loop_add_data_source(&g_gyro_ds);
  btstack_run_loop_enable_data_source_callbacks(
      &g_gyro_ds, DATA_SOURCE_CALLBACK_READ);

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
