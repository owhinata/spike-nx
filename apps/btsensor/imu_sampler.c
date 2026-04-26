/****************************************************************************
 * apps/btsensor/imu_sampler.c
 *
 * IMU sampler — frame producer for btsensor (Issue #56 Commit B).
 *
 * The LSM6DSL uORB driver (Commit A) publishes a single struct
 * sensor_imu on /dev/uorb/sensor_imu0 carrying paired accel + gyro raw
 * int16 LSB values plus an ISR-captured timestamp.  We open the topic
 * non-blocking, register its fd as a btstack data source, copy each
 * sample's raw values straight into a wire-format slot (no physical-
 * unit conversion), and hand each completed batch to btsensor_tx for
 * arbitrated send.
 *
 * Wire format (all little-endian) — unchanged in Commit A/B; the header
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
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board_lsm6dsl.h>

#include "btstack_event.h"
#include "btstack_run_loop.h"

#include "btsensor_tx.h"
#include "imu_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMU_DEVPATH                       "/dev/uorb/sensor_imu0"

#ifndef CONFIG_APP_BTSENSOR_BATCH
#  define CONFIG_APP_BTSENSOR_BATCH       8
#endif

#define BTSENSOR_FRAME_HDR_SIZE           12
#define BTSENSOR_SAMPLE_SIZE              12
#define BTSENSOR_FRAME_MAX_SAMPLES        80
#define BTSENSOR_FRAME_MAX_SIZE \
    (BTSENSOR_FRAME_HDR_SIZE + BTSENSOR_SAMPLE_SIZE * BTSENSOR_FRAME_MAX_SAMPLES)

#define IMU_SAMPLE_RATE_HZ                833

/****************************************************************************
 * Private Data
 ****************************************************************************/

static btstack_data_source_t g_imu_ds;
static int                   g_imu_fd = -1;
static bool                  g_initialized;

/* Frame in progress (assembled sample-by-sample in a stack-resident
 * scratch buffer flushed via btsensor_tx_try_enqueue_frame()).
 */

static uint8_t  g_current[BTSENSOR_FRAME_MAX_SIZE];
static uint8_t  g_current_count;
static uint32_t g_current_first_ts;

/* Runtime-tunable batch size (samples per RFCOMM frame). */

static uint8_t  g_batch = CONFIG_APP_BTSENSOR_BATCH;

/* Frame sequence counter. */

static uint16_t g_seq;

/* Session-start time in the same time base as the driver's sensor_imu
 * timestamp (low 32 bits of CLOCK_BOOTTIME us).  Per-frame ts_us is
 * `first_sample.timestamp - g_start_us` mod 2^32 (Issue #55).
 */

static uint32_t g_start_us;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void encode_sample(uint8_t *dst, const struct sensor_imu *s)
{
  dst[0]  = (uint8_t)(s->ax & 0xff); dst[1]  = (uint8_t)(s->ax >> 8);
  dst[2]  = (uint8_t)(s->ay & 0xff); dst[3]  = (uint8_t)(s->ay >> 8);
  dst[4]  = (uint8_t)(s->az & 0xff); dst[5]  = (uint8_t)(s->az >> 8);
  dst[6]  = (uint8_t)(s->gx & 0xff); dst[7]  = (uint8_t)(s->gx >> 8);
  dst[8]  = (uint8_t)(s->gy & 0xff); dst[9]  = (uint8_t)(s->gy >> 8);
  dst[10] = (uint8_t)(s->gz & 0xff); dst[11] = (uint8_t)(s->gz >> 8);
}

static void flush_current_frame(void)
{
  if (g_current_count == 0)
    {
      return;
    }

  /* Patch header fields decided at flush time. */

  uint32_t ts = g_current_first_ts - g_start_us;   /* mod 2^32 */

  g_current[4]  = (uint8_t)(ts & 0xff);
  g_current[5]  = (uint8_t)((ts >> 8) & 0xff);
  g_current[6]  = (uint8_t)((ts >> 16) & 0xff);
  g_current[7]  = (uint8_t)((ts >> 24) & 0xff);
  g_current[10] = g_current_count;

  uint16_t len = BTSENSOR_FRAME_HDR_SIZE +
                 BTSENSOR_SAMPLE_SIZE * g_current_count;

  (void)btsensor_tx_try_enqueue_frame(g_current, len);
  g_current_count = 0;
}

static void append_sample(const struct sensor_imu *sample)
{
  if (g_current_count == 0)
    {
      uint16_t magic     = BTSENSOR_FRAME_MAGIC;
      uint16_t seq       = g_seq++;
      uint16_t rate      = IMU_SAMPLE_RATE_HZ;
      uint8_t  type      = BTSENSOR_FRAME_TYPE_IMU;
      uint8_t  reserved  = 0;

      g_current_first_ts = sample->timestamp;

      g_current[0]  = (uint8_t)(magic & 0xff);
      g_current[1]  = (uint8_t)(magic >> 8);
      g_current[2]  = (uint8_t)(seq & 0xff);
      g_current[3]  = (uint8_t)(seq >> 8);
      g_current[4]  = 0;             /* ts_us — patched at flush */
      g_current[5]  = 0;
      g_current[6]  = 0;
      g_current[7]  = 0;
      g_current[8]  = (uint8_t)(rate & 0xff);
      g_current[9]  = (uint8_t)(rate >> 8);
      g_current[10] = 0;             /* sample_count — patched at flush */
      g_current[11] = (type & 0x7f) | ((reserved & 1) << 7);
    }

  uint8_t *slot = &g_current[BTSENSOR_FRAME_HDR_SIZE +
                             BTSENSOR_SAMPLE_SIZE * g_current_count];
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
  if (g_initialized)
    {
      return 0;
    }

  struct timespec start;
  clock_gettime(CLOCK_BOOTTIME, &start);
  uint64_t start_us = (uint64_t)start.tv_sec * 1000000ULL +
                      (uint64_t)start.tv_nsec / 1000ULL;
  g_start_us       = (uint32_t)start_us;
  g_seq            = 0;
  g_current_count  = 0;

  g_imu_fd = open(IMU_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (g_imu_fd < 0)
    {
      syslog(LOG_ERR, "btsensor: open %s errno %d\n", IMU_DEVPATH, errno);
      return -errno;
    }

  btstack_run_loop_set_data_source_fd(&g_imu_ds, g_imu_fd);
  btstack_run_loop_set_data_source_handler(&g_imu_ds, imu_process);
  btstack_run_loop_add_data_source(&g_imu_ds);
  btstack_run_loop_enable_data_source_callbacks(
      &g_imu_ds, DATA_SOURCE_CALLBACK_READ);

  g_initialized = true;
  return 0;
}

void imu_sampler_deinit(void)
{
  if (!g_initialized)
    {
      return;
    }

  btstack_run_loop_disable_data_source_callbacks(
      &g_imu_ds, DATA_SOURCE_CALLBACK_READ);
  btstack_run_loop_remove_data_source(&g_imu_ds);

  if (g_imu_fd >= 0)
    {
      close(g_imu_fd);
      g_imu_fd = -1;
    }

  g_current_count = 0;
  g_seq           = 0;
  g_initialized   = false;
}

void imu_sampler_set_rfcomm_cid(uint16_t rfcomm_cid, uint16_t mtu)
{
  (void)mtu;

  if (rfcomm_cid == 0)
    {
      g_current_count = 0;
      g_seq           = 0;
    }

  btsensor_tx_set_rfcomm_cid(rfcomm_cid);
}
