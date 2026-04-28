/****************************************************************************
 * apps/btsensor/imu_sampler.c
 *
 * IMU sampler — frame producer for btsensor.
 *
 * The LSM6DSL uORB driver (Commit A) publishes a single struct
 * sensor_imu on /dev/uorb/sensor_imu0 carrying paired accel + gyro raw
 * int16 LSB values plus an ISR-captured timestamp.  We open the topic
 * non-blocking, register its fd as a btstack data source, copy each
 * sample's raw values straight into a wire-format slot (no physical-
 * unit conversion), and hand each completed batch to btsensor_tx for
 * arbitrated send.
 *
 * Wire format (all little-endian) — Commit E layout (incompatible
 * with the Commit A/B 0xA55A layout):
 *
 *   struct spp_frame_hdr {                  // 18 bytes
 *     uint16_t magic;          // 0xB66B
 *     uint8_t  type;           // 0x01 = IMU
 *     uint8_t  sample_count;   // 1..BTSENSOR_FRAME_MAX_SAMPLES (80)
 *     uint16_t sample_rate_hz; // current ODR
 *     uint16_t accel_fsr_g;    // 2 / 4 / 8 / 16
 *     uint16_t gyro_fsr_dps;   // 125 / 250 / 500 / 1000 / 2000
 *     uint16_t seq;            // monotonic
 *     uint32_t first_sample_ts_us;  // low 32 bits of CLOCK_BOOTTIME us
 *                              // (mod 2^32, ~71m35s wrap — PC unwrap)
 *     uint16_t frame_len;      // = 18 + sample_count * 16
 *   };
 *
 *   struct imu_sample {                     // 16 bytes
 *     int16_t  ax, ay, az;     // accel raw LSB, Hub body frame
 *     int16_t  gx, gy, gz;     // gyro  raw LSB, Hub body frame
 *     uint32_t ts_delta_us;    // sample.ts - first_sample_ts_us
 *                              // (sample[0] = 0)
 *   };
 *
 * The header carries ODR + FSR so the PC can convert raw -> physical
 * units without an out-of-band probe; ts_delta_us per sample lets the
 * receiver reconstruct exact ISR timing for jitter analysis.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/sensors/ioctl.h>

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

/* BTSENSOR_HDR_SIZE / BTSENSOR_SAMPLE_SIZE come from the public
 * imu_sampler.h so any in-tree consumer sees the same constants.
 * Frame max = 18 + 80 * 16 = 1298 bytes (fits BTSENSOR_TX_FRAME_MAX_SIZE).
 */

#define BTSENSOR_FRAME_MAX_SAMPLES        80
#define BTSENSOR_FRAME_MAX_SIZE \
    (BTSENSOR_HDR_SIZE + BTSENSOR_SAMPLE_SIZE * BTSENSOR_FRAME_MAX_SAMPLES)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static btstack_data_source_t g_imu_ds;
static int                   g_imu_fd = -1;
static bool                  g_initialized;
static bool                  g_enabled;

/* Cached configuration.  Mirrors the LSM6DSL driver's startup state
 * (Commit A) so the first PC-side `SET *` while disabled lines up with
 * the actual hardware.  These are kept in sync with the driver via
 * ioctl in imu_sampler_set_*; on ioctl failure we roll back the cache.
 */

static uint32_t g_odr_hz       = 833;
static uint32_t g_accel_fsr_g  = 8;
static uint32_t g_gyro_fsr_dps = 2000;

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

static inline void put_u16le(uint8_t *dst, uint16_t v)
{
  dst[0] = (uint8_t)(v & 0xff);
  dst[1] = (uint8_t)((v >> 8) & 0xff);
}

static inline void put_u32le(uint8_t *dst, uint32_t v)
{
  dst[0] = (uint8_t)(v & 0xff);
  dst[1] = (uint8_t)((v >> 8) & 0xff);
  dst[2] = (uint8_t)((v >> 16) & 0xff);
  dst[3] = (uint8_t)((v >> 24) & 0xff);
}

static void encode_sample(uint8_t *dst, const struct sensor_imu *s,
                          uint32_t ts_delta_us)
{
  put_u16le(dst + 0,  (uint16_t)s->ax);
  put_u16le(dst + 2,  (uint16_t)s->ay);
  put_u16le(dst + 4,  (uint16_t)s->az);
  put_u16le(dst + 6,  (uint16_t)s->gx);
  put_u16le(dst + 8,  (uint16_t)s->gy);
  put_u16le(dst + 10, (uint16_t)s->gz);
  put_u32le(dst + 12, ts_delta_us);
}

static void flush_current_frame(void)
{
  if (g_current_count == 0)
    {
      return;
    }

  /* Patch the header fields that depend on flush-time state. */

  uint16_t frame_len     = BTSENSOR_HDR_SIZE +
                           BTSENSOR_SAMPLE_SIZE * g_current_count;
  uint32_t first_ts_off  = g_current_first_ts - g_start_us; /* mod 2^32 */

  g_current[3] = g_current_count;             /* sample_count */
  put_u32le(&g_current[12], first_ts_off);    /* first_sample_ts_us */
  put_u16le(&g_current[16], frame_len);       /* frame_len */

  (void)btsensor_tx_try_enqueue_frame(g_current, frame_len);
  g_current_count = 0;
}

static void append_sample(const struct sensor_imu *sample)
{
  if (g_current_count == 0)
    {
      g_current_first_ts = sample->timestamp;

      /* Lay down the static portion of the header.  Fields patched at
       * flush time (sample_count, first_sample_ts_us, frame_len) are
       * left zeroed here for clarity.
       */

      put_u16le(&g_current[0],  BTSENSOR_FRAME_MAGIC);  /* magic   */
      g_current[2]  = BTSENSOR_FRAME_TYPE_IMU;          /* type    */
      g_current[3]  = 0;                                /* count   */
      put_u16le(&g_current[4],  (uint16_t)g_odr_hz);    /* rate    */
      put_u16le(&g_current[6],  (uint16_t)g_accel_fsr_g);
      put_u16le(&g_current[8],  (uint16_t)g_gyro_fsr_dps);
      put_u16le(&g_current[10], g_seq++);               /* seq     */
      put_u32le(&g_current[12], 0);                     /* first ts */
      put_u16le(&g_current[16], 0);                     /* frame_len */
    }

  /* Per-sample slot, including 4-byte ts_delta_us at offset 12. */

  uint32_t ts_delta = sample->timestamp - g_current_first_ts;
  uint8_t *slot = &g_current[BTSENSOR_HDR_SIZE +
                             BTSENSOR_SAMPLE_SIZE * g_current_count];
  encode_sample(slot, sample, ts_delta);
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

  /* Skip the encode + memcpy work entirely when no RFCOMM consumer is
   * bound — the frames would just be dropped by btsensor_tx anyway,
   * and uncosumed sensor reads dominate the daemon's CPU footprint
   * while a PC isn't connected.  We still drain the kernel buffer so
   * it doesn't grow unbounded.
   */

  bool has_consumer = btsensor_tx_has_consumer();

  while (1)
    {
      struct sensor_imu s;
      ssize_t n = read(ds->source.fd, &s, sizeof(s));
      if (n != sizeof(s))
        {
          break;
        }

      if (has_consumer)
        {
          append_sample(&s);
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imu_sampler_init(void)
{
  if (g_initialized)
    {
      return 0;
    }

  /* Module bring-up only: do not open the driver fd yet — sampling
   * starts when the PC sends `IMU ON` (or the daemon explicitly calls
   * imu_sampler_set_enabled(true)).
   */

  struct timespec start;
  clock_gettime(CLOCK_BOOTTIME, &start);
  uint64_t start_us = (uint64_t)start.tv_sec * 1000000ULL +
                      (uint64_t)start.tv_nsec / 1000ULL;
  g_start_us       = (uint32_t)start_us;
  g_seq            = 0;
  g_current_count  = 0;
  g_enabled        = false;
  g_initialized    = true;
  return 0;
}

void imu_sampler_deinit(void)
{
  if (!g_initialized)
    {
      return;
    }

  imu_sampler_set_enabled(false);
  g_initialized = false;
}

bool imu_sampler_is_enabled(void)
{
  return g_enabled;
}

int imu_sampler_set_enabled(bool on)
{
  if (!g_initialized)
    {
      return -EINVAL;
    }

  if (on == g_enabled)
    {
      return 0;
    }

  if (on)
    {
      g_imu_fd = open(IMU_DEVPATH, O_RDONLY | O_NONBLOCK);
      if (g_imu_fd < 0)
        {
          syslog(LOG_ERR, "btsensor: open %s errno %d\n",
                 IMU_DEVPATH, errno);
          return -errno;
        }

      btstack_run_loop_set_data_source_fd(&g_imu_ds, g_imu_fd);
      btstack_run_loop_set_data_source_handler(&g_imu_ds, imu_process);
      btstack_run_loop_add_data_source(&g_imu_ds);
      btstack_run_loop_enable_data_source_callbacks(
          &g_imu_ds, DATA_SOURCE_CALLBACK_READ);

      g_current_count = 0;
      g_enabled       = true;
      syslog(LOG_INFO, "btsensor: IMU sampling on (%u Hz / %ug / %udps)\n",
             (unsigned)g_odr_hz, (unsigned)g_accel_fsr_g,
             (unsigned)g_gyro_fsr_dps);
    }
  else
    {
      btstack_run_loop_disable_data_source_callbacks(
          &g_imu_ds, DATA_SOURCE_CALLBACK_READ);
      btstack_run_loop_remove_data_source(&g_imu_ds);

      if (g_imu_fd >= 0)
        {
          close(g_imu_fd);
          g_imu_fd = -1;
        }

      g_current_count = 0;
      g_enabled       = false;
      syslog(LOG_INFO, "btsensor: IMU sampling off\n");
    }

  return 0;
}

/* Open a transient O_WRONLY fd to issue a reconfiguration ioctl without
 * subscribing to the topic (sensor upper-half only auto-activates on
 * O_RDOK).  Returns the fd or a negated errno.
 */

static int imu_open_ctrl_fd(void)
{
  int fd = open(IMU_DEVPATH, O_WRONLY);
  if (fd < 0)
    {
      return -errno;
    }

  return fd;
}

int imu_sampler_set_odr_hz(uint32_t hz)
{
  if (g_enabled)
    {
      return -EBUSY;
    }

  int fd = imu_open_ctrl_fd();
  if (fd < 0)
    {
      return fd;
    }

  uint32_t prev = g_odr_hz;
  g_odr_hz = hz;
  int rc = ioctl(fd, SNIOC_SETSAMPLERATE, hz);
  close(fd);
  if (rc < 0)
    {
      g_odr_hz = prev;
      return -errno;
    }

  return 0;
}

int imu_sampler_set_batch(uint8_t n)
{
  if (g_enabled)
    {
      return -EBUSY;
    }

  if (n == 0 || n > BTSENSOR_FRAME_MAX_SAMPLES)
    {
      return -EINVAL;
    }

  g_batch = n;
  return 0;
}

int imu_sampler_set_accel_fsr(uint32_t g)
{
  if (g_enabled)
    {
      return -EBUSY;
    }

  int fd = imu_open_ctrl_fd();
  if (fd < 0)
    {
      return fd;
    }

  uint32_t prev = g_accel_fsr_g;
  g_accel_fsr_g = g;
  int rc = ioctl(fd, LSM6DSL_IOC_SETACCELFSR, g);
  close(fd);
  if (rc < 0)
    {
      g_accel_fsr_g = prev;
      return -errno;
    }

  return 0;
}

int imu_sampler_set_gyro_fsr(uint32_t dps)
{
  if (g_enabled)
    {
      return -EBUSY;
    }

  int fd = imu_open_ctrl_fd();
  if (fd < 0)
    {
      return fd;
    }

  uint32_t prev = g_gyro_fsr_dps;
  g_gyro_fsr_dps = dps;
  int rc = ioctl(fd, LSM6DSL_IOC_SETGYROFSR, dps);
  close(fd);
  if (rc < 0)
    {
      g_gyro_fsr_dps = prev;
      return -errno;
    }

  return 0;
}

uint32_t imu_sampler_get_odr_hz(void)
{
  return g_odr_hz;
}

uint32_t imu_sampler_get_accel_fsr_g(void)
{
  return g_accel_fsr_g;
}

uint32_t imu_sampler_get_gyro_fsr_dps(void)
{
  return g_gyro_fsr_dps;
}

uint8_t imu_sampler_get_batch(void)
{
  return g_batch;
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
