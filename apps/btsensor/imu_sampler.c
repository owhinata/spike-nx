/****************************************************************************
 * apps/btsensor/imu_sampler.c
 *
 * IMU drain helper for btsensor.
 *
 * The LSM6DSL uORB driver publishes paired raw int16 accel + gyro values
 * with an ISR-captured timestamp on /dev/uorb/sensor_imu0.  We open the
 * topic non-blocking, register its fd as a btstack data source, and
 * pull every available sample into a private ring on each read
 * callback.  bundle_emitter (Issue #88) drains the ring once per 10 ms
 * tick and folds the samples into the IMU section of a BUNDLE frame.
 *
 * Configuration ioctls (ODR, accel_fsr, gyro_fsr) stay live here.  ODR
 * is capped at 833 Hz to keep the per-tick sample count under bundle's
 * 8-sample limit.
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

#include "btstack_event.h"
#include "btstack_run_loop.h"

#include "btsensor_tx.h"
#include "imu_sampler.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMU_DEVPATH                "/dev/uorb/sensor_imu0"

/* Private ring size.  Sized for ~38 ms of 833 Hz samples so two missed
 * 10 ms ticks still don't overflow.  Power-of-two for cheap modulus.
 */

#define IMU_DRAIN_RING             32

/* IMU ODR ceiling — see imu_sampler.h::imu_sampler_set_odr_hz(). */

#define IMU_ODR_MAX_HZ             833

/****************************************************************************
 * Private Data
 ****************************************************************************/

static btstack_data_source_t g_imu_ds;
static int                   g_imu_fd = -1;
static bool                  g_initialized;
static bool                  g_enabled;

/* Issue #139: no local cache of ODR/FSR.  Drives state via the driver
 * GET ioctls and per-sample idx fields in struct sensor_imu, so any
 * client (host SET, drivebase, imu daemon) sees a single source of
 * truth and live reconfig propagates correctly.
 */

/* Drain ring (single producer = btstack data source callback,
 * single consumer = bundle_emitter tick callback; both run on the
 * BTstack run-loop thread so no lock is needed).
 */

static struct sensor_imu g_ring[IMU_DRAIN_RING];
static uint16_t          g_ring_head;
static uint16_t          g_ring_tail;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint16_t ring_next(uint16_t i)
{
  return (uint16_t)((i + 1U) % IMU_DRAIN_RING);
}

static inline bool ring_empty(void)
{
  return g_ring_head == g_ring_tail;
}

static inline bool ring_full(void)
{
  return ring_next(g_ring_head) == g_ring_tail;
}

static void ring_push(const struct sensor_imu *s)
{
  if (ring_full())
    {
      /* Overflow: drop oldest to make room.  ring is small (32) but the
       * bundle emitter is supposed to drain it every 10 ms; if it falls
       * far enough behind to overflow, oldest-wins matches the rest of
       * the daemon's drop policy.
       */

      g_ring_tail = ring_next(g_ring_tail);
    }

  g_ring[g_ring_head] = *s;
  g_ring_head = ring_next(g_ring_head);
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

      ring_push(&s);
    }
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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int imu_sampler_init(void)
{
  if (g_initialized)
    {
      return 0;
    }

  g_ring_head    = 0;
  g_ring_tail    = 0;
  g_enabled      = false;
  g_initialized  = true;
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

      g_ring_head = g_ring_tail;       /* drop any stale samples */
      g_enabled   = true;
      syslog(LOG_INFO, "btsensor: IMU sampling on\n");
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

      g_enabled = false;
      syslog(LOG_INFO, "btsensor: IMU sampling off\n");
    }

  return 0;
}

/* Issue #139: thin ioctl wrappers — no local cache.  Live SET while
 * streaming is enabled is now allowed (the driver no longer rejects
 * with -EBUSY; per-sample idx in struct sensor_imu lets consumers
 * follow the change).  The 833 Hz ODR cap stays because it is a
 * btsensor-level constraint (100 Hz tick, 8 samples / frame budget).
 */

int imu_sampler_set_odr_hz(uint32_t hz)
{
  if (hz == 0 || hz > IMU_ODR_MAX_HZ)
    {
      return -EINVAL;
    }

  int fd = imu_open_ctrl_fd();
  if (fd < 0)
    {
      return fd;
    }

  int rc = ioctl(fd, SNIOC_SETSAMPLERATE, hz);
  close(fd);
  return (rc < 0) ? -errno : 0;
}

int imu_sampler_set_accel_fsr(uint32_t g)
{
  int fd = imu_open_ctrl_fd();
  if (fd < 0)
    {
      return fd;
    }

  int rc = ioctl(fd, LSM6DSL_IOC_SETACCELFSR, g);
  close(fd);
  return (rc < 0) ? -errno : 0;
}

int imu_sampler_set_gyro_fsr(uint32_t dps)
{
  int fd = imu_open_ctrl_fd();
  if (fd < 0)
    {
      return fd;
    }

  int rc = ioctl(fd, LSM6DSL_IOC_SETGYROFSR, dps);
  close(fd);
  return (rc < 0) ? -errno : 0;
}

/* Issue #139: GET helpers — open ctrl fd, ioctl, return idx as uint32_t.
 * The idx is the driver-internal enum value (lsm6dsl_odr_e etc.);
 * callers convert to physical units via the corresponding lookup
 * table.  This matches the per-sample idx fields in struct sensor_imu.
 */

static int imu_sampler_get_idx(int cmd, uint32_t *out)
{
  if (out == NULL)
    {
      return -EINVAL;
    }

  int fd = imu_open_ctrl_fd();
  if (fd < 0)
    {
      return fd;
    }

  uint32_t value = 0;
  int rc = ioctl(fd, cmd, (unsigned long)&value);
  close(fd);
  if (rc < 0)
    {
      return -errno;
    }

  *out = value;
  return 0;
}

int imu_sampler_get_odr_idx(uint32_t *out)
{
  return imu_sampler_get_idx(SNIOC_GETSAMPLERATE, out);
}

int imu_sampler_get_accel_fsr_idx(uint32_t *out)
{
  return imu_sampler_get_idx(LSM6DSL_IOC_GETACCELFSR, out);
}

int imu_sampler_get_gyro_fsr_idx(uint32_t *out)
{
  return imu_sampler_get_idx(LSM6DSL_IOC_GETGYROFSR, out);
}

size_t imu_sampler_drain(struct sensor_imu *out, size_t max)
{
  size_t n = 0;
  while (n < max && !ring_empty())
    {
      out[n++] = g_ring[g_ring_tail];
      g_ring_tail = ring_next(g_ring_tail);
    }

  return n;
}

uint32_t imu_sampler_now_us(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_BOOTTIME, &ts);
  uint64_t us = (uint64_t)ts.tv_sec * 1000000ULL +
                (uint64_t)ts.tv_nsec / 1000ULL;
  return (uint32_t)us;
}

void imu_sampler_set_rfcomm_cid(uint16_t rfcomm_cid, uint16_t mtu)
{
  (void)mtu;

  if (rfcomm_cid == 0)
    {
      g_ring_head = g_ring_tail;       /* drop pending on disconnect */
    }

  btsensor_tx_set_rfcomm_cid(rfcomm_cid);
}
