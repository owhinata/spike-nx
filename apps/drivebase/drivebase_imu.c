/****************************************************************************
 * apps/drivebase/drivebase_imu.c
 *
 * 1D heading integration over Z-axis gyro with running-bias estimate.
 *
 * Sample format: struct sensor_imu (NuttX standard) carrying raw
 * int16 gx/gy/gz + ax/ay/az already rotated into the SPIKE Prime
 * Hub body frame by the lsm6dsl_uorb driver.  Default sensitivity is
 * 70 mdps/LSB at FS=2000 dps; sample rate 833 Hz from the LSM6DSL
 * boot config (we do not change ODR — sharing with btsensor
 * sampler).
 *
 * Bias estimation: while |gz| stays below `bias_idle_threshold_lsb`
 * (default 28 LSB ≈ 2 dps) for `bias_window_us` (default 200 ms), we
 * accumulate the running average and update bias_z_lsb.  The
 * subtracting integrator then produces ~0 deg drift per second when
 * the chassis is still.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/board/board_lsm6dsl.h>   /* struct sensor_imu        */

#include "drivebase_imu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_IMU_DEVPATH               "/dev/uorb/sensor_imu0"
#define DB_IMU_DEFAULT_BIAS_THRESH    143u   /* ≈ 10 dps; tolerates raw  */
                                             /* gyro noise floor while   */
                                             /* still rejecting any move */
                                             /* the robot would actually */
                                             /* care about               */
#define DB_IMU_DEFAULT_BIAS_WINDOW_US 200000u

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

static void integrate(struct db_imu_s *im, int16_t gz_raw, uint64_t ts_us)
{
  /* Bias-corrected gyro Z in LSB. */

  int32_t gz_lsb = (int32_t)gz_raw - im->bias_z_lsb;

  /* Bias estimation gate. */

  if ((uint32_t)abs32(gz_lsb) < im->bias_idle_threshold_lsb)
    {
      /* Currently idle.  Accumulate raw (NOT bias-corrected) so the
       * estimate moves toward the true drift of the gyro itself.
       */

      im->bias_acc_lsb += (int32_t)gz_raw;
      im->bias_acc_count++;

      if (im->last_sample_ts_us != 0 && ts_us > im->last_sample_ts_us)
        {
          im->bias_idle_streak_us += ts_us - im->last_sample_ts_us;
        }

      if (im->bias_idle_streak_us >= im->bias_window_us &&
          im->bias_acc_count > 0)
        {
          im->bias_z_lsb     = (int32_t)(im->bias_acc_lsb /
                                         (int64_t)im->bias_acc_count);
          im->bias_acc_lsb   = 0;
          im->bias_acc_count = 0;
          im->bias_idle_streak_us = 0;
          im->calibrated     = true;
        }
    }
  else
    {
      /* Motion.  Reset the calibration window so a single noisy frame
       * doesn't poison the average.
       */

      im->bias_idle_streak_us = 0;
      im->bias_acc_lsb        = 0;
      im->bias_acc_count      = 0;
    }

  /* Heading integration: Δheading_mdeg = gz_corrected_mdps × Δt_us /
   * 1e6.  gz_corrected_mdps = gz_lsb × 70.
   */

  if (im->last_sample_ts_us != 0 && ts_us > im->last_sample_ts_us)
    {
      uint64_t dt_us = ts_us - im->last_sample_ts_us;
      int64_t  d_mdeg = (int64_t)gz_lsb * DB_IMU_GYRO_LSB_TO_MDPS *
                        (int64_t)dt_us / 1000000;
      im->heading_mdeg += d_mdeg;
    }

  im->last_sample_ts_us = ts_us;
  im->sample_count++;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int db_imu_open(struct db_imu_s *im)
{
  memset(im, 0, sizeof(*im));
  im->fd = -1;
  im->bias_idle_threshold_lsb = DB_IMU_DEFAULT_BIAS_THRESH;
  im->bias_window_us          = DB_IMU_DEFAULT_BIAS_WINDOW_US;

  im->fd = open(DB_IMU_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (im->fd < 0)
    {
      return -errno;
    }
  im->opened = true;
  return 0;
}

void db_imu_close(struct db_imu_s *im)
{
  if (im->fd >= 0)
    {
      close(im->fd);
      im->fd = -1;
    }
  im->opened = false;
}

bool db_imu_is_open(const struct db_imu_s *im)
{
  return im->opened;
}

int db_imu_drain_and_update(struct db_imu_s *im, uint64_t now_us)
{
  (void)now_us;
  if (!im->opened) return -ENODEV;

  struct sensor_imu batch[DB_IMU_DRAIN_BATCH];
  ssize_t n = read(im->fd, batch, sizeof(batch));
  if (n < 0)
    {
      if (errno == EAGAIN || errno == ENODATA)
        {
          return -EAGAIN;
        }
      return -errno;
    }

  size_t count = (size_t)n / sizeof(batch[0]);
  if (count == 0) return -EAGAIN;

  /* Process every sample in the batch — heading integration needs all
   * of them, not just the last one.  (Velocity-estimator-style
   * drop-to-latest doesn't apply because the integrator's accuracy is
   * proportional to sample-count.)
   */

  for (size_t i = 0; i < count; i++)
    {
      integrate(im, batch[i].gz, batch[i].timestamp);
    }
  return 0;
}

void db_imu_push_sample(struct db_imu_s *im,
                        int16_t gx, int16_t gy, int16_t gz,
                        int16_t ax, int16_t ay, int16_t az,
                        uint64_t ts_us)
{
  (void)gx;
  (void)gy;
  (void)ax;
  (void)ay;
  (void)az;
  if (!im->opened) return;
  integrate(im, gz, ts_us);
}

int64_t db_imu_get_heading_mdeg(const struct db_imu_s *im)
{
  return im->heading_mdeg;
}

void db_imu_set_heading_mdeg(struct db_imu_s *im, int64_t heading)
{
  im->heading_mdeg = heading;
}

int32_t db_imu_get_bias_z_lsb(const struct db_imu_s *im)
{
  return im->bias_z_lsb;
}

bool db_imu_is_calibrated(const struct db_imu_s *im)
{
  return im->calibrated;
}

void db_imu_recalibrate(struct db_imu_s *im)
{
  im->bias_acc_lsb        = 0;
  im->bias_acc_count      = 0;
  im->bias_idle_streak_us = 0;
  im->calibrated          = false;
}
