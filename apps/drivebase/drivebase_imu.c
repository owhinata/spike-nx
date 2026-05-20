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

/* Issue #139: enum lsm6dsl_fsr_gy_e -> dps lookup, mirrored from the
 * driver-private enum in boards/spike-prime-hub/src/lsm6dsl_uorb.c
 * (0=250, 1=125, 2=500, 3=invalid, 4=1000, 5=invalid, 6=2000).  An
 * unknown idx returns 0 and integrate() then skips the heading update
 * (gyro_mdps_num stays 0) — fail-soft when the driver enum drifts.
 */

static uint16_t fsr_gy_idx_to_dps(uint8_t idx)
{
  static const uint16_t table[] = {
    [0] = 250,
    [1] = 125,
    [2] = 500,
    [3] = 0,
    [4] = 1000,
    [5] = 0,
    [6] = 2000,
  };
  return (idx < (sizeof(table) / sizeof(table[0]))) ? table[idx] : 0;
}

/* On FSR change, rescale the cached bias_z_lsb (raw-LSB units) to
 * preserve its physical (mdps) meaning under the new sensitivity.
 * Rescale the bias_idle_threshold_lsb too: the default 143 LSB is
 * tuned for 2000 dps (≈ 10 dps), so for 500 dps it becomes 572 LSB to
 * keep the idle window at the same physical rate.
 *
 * Must run BEFORE the per-sample subtraction `gz_raw - bias_z_lsb`,
 * otherwise the new sample would be biased with the wrong-units value
 * and add a one-sample drift each time FSR changes.
 */

static void apply_fsr_change(struct db_imu_s *im, uint8_t new_idx)
{
  uint16_t new_dps = fsr_gy_idx_to_dps(new_idx);
  uint16_t old_dps = im->cur_fsr_gy_dps;
  bool     first   = (im->cur_fsr_gy_idx == DB_IMU_FSR_GY_IDX_UNKNOWN);

  im->cur_fsr_gy_idx = new_idx;
  im->cur_fsr_gy_dps = new_dps;
  im->gyro_mdps_num  = (int32_t)new_dps * 35;     /* 0 if invalid */

  if (new_dps == 0)
    {
      return;                                     /* keep bias as-is */
    }

  if (first || old_dps == 0)
    {
      /* First sample after open (or recovering from invalid idx):
       * no meaningful old bias to convert.  Start fresh and let the
       * idle accumulator re-converge.
       */

      im->bias_z_lsb = 0;
      im->bias_idle_threshold_lsb =
          (uint32_t)((uint64_t)DB_IMU_DEFAULT_BIAS_THRESH
                     * DB_IMU_DEFAULT_FSR_GY_DPS / new_dps);
      db_imu_recalibrate(im);
      return;
    }

  if (old_dps != new_dps)
    {
      /* sensitivity_new / sensitivity_old = old_dps / new_dps  (smaller
       * FSR → more LSB per dps).  Multiply the cached raw bias by that
       * ratio so the physical rate it represents is preserved.
       */

      im->bias_z_lsb = (int32_t)((int64_t)im->bias_z_lsb * old_dps / new_dps);
      im->bias_idle_threshold_lsb =
          (uint32_t)((uint64_t)DB_IMU_DEFAULT_BIAS_THRESH
                     * DB_IMU_DEFAULT_FSR_GY_DPS / new_dps);
      db_imu_recalibrate(im);
    }
}

static void integrate(struct db_imu_s *im, int16_t gz_raw,
                      uint8_t fsr_gy_idx, uint64_t ts_us)
{
  /* Issue #139: detect FSR transitions before any bias arithmetic. */

  if (fsr_gy_idx != im->cur_fsr_gy_idx)
    {
      apply_fsr_change(im, fsr_gy_idx);
    }

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
   * 1e6.  gz_corrected_mdps = gz_lsb × (fsr_dps × 35) / 1000 because
   * the LSM6DSL sensitivity at FSR=N dps is N × 1000 / 32768 mdps/LSB
   * ≈ N × 0.03052 mdps/LSB ≈ N × 35 / 1000 mdps/LSB (the / 32768 is
   * folded with the / 1e6 in the dt term below into a single / 1e9).
   * gyro_mdps_num == 0 means the FSR idx is invalid: skip integration
   * rather than producing garbage heading.
   */

  if (im->last_sample_ts_us != 0 && ts_us > im->last_sample_ts_us
      && im->gyro_mdps_num != 0)
    {
      uint64_t dt_us = ts_us - im->last_sample_ts_us;
      int64_t  d_mdeg = (int64_t)gz_lsb * im->gyro_mdps_num *
                        (int64_t)dt_us / 1000000000;
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

  /* Issue #139: force fresh-start FSR scaling on the very first sample.
   * Any valid driver enum value will mismatch the UNKNOWN sentinel,
   * triggering apply_fsr_change() to compute gyro_mdps_num and set
   * bias_z_lsb=0 (no rescale needed because there is no old bias).
   */

  im->cur_fsr_gy_idx = DB_IMU_FSR_GY_IDX_UNKNOWN;
  im->cur_fsr_gy_dps = 0;
  im->gyro_mdps_num  = 0;

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
      integrate(im, batch[i].gz, batch[i].fsr_gy_idx, batch[i].timestamp);
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
  /* No external caller knows the FSR idx for a hand-injected sample;
   * use the integrator's current cached idx so the scale stays
   * consistent with whatever the driver last fed in.  If the
   * integrator has never seen a real sample, gyro_mdps_num is 0 and
   * the heading update is skipped (fail-soft).
   */

  integrate(im, gz, im->cur_fsr_gy_idx, ts_us);
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
