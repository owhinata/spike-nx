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
 * (default 71 LSB ≈ 2.5 dps at the system default FS=1000 dps) for
 * `bias_window_us` (default 200 ms), we accumulate the running
 * average and update bias_z_lsb.  The subtracting integrator then
 * produces ~0 deg drift per second when the chassis is still.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/board/board_lsm6dsl.h>   /* struct sensor_imu        */

#include "drivebase_imu.h"
#include "drivebase_imu_cal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_IMU_DEVPATH               "/dev/uorb/sensor_imu0"
#define DB_IMU_DEFAULT_BIAS_THRESH    71u    /* ≈ 2.5 dps @ FS=1000 dps  */
                                             /* (Phase 2.5: narrowed     */
                                             /* from 143/10 dps so slow  */
                                             /* curves are not mistaken  */
                                             /* for idle).               */
#define DB_IMU_DEFAULT_BIAS_WINDOW_US 200000u

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

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

/* On FSR change, rescale the cached bias_lsb_x1000 (raw-LSB × 1000
 * units) to preserve its physical (mdps) meaning under the new
 * sensitivity.  All 3 axes are rescaled so a later live SET FSR does
 * not leak into the matmul off-diagonal terms.  Rescale the
 * bias_idle_threshold_lsb too: the default 71 LSB is tuned for
 * 1000 dps (≈ 2.5 dps), so for 500 dps it becomes 142 LSB (and for
 * 2000 dps it becomes 35 LSB) to keep the idle window at the same
 * physical rate.
 *
 * Must run BEFORE the per-sample matmul, otherwise the new sample
 * would be corrected with the wrong-units bias and add a one-sample
 * drift each time FSR changes.
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
      /* First sample after open (or recovering from invalid idx).
       * When the cal file loaded, db_imu_open already populated
       * bias_lsb_x1000 from cal->gyro_bias_lsb_x1000 — leave it.  No
       * cal means the integrator starts at zero bias and the idle
       * estimator converges from there.
       */

      if (!im->cal.loaded)
        {
          im->bias_lsb_x1000[0] = 0;
          im->bias_lsb_x1000[1] = 0;
          im->bias_lsb_x1000[2] = 0;
        }

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
       * ratio so the physical rate it represents is preserved.  All
       * three axes rescaled — the matmul off-diagonal terms would
       * otherwise leak axes that were at the old scale into the new-
       * scale corrected output.
       */

      for (int i = 0; i < 3; i++)
        {
          im->bias_lsb_x1000[i] = (int32_t)(
              (int64_t)im->bias_lsb_x1000[i] * old_dps / new_dps);
        }

      im->bias_idle_threshold_lsb =
          (uint32_t)((uint64_t)DB_IMU_DEFAULT_BIAS_THRESH
                     * DB_IMU_DEFAULT_FSR_GY_DPS / new_dps);
      db_imu_recalibrate(im);
    }
}

static void integrate(struct db_imu_s *im,
                      int16_t gx_raw, int16_t gy_raw, int16_t gz_raw,
                      uint8_t fsr_gy_idx, uint64_t ts_us)
{
  /* Issue #139: detect FSR transitions before any bias arithmetic. */

  if (fsr_gy_idx != im->cur_fsr_gy_idx)
    {
      apply_fsr_change(im, fsr_gy_idx);
    }

  /* Phase 2.5: 3×3 matmul with x1000 fractional bias.
   *
   *   residual_x1000[j] = g_raw[j] * 1000 - bias_lsb_x1000[j]
   *   g_corr_x1000[i]   = (sum_j M_x1000[i][j] * residual_x1000[j]) / 1000
   *
   * Both sides of the multiplication stay in 64-bit so a worst-case
   * |M_x1000| ≈ 1100 and |residual| ≈ 33e6 (full-scale int16 raw +
   * bias drift) sum to at most ~1.1e11 across the 3 terms — well
   * within int64.  The final divide-by-1000 keeps g_corr_x1000 in
   * millis-LSB so the downstream heading integration and idle gate
   * see the same units.
   *
   * We only consume index 2 (Z) downstream; computing the full 3-row
   * matmul costs the same since the matrix is small and the loop
   * unrolls trivially.  Keeping the loop also leaves the X/Y entries
   * available for future tilt-estimator wiring without another
   * refactor.
   */

  const int16_t g_raw[3] = { gx_raw, gy_raw, gz_raw };
  int64_t g_corr_x1000[3];

  for (int i = 0; i < 3; i++)
    {
      int64_t sum = 0;
      for (int j = 0; j < 3; j++)
        {
          int64_t residual_j = (int64_t)g_raw[j] * 1000
                              - (int64_t)im->bias_lsb_x1000[j];
          sum += (int64_t)im->cal.gyro_M_x1000[i][j] * residual_j;
        }

      g_corr_x1000[i] = sum / 1000;
    }

  int64_t gz_corr_x1000 = g_corr_x1000[2];

  /* Bias estimation gate.  Compare in millis-LSB so the threshold
   * (raw-LSB) needs to be scaled up.  Using the corrected residual
   * (not raw - bias_z_only) means the gate keeps its physical
   * meaning even when the off-diagonal matrix terms have nudged the
   * effective bias.
   */

  int64_t threshold_x1000 = (int64_t)im->bias_idle_threshold_lsb * 1000;

  if (llabs(gz_corr_x1000) < threshold_x1000)
    {
      /* Idle.  Accumulate raw Z × 1000 (not bias-corrected) so the
       * estimate tracks the true drift of the gyro itself.
       */

      im->bias_acc_lsb_x1000 += (int64_t)gz_raw * 1000;
      im->bias_acc_count++;

      if (im->last_sample_ts_us != 0 && ts_us > im->last_sample_ts_us)
        {
          im->bias_idle_streak_us += ts_us - im->last_sample_ts_us;
        }

      if (im->bias_idle_streak_us >= im->bias_window_us &&
          im->bias_acc_count > 0)
        {
          int64_t new_bias_z_x1000 = im->bias_acc_lsb_x1000 /
                                     (int64_t)im->bias_acc_count;

          if (!im->calibrated)
            {
              /* First successful idle window after open / recalibrate.
               * Full assignment so the very first capture lands on the
               * actual bias instead of a 10 % blend with whatever
               * (often zero or a stale cal value) was sitting in
               * bias_lsb_x1000[2].
               */

              im->bias_lsb_x1000[2] = (int32_t)new_bias_z_x1000;
            }
          else
            {
              /* α = 0.1 EMA tracks slow temperature drift without
               * over-reacting to a single noisy idle window.  Integer
               * form: (new + 9 * old) / 10.  At FSR=1000 dps with
               * x1000 fractional, this preserves sub-LSB precision
               * across temperature swings of ~20°C.
               */

              int64_t prev_x1000 = (int64_t)im->bias_lsb_x1000[2];
              im->bias_lsb_x1000[2] = (int32_t)
                  ((new_bias_z_x1000 + prev_x1000 * 9) / 10);
            }

          im->bias_acc_lsb_x1000  = 0;
          im->bias_acc_count      = 0;
          im->bias_idle_streak_us = 0;
          im->calibrated          = true;
        }
    }
  else
    {
      /* Motion.  Reset the calibration window so a single noisy frame
       * doesn't poison the average.
       */

      im->bias_idle_streak_us = 0;
      im->bias_acc_lsb_x1000  = 0;
      im->bias_acc_count      = 0;
    }

  /* Heading integration in x1000 domain:
   *
   *   d_mdeg = gz_corr_x1000 (millis-LSB) × gyro_mdps_num
   *            × dt_us / (1000 × 1e9)
   *
   * The (/ 1000) drops the millis-LSB scale; the (/ 1e9) folds the
   * 1/32768 sensitivity into the 1/1e6 dt seconds factor as before
   * (see the pre-Phase 2.5 comment).  gyro_mdps_num == 0 means the
   * FSR idx was invalid — skip integration rather than producing
   * garbage heading.
   */

  if (im->last_sample_ts_us != 0 && ts_us > im->last_sample_ts_us
      && im->gyro_mdps_num != 0)
    {
      uint64_t dt_us = ts_us - im->last_sample_ts_us;
      int64_t  d_mdeg = gz_corr_x1000 * (int64_t)im->gyro_mdps_num *
                        (int64_t)dt_us /
                        ((int64_t)1000 * 1000000000);
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

  /* Phase 2.5: load offline calibration (Identity + zero bias on any
   * failure, so the daemon runs uncalibrated rather than refusing to
   * start).  Seed bias_lsb_x1000 from the cal-file initial bias so
   * the very first integration step already has a reasonable bias —
   * otherwise the daemon would drift heavily until the idle estimator
   * converges (which takes 200 ms of stillness, not guaranteed
   * immediately after `drivebase start`).
   */

  (void)db_imu_cal_load(&im->cal);
  for (int i = 0; i < 3; i++)
    {
      im->bias_lsb_x1000[i] = im->cal.gyro_bias_lsb_x1000[i];
    }

  /* Issue #139: force fresh-start FSR scaling on the very first sample.
   * Any valid driver enum value will mismatch the UNKNOWN sentinel,
   * triggering apply_fsr_change() to compute gyro_mdps_num.  The
   * first-sample path preserves the cal-loaded bias_lsb_x1000 when
   * cal->loaded; otherwise it zeroes them.
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
      integrate(im,
                batch[i].gx, batch[i].gy, batch[i].gz,
                batch[i].fsr_gy_idx, batch[i].timestamp);
    }

  /* Cache last sample's temperature for the Section F `_imu show` /
   * `_imu drift` verbs.  The driver decimates OUT_TEMP reads
   * internally so this value is updated every TEMP_DECIMATE = 16
   * samples on the publish side, which is plenty for ambient
   * temperature logging.
   */

  im->last_temperature_raw = batch[count - 1].temperature_raw;
  return 0;
}

void db_imu_push_sample(struct db_imu_s *im,
                        int16_t gx, int16_t gy, int16_t gz,
                        int16_t ax, int16_t ay, int16_t az,
                        uint64_t ts_us)
{
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

  integrate(im, gx, gy, gz, im->cur_fsr_gy_idx, ts_us);
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
  /* Backward-compatible accessor: return the Z-axis bias in raw LSB
   * (truncated from the x1000 fractional storage).  Existing CLIs
   * (`drivebase _imu calibrate` / `_imu heading`) and tests print
   * this as an int LSB value.  Use db_imu_get_bias_z_lsb_x1000() if
   * sub-LSB precision matters (Section F verify CLI).
   */

  return (int32_t)(im->bias_lsb_x1000[2] / 1000);
}

int32_t db_imu_get_bias_z_lsb_x1000(const struct db_imu_s *im)
{
  return im->bias_lsb_x1000[2];
}

int16_t db_imu_get_temperature_raw(const struct db_imu_s *im)
{
  return im->last_temperature_raw;
}

bool db_imu_is_calibrated(const struct db_imu_s *im)
{
  return im->calibrated;
}

void db_imu_recalibrate(struct db_imu_s *im)
{
  im->bias_acc_lsb_x1000  = 0;
  im->bias_acc_count      = 0;
  im->bias_idle_streak_us = 0;
  im->calibrated          = false;
}
