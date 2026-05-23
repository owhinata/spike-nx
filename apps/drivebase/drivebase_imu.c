/****************************************************************************
 * apps/drivebase/drivebase_imu.c
 *
 * IMU heading integration for the drivebase daemon (Phase 3a,
 * Issue #147).  Per sample we:
 *
 *   1. Detect FSR transitions (Issue #139) and rescale the cached
 *      gyro bias / idle threshold to preserve their physical meaning.
 *   2. Apply the Tedaldi gyro matmul (Phase 2.5, Issue #145) on
 *      bias-residual raw LSB and integrate the idle estimator over
 *      the corrected Z component.
 *   3. Apply the Tedaldi accel matmul, gated by an accel-FSR match
 *      check (cal was taken at a specific fsr_xl_g; if the runtime
 *      driver has switched FSR live we fall back to identity so the
 *      orientation filter does not see a wildly wrong gravity).
 *   4. Convert corrected gyro/accel to float (rad/s, g) and step a
 *      Madgwick 6-DOF IMU-only fusion, attenuating β by a pybricks-
 *      style stationary gate so vibration / wheel impacts can't pull
 *      yaw via the accel.
 *
 * Per drain (≈ daemon-tick = 500 Hz) we extract yaw with a single
 * atan2f and unwrap into heading_mdeg, keeping the public API on int64
 * even though the fusion state is float.  This per-drain extraction
 * is safe: even at 2000 dps the worst-case angular displacement
 * across a 2 ms RT tick is ~4 deg, far from the ±π wrap point.
 *
 * Sample rate is 833 Hz from the LSM6DSL boot config (we do not
 * change ODR — shared with btsensor).  The host ImuViewer
 * MadgwickFilter (Issue #146) ports the same algorithm + β=0.05 so
 * Hub vs host numbers stay comparable.
 *
 * integrate() must be called from a task context only (daemon RT
 * tick work-queue or the _imu CLI); the FPU is enabled (lazy stacking
 * disabled by arm_fpuconfig.c) so context switching is safe, but the
 * libm calls cost cycles the ISR path doesn't have to spare.
 ****************************************************************************/

#include <nuttx/config.h>

#if !defined(CONFIG_ARCH_FPU)
#  error "drivebase_imu Madgwick fusion requires CONFIG_ARCH_FPU"
#endif
#if !defined(CONFIG_LIBM_NEWLIB) && !defined(CONFIG_LIBM)
#  error "drivebase_imu Madgwick fusion requires libm (CONFIG_LIBM_NEWLIB)"
#endif

#include <errno.h>
#include <fcntl.h>
#include <math.h>
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

#ifndef M_PI_F
#  define M_PI_F                     3.14159265358979323846f
#endif

#define DB_IMU_DEG_PER_RAD_X1000    (180000.0f / M_PI_F)

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

/* Accel FSR enum (board_lsm6dsl.h: 0=2g, 1=16g, 2=4g, 3=8g) -> g
 * lookup.  Used by integrate() to compare the per-sample idx against
 * the cal-file fsr_xl_g and fall back to identity correction when the
 * live driver FSR has drifted away from the calibration condition.
 */

static uint8_t fsr_xl_idx_to_g(uint8_t idx)
{
  static const uint8_t table[] = {
    [0] = 2,
    [1] = 16,
    [2] = 4,
    [3] = 8,
  };
  return (idx < (sizeof(table) / sizeof(table[0]))) ? table[idx] : 0;
}

/* Madgwick 6-DOF IMU-only update (port of host
 * ImuViewer.Core/Filters/MadgwickFilter.cs::Update).  q is
 * (q0=w, q1=x, q2=y, q3=z); g_rps is gyro rad/s; a_g is accel in
 * g-units.  beta is the *effective* gain (stationary-gated by the
 * caller).  Normalises the quaternion at the end so accumulated
 * float error stays bounded.
 */

static void madgwick_update_imu(struct db_madgwick_state_s *m,
                                float ax, float ay, float az,
                                float gx, float gy, float gz,
                                float dt, float beta)
{
  float q0 = m->q0, q1 = m->q1, q2 = m->q2, q3 = m->q3;

  /* Rate of change from gyroscope. */

  float qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
  float qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
  float qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
  float qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

  float aNormSq = ax * ax + ay * ay + az * az;
  if (aNormSq > 1e-12f)
    {
      float aRecipNorm = 1.0f / sqrtf(aNormSq);
      ax *= aRecipNorm;
      ay *= aRecipNorm;
      az *= aRecipNorm;

      float _2q0 = 2.0f * q0;
      float _2q1 = 2.0f * q1;
      float _2q2 = 2.0f * q2;
      float _2q3 = 2.0f * q3;
      float _4q0 = 4.0f * q0;
      float _4q1 = 4.0f * q1;
      float _4q2 = 4.0f * q2;
      float _8q1 = 8.0f * q1;
      float _8q2 = 8.0f * q2;
      float q0q0 = q0 * q0;
      float q1q1 = q1 * q1;
      float q2q2 = q2 * q2;
      float q3q3 = q3 * q3;

      float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
      float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay
                 - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
      float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay
                 - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
      float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

      float sNormSq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
      if (sNormSq > 1e-12f)
        {
          float sRecipNorm = 1.0f / sqrtf(sNormSq);
          qDot1 -= beta * s0 * sRecipNorm;
          qDot2 -= beta * s1 * sRecipNorm;
          qDot3 -= beta * s2 * sRecipNorm;
          qDot4 -= beta * s3 * sRecipNorm;
        }
    }

  q0 += qDot1 * dt;
  q1 += qDot2 * dt;
  q2 += qDot3 * dt;
  q3 += qDot4 * dt;

  float qNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (qNormSq > 1e-12f)
    {
      float qRecipNorm = 1.0f / sqrtf(qNormSq);
      q0 *= qRecipNorm;
      q1 *= qRecipNorm;
      q2 *= qRecipNorm;
      q3 *= qRecipNorm;
    }

  m->q0 = q0;
  m->q1 = q1;
  m->q2 = q2;
  m->q3 = q3;
}

/* Seed the quaternion from a first accel sample so post-init the
 * gravity vector is already aligned to (0,0,1) in the body frame.
 * Without this the filter would start at identity (= board-up
 * assumption) and take β-scaled samples to converge, which on a
 * tilted Hub means seconds of yaw drift before _imu show would
 * report a stable tilt_deg.
 *
 * Formula: shortest-arc quaternion from world-down (0,0,1) to the
 * normalised accel vector.  Hand-derived in the (q0=w, q1=x, q2=y,
 * q3=z) convention.  Falls back to identity if the accel norm is
 * degenerate.
 */

static void madgwick_seed_from_accel(struct db_madgwick_state_s *m,
                                     float ax, float ay, float az)
{
  float norm_sq = ax * ax + ay * ay + az * az;
  if (norm_sq < 1e-12f)
    {
      m->q0 = 1.0f;
      m->q1 = m->q2 = m->q3 = 0.0f;
      m->initialized = true;
      return;
    }

  float recip = 1.0f / sqrtf(norm_sq);
  float ux = ax * recip;
  float uy = ay * recip;
  float uz = az * recip;

  /* Shortest-arc body-to-world quaternion: q rotates the body-frame
   * accel direction u onto the world-up vector v = (0,0,1).
   *
   *   q_unnorm = (1 + u·v, u × v)
   *            = (1 + uz, (uy, -ux, 0))
   *
   * When uz approaches -1 (sensor upside-down) the formula
   * degenerates; pick a fallback 180° rotation about X.
   *
   * Note: an earlier draft used q = (1 + uz, -uy, ux, 0), which is
   * the conjugate (world-to-body) of the shortest arc and seeded the
   * filter on the *opposite* side of the accel constraint manifold.
   * The accel correction is too weak to slide q back across the
   * manifold under β=0.05 + stationary gate, so heading then traced
   * the wrong projection of world-yaw (0° → +39° → 0° → -39° → 0°
   * for a 51° tilted IMU).  See Issue #147 hardware-verification
   * notes.
   */

  float q0 = 1.0f + uz;
  float q1 = uy;
  float q2 = -ux;
  float q3 = 0.0f;

  float qNormSq = q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3;
  if (qNormSq < 1e-6f)
    {
      /* Upside-down: rotate π about X (q = (0, 1, 0, 0)). */

      m->q0 = 0.0f;
      m->q1 = 1.0f;
      m->q2 = m->q3 = 0.0f;
    }
  else
    {
      float r = 1.0f / sqrtf(qNormSq);
      m->q0 = q0 * r;
      m->q1 = q1 * r;
      m->q2 = q2 * r;
      m->q3 = q3 * r;
    }

  m->initialized = true;
}

/* Wrap-safe modular dt (µs).  sensor_imu.timestamp is the low 32 bits
 * of CLOCK_BOOTTIME us; subtracting in uint32_t domain gives the
 * natural mod-2^32 difference, which is correct as long as the
 * inter-sample gap stays well below 71 minutes — true for any LSM6DSL
 * configuration (default 833 Hz = 1.2 ms gap).
 */

static uint32_t wrap_safe_dt_us(uint32_t now, uint32_t prev)
{
  return (uint32_t)(now - prev);
}

/* Wrap a delta into (-π, π] for unwrap accumulation. */

static float wrap_pi(float x)
{
  while (x >   M_PI_F) x -= 2.0f * M_PI_F;
  while (x <= -M_PI_F) x += 2.0f * M_PI_F;
  return x;
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
                      int16_t ax_raw, int16_t ay_raw, int16_t az_raw,
                      uint8_t fsr_gy_idx, uint8_t fsr_xl_idx,
                      uint32_t ts_us)
{
  /* Issue #139: detect FSR transitions before any bias arithmetic. */

  if (fsr_gy_idx != im->cur_fsr_gy_idx)
    {
      apply_fsr_change(im, fsr_gy_idx);
    }

  /* Phase 2.5: 3×3 gyro matmul with x1000 fractional bias.
   *
   *   residual_x1000[j] = g_raw[j] * 1000 - bias_lsb_x1000[j]
   *   g_corr_x1000[i]   = (sum_j M_x1000[i][j] * residual_x1000[j]) / 1000
   *
   * Both sides of the multiplication stay in 64-bit so a worst-case
   * |M_x1000| ≈ 1100 and |residual| ≈ 33e6 (full-scale int16 raw +
   * bias drift) sum to at most ~1.1e11 across the 3 terms — well
   * within int64.  Phase 3a consumes the full 3-vector (Madgwick),
   * not just Z, so the matmul is no longer "computed for free".
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

  /* Bias estimation gate (idle EMA over Z).  Threshold lives in
   * raw-LSB so multiply by 1000 to compare against millis-LSB.
   * Using the corrected residual keeps the physical meaning even
   * when the off-diagonal matrix terms nudge the effective bias.
   */

  int64_t threshold_x1000 = (int64_t)im->bias_idle_threshold_lsb * 1000;

  if (llabs(gz_corr_x1000) < threshold_x1000)
    {
      /* Idle.  Accumulate raw Z × 1000 so the estimate tracks the
       * gyro's true drift.
       */

      im->bias_acc_lsb_x1000 += (int64_t)gz_raw * 1000;
      im->bias_acc_count++;

      if (im->last_sample_valid)
        {
          im->bias_idle_streak_us += wrap_safe_dt_us(ts_us,
                                                     im->last_sample_ts_us);
        }

      if (im->bias_idle_streak_us >= im->bias_window_us &&
          im->bias_acc_count > 0)
        {
          int64_t new_bias_z_x1000 = im->bias_acc_lsb_x1000 /
                                     (int64_t)im->bias_acc_count;

          if (!im->calibrated)
            {
              /* First successful idle window after open / recalibrate.
               * Full assignment instead of α-blend so the very first
               * capture lands on the actual bias.
               */

              im->bias_lsb_x1000[2] = (int32_t)new_bias_z_x1000;
            }
          else
            {
              /* α = 0.1 EMA: (new + 9 * old) / 10. */

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
      im->bias_idle_streak_us = 0;
      im->bias_acc_lsb_x1000  = 0;
      im->bias_acc_count      = 0;
    }

  /* Accel: apply the Tedaldi accel matmul gated on FSR match.
   *
   * Cal was taken at a specific fsr_xl_g; if the runtime driver has
   * switched FSR live (Issue #139) the bias and matrix scale would
   * be wrong by a factor of (cal_g / live_g).  Detect mismatch and
   * fall back to (raw - 0) identity correction so the orientation
   * filter sees a roughly-correct gravity vector — degraded tilt
   * but not divergent.
   */

  uint8_t live_xl_g = fsr_xl_idx_to_g(fsr_xl_idx);
  bool xl_match = (im->cal.loaded && im->cal.fsr_xl_g != 0 &&
                   live_xl_g == im->cal.fsr_xl_g);
  im->accel_fsr_match = (uint8_t)xl_match;

  const int16_t a_raw[3] = { ax_raw, ay_raw, az_raw };
  int64_t a_corr_x1000[3];

  if (xl_match)
    {
      for (int i = 0; i < 3; i++)
        {
          int64_t sum = 0;
          for (int j = 0; j < 3; j++)
            {
              int64_t residual_j = (int64_t)a_raw[j] * 1000
                                  - (int64_t)im->cal.accel_bias_lsb_x1000[j];
              sum += (int64_t)im->cal.accel_M_x1000[i][j] * residual_j;
            }

          a_corr_x1000[i] = sum / 1000;
        }
    }
  else
    {
      /* Identity correction.  Keeps the x1000 scale consistent with
       * the matched branch so the float conversion below stays one
       * expression.
       */

      a_corr_x1000[0] = (int64_t)ax_raw * 1000;
      a_corr_x1000[1] = (int64_t)ay_raw * 1000;
      a_corr_x1000[2] = (int64_t)az_raw * 1000;
    }

  /* Float conversion.
   *
   * Gyro: corrected x1000 LSB → rad/s.  Existing fixed-point uses
   *   gyro_mdps_num = cur_fsr_gy_dps * 35   [mdps/LSB × 1000]
   * (repo convention 35 mdps/LSB at FSR=1000 dps; datasheet 30.5).
   * For float we divide by 1000 to recover mdps/LSB, then chain:
   *   ω_f [rad/s] = corr_x1000 * mdps_per_lsb * π/180 / 1e6
   * The /1e6 folds the x1000 (corr) × 1000 (mdps→dps) → 1e6, and the
   * /1000 inside mdps→dps cancels with x1000 to a single /1e6.
   *
   * Sanity (FSR=1000 dps): 1 LSB = 35 mdps = 35e-3 * π/180 ≈
   * 6.109e-4 rad/s — matches lsm6dsl_uorb.c and the cal-file
   * informational constant.
   *
   * Accel: corrected x1000 LSB → g-units.  ±cal.fsr_xl_g maps to
   * ±32768 LSB, so a_f[g] = corr_x1000 * (fsr_xl_g / 32768) / 1000.
   * If cal is identity (mismatch / not loaded) we still need a g
   * value; use live_xl_g when known, else default to 2g (the post-
   * boot driver default).  Wrong scale would only stretch the
   * accel vector — Madgwick re-normalises before the s-error
   * gradient, so the gain stays predictable.
   */

  float mdps_per_lsb = (float)im->gyro_mdps_num / 1000.0f;
  float wf[3];
  for (int i = 0; i < 3; i++)
    {
      wf[i] = (float)g_corr_x1000[i] * mdps_per_lsb
              * (M_PI_F / 180.0f) / 1.0e6f;
    }

  uint8_t a_scale_g = xl_match ? im->cal.fsr_xl_g
                               : (live_xl_g != 0 ? live_xl_g : 2);
  float a_scale = (float)a_scale_g / (32768.0f * 1000.0f);
  float af[3];
  for (int i = 0; i < 3; i++)
    {
      af[i] = (float)a_corr_x1000[i] * a_scale;
    }

  /* Stationary-gated β (pybricks pbio/src/imu.c:327 form, converted
   * to (rad/s, g) units).  When the chassis is still both errs go to
   * zero and β_eff == β_base; under wheel impacts / vibration the
   * accel-err climbs > ACCL_MIN_G and β scales down toward zero so
   * Madgwick's accel-correction term stops pulling yaw.
   */

  float a_norm = sqrtf(af[0] * af[0] + af[1] * af[1] + af[2] * af[2]);
  float accl_err = fabsf(a_norm - 1.0f);
  float gyro_err = sqrtf(wf[0] * wf[0] + wf[1] * wf[1] + wf[2] * wf[2]);

  float accl_term = DB_IMU_ACCL_MIN_G /
                    (accl_err > DB_IMU_ACCL_MIN_G ? accl_err : DB_IMU_ACCL_MIN_G);
  float gyro_term = DB_IMU_GYRO_MIN_RADPS /
                    (gyro_err > DB_IMU_GYRO_MIN_RADPS ? gyro_err : DB_IMU_GYRO_MIN_RADPS);

  float stationary = accl_term * gyro_term;
  if (stationary > 1.0f) stationary = 1.0f;
  if (stationary < 0.0f) stationary = 0.0f;
  float beta_eff = im->madgwick.beta * stationary;

  /* First-sample bootstrap: seed the quaternion from accel, then
   * skip the Madgwick update (no dt yet) so the seed is not
   * immediately wobbled by a 1.2 ms integration step.
   */

  if (!im->madgwick.initialized)
    {
      madgwick_seed_from_accel(&im->madgwick, af[0], af[1], af[2]);
      im->last_sample_ts_us = ts_us;
      im->last_sample_valid = true;
      im->sample_count++;
      return;
    }

  if (im->last_sample_valid)
    {
      uint32_t dt_us = wrap_safe_dt_us(ts_us, im->last_sample_ts_us);

      /* Skip the Madgwick step when dt_us is implausibly large
       * (sensor hang / I²C recovery): 20 ms = ~16 nominal sample
       * periods.  This is freeze-not-clamp — quaternion stays put
       * for the long-dt sample, so per-drain yaw extraction sees
       * ψ_curr == ψ_prev and heading_mdeg also freezes.  Once
       * sampling resumes the next dt_us is back under the gate and
       * fusion catches up live.  Free-integrating gz for hundreds
       * of ms during a stall would otherwise corrupt yaw far more
       * than a 20 ms freeze does.
       */

      if (dt_us > 0 && dt_us < 20000)
        {
          float dt_s = (float)dt_us * 1.0e-6f;
          madgwick_update_imu(&im->madgwick,
                              af[0], af[1], af[2],
                              wf[0], wf[1], wf[2],
                              dt_s, beta_eff);
        }
    }

  im->last_sample_ts_us = ts_us;
  im->last_sample_valid = true;
  im->sample_count++;
}

/* Extract world-vertical yaw (rad) from the Madgwick quaternion and
 * unwrap-accumulate into heading_mdeg.  Called per drain (not per
 * sample): even at 2000 dps the worst-case dψ across a 2 ms RT tick
 * is ~4 deg, far from ±π, so unwrap accuracy holds.
 */

static void extract_yaw_and_unwrap(struct db_imu_s *im)
{
  if (!im->madgwick.initialized)
    {
      return;
    }

  float q0 = im->madgwick.q0;
  float q1 = im->madgwick.q1;
  float q2 = im->madgwick.q2;
  float q3 = im->madgwick.q3;

  float psi = atan2f(2.0f * (q0 * q3 + q1 * q2),
                     1.0f - 2.0f * (q2 * q2 + q3 * q3));

  if (!im->psi_valid)
    {
      im->psi_prev  = psi;
      im->psi_valid = true;
      return;
    }

  float dpsi = wrap_pi(psi - im->psi_prev);
  im->heading_mdeg += (int64_t)(dpsi * DB_IMU_DEG_PER_RAD_X1000);
  im->psi_prev = psi;
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

  /* Phase 3a: Madgwick state.  Quaternion starts at identity; the
   * first integrate() call replaces it with a shortest-arc seed from
   * the first accel sample so post-init the gravity vector is
   * already aligned.
   */

  im->madgwick.q0           = 1.0f;
  im->madgwick.q1           = 0.0f;
  im->madgwick.q2           = 0.0f;
  im->madgwick.q3           = 0.0f;
  im->madgwick.beta         = DB_IMU_MADGWICK_BETA;
  im->madgwick.initialized  = false;
  im->psi_valid             = false;
  im->accel_fsr_match       = 0;
  im->last_drain_ok_us      = 0;
  im->last_sample_valid     = false;

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

  /* Process every sample in the batch — Madgwick accuracy is
   * proportional to sample-count, so drop-to-latest is not safe.
   */

  for (size_t i = 0; i < count; i++)
    {
      integrate(im,
                batch[i].gx, batch[i].gy, batch[i].gz,
                batch[i].ax, batch[i].ay, batch[i].az,
                batch[i].fsr_gy_idx, batch[i].fsr_xl_idx,
                batch[i].timestamp);
    }

  /* Per-drain: extract world-vertical yaw and unwrap-accumulate. */

  extract_yaw_and_unwrap(im);

  /* Cache last sample's temperature for the Section F `_imu show` /
   * `_imu drift` verbs.  The driver decimates OUT_TEMP reads
   * internally so this value is updated every TEMP_DECIMATE = 16
   * samples on the publish side, which is plenty for ambient
   * temperature logging.
   */

  im->last_temperature_raw = batch[count - 1].temperature_raw;
  im->last_drain_ok_us     = now_us;
  return 0;
}

void db_imu_push_sample(struct db_imu_s *im,
                        int16_t gx, int16_t gy, int16_t gz,
                        int16_t ax, int16_t ay, int16_t az,
                        uint8_t fsr_xl_idx, uint32_t ts_us)
{
  if (!im->opened) return;

  /* No external caller knows the gyro FSR idx for a hand-injected
   * sample; use the integrator's current cached idx so the scale
   * stays consistent with whatever the driver last fed in.  If the
   * integrator has never seen a real sample, gyro_mdps_num is 0 and
   * the corrected gyro stays at LSB — Madgwick still runs but only
   * the accel correction is meaningful, which is fine for
   * unit-test injection.
   */

  integrate(im, gx, gy, gz, ax, ay, az,
            im->cur_fsr_gy_idx, fsr_xl_idx, ts_us);
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

bool db_imu_is_stale(const struct db_imu_s *im, uint64_t now_us,
                     uint32_t threshold_us)
{
  if (!im->opened || im->last_drain_ok_us == 0)
    {
      return true;
    }

  if (now_us <= im->last_drain_ok_us)
    {
      return false;
    }

  return (now_us - im->last_drain_ok_us) > (uint64_t)threshold_us;
}

void db_imu_get_quaternion(const struct db_imu_s *im, float q[4])
{
  q[0] = im->madgwick.q0;
  q[1] = im->madgwick.q1;
  q[2] = im->madgwick.q2;
  q[3] = im->madgwick.q3;
}

float db_imu_get_tilt_deg(const struct db_imu_s *im)
{
  /* tilt = angle between body Z and world Z.  In (q0=w, q1=x, q2=y,
   * q3=z) convention the rotation-matrix R[2][2] entry is
   *   cos(tilt) = 1 - 2*(q1² + q2²).
   * Clamp into [-1,1] before acosf to defend against FP rounding.
   */

  float c = 1.0f - 2.0f * (im->madgwick.q1 * im->madgwick.q1 +
                           im->madgwick.q2 * im->madgwick.q2);
  if (c >  1.0f) c =  1.0f;
  if (c < -1.0f) c = -1.0f;
  return acosf(c) * (180.0f / M_PI_F);
}

float db_imu_get_beta(const struct db_imu_s *im)
{
  return im->madgwick.beta;
}
