/****************************************************************************
 * apps/drivebase/drivebase_imu.h
 *
 * IMU integration for the drivebase daemon.  Drains LSM6DSL samples
 * from /dev/uorb/sensor_imu0, applies the Phase 2.5 Tedaldi gyro/accel
 * correction, runs a Madgwick 6-DOF fusion (Phase 3a, Issue #147) to
 * recover world-vertical yaw on a tilted Hub mounting, and exposes
 *   db_imu_get_heading_mdeg() / set_heading() for the drivebase
 *   aggregator's optional gyro-locked heading mode.
 *
 * Heading is the unwrapped world-frame yaw projected back into mdeg
 * about the robot's actual rotation axis.  The fixed-point integration
 * over raw gz_corr is gone; integrate() now feeds the corrected gyro
 * (rad/s) and accel (g) into Madgwick, then atan2f-extracts yaw once
 * per drain and unwraps into im->heading_mdeg.
 *
 * Coupled-loop wiring into drivebase_drivebase (heading_actual = gyro
 * instead of L−R / axle) is Phase 3b.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_IMU_H
#define __APPS_DRIVEBASE_DRIVEBASE_IMU_H

#include <stdbool.h>
#include <stdint.h>

#include "drivebase_imu_cal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Issue #139: gyro sensitivity now derived dynamically from the
 * per-sample fsr_gy_idx embedded in struct sensor_imu, so the
 * historical fixed `DB_IMU_GYRO_LSB_TO_MDPS = 70` (FS=2000 dps only)
 * is gone.  See integrate() in drivebase_imu.c for the lookup.
 *
 * Phase 2.5: driver default FSR is now ±1000 dps (Section A1).  The
 * default bias-idle threshold and reference dps below are anchored at
 * that new value; when the driver reports a different FSR the
 * integrate() path rescales the cached threshold so the physical
 * idle window (≈ 2.5 dps) stays roughly constant.
 */

#define DB_IMU_DRAIN_BATCH         16
#define DB_IMU_DEFAULT_FSR_GY_DPS  1000

/* Sentinel for "first sample after open" — any valid driver enum
 * value will mismatch this on the first integrate(), forcing the
 * fresh-start path that initialises gyro_mdps_num.
 */

#define DB_IMU_FSR_GY_IDX_UNKNOWN  0xFF

/* Stale threshold (µs) for db_imu_is_stale().  50 ms ≈ 25 RT-tick at
 * 2 ms, covers the 3-retry I2C reset window (Issue #102) without
 * letting Phase 3b's heading PID run on data older than that.
 */

#define DB_IMU_DEFAULT_STALE_THRESHOLD_US  50000u

/* Phase 3a: Madgwick fusion gain and the stationary-gate thresholds
 * (pybricks pbio/src/imu.c:327 form, converted to this plan's units of
 * rad/s and g): below GYRO_MIN_RADPS *and* with |‖a‖-1| below
 * ACCL_MIN_G the accel correction runs at full β; above either, β is
 * attenuated linearly toward 0 so wheel impacts / vibration do not
 * pull yaw via the accel.
 */

#define DB_IMU_MADGWICK_BETA       0.05f
#define DB_IMU_GYRO_MIN_RADPS      0.174533f   /* = 10 deg/s */
#define DB_IMU_ACCL_MIN_G          0.015296f   /* = 150 mm/s² / 9806.65 */

/****************************************************************************
 * Types
 ****************************************************************************/

/* Madgwick 6-DOF IMU-only fusion state.  Quaternion is stored as
 * (q0=w, q1=x, q2=y, q3=z) to match host MadgwickFilter.cs and the
 * 2010 paper.
 */

struct db_madgwick_state_s
{
  float    q0, q1, q2, q3;        /* identity (1,0,0,0) until seeded */
  float    beta;                   /* base fusion gain (default 0.05) */
  bool     initialized;            /* seeded from first usable accel */
};

struct db_imu_s
{
  int      fd;
  bool     opened;
  bool     calibrated;

  /* Phase 2.5: per-axis gyro bias in x1000 fractional LSB (millis-LSB).
   *
   * Phase 3a (#147) introduced Madgwick world-vertical yaw extraction,
   * which projects X/Y gyro bias error into the published heading
   * (chip tilt ⇒ X/Y component bleeds into world Z via the rotation
   * matrix).  Phase 3b cold-start measurements showed -5000 mdegpm
   * drift at 30 °C falling to -2500 mdegpm at 34 °C (slope ≈ 625
   * mdegpm/°C, matching LSM6DSL ZRL datasheet), so X/Y bias also
   * drifts with chip warmup and the cal-time initial value cannot stay
   * accurate.  #150 extends the idle EMA to all 3 axes so the
   * estimator tracks warmup-induced X/Y drift in addition to Z.
   *
   * x1000 lets the idle EMA hold sub-LSB bias values: ±1 LSB at
   * FSR=1000 dps equals ±35 mdps ≈ ±2 deg/min, which would dominate
   * the static-drift budget if quantised to int LSB.  See
   * [[project_phase_2_5_plan]] Blocker 2.
   */

  int32_t  bias_lsb_x1000[3];
  uint32_t bias_idle_threshold_lsb;
  uint64_t bias_idle_streak_us;
  uint64_t bias_window_us;
  int64_t  bias_acc_lsb_x1000[3]; /* per-axis raw × 1000 summed during  */
                                  /* idle window (#150)                  */
  uint32_t bias_acc_count;

  /* Issue #139: dynamic gyro sensitivity tracking.  cur_fsr_gy_idx is
   * the enum value last seen on an incoming sample; when it changes,
   * integrate() rescales bias_lsb_x1000 / bias_idle_threshold_lsb to
   * preserve their physical meaning, then recalibrates the
   * accumulator.  gyro_mdps_num = cur_fsr_gy_dps * 35 is used in the
   * per-sample mdeg formula (lsb * num * dt_us / 1e9 → mdeg).  Both
   * 0 means "scale unknown, skip integration" — happens before the
   * first sample arrives.
   */

  uint8_t  cur_fsr_gy_idx;
  uint16_t cur_fsr_gy_dps;
  int32_t  gyro_mdps_num;

  /* Heading state, mdeg, signed.  In Phase 3a this is the unwrapped
   * world-vertical yaw extracted from the Madgwick quaternion (per
   * drain), not the raw ∫gz_corr dt of pre-Phase-3.  ±2^63 mdeg
   * headroom dwarfs any realistic continuous run.
   */

  int64_t  heading_mdeg;
  uint32_t last_sample_ts_us;     /* sensor_imu.timestamp (32-bit µs,
                                   * 71-min wrap); modular subtract
                                   * for dt in integrate(). */
  bool     last_sample_valid;     /* false until first sample seen
                                   * (replaces the old "0 means no
                                   * sample" sentinel that broke on
                                   * timestamps that happened to be
                                   * 0 after wrap). */

  /* Phase 3a Madgwick fusion + per-drain yaw extraction state. */

  struct db_madgwick_state_s madgwick;
  float    psi_prev;              /* yaw rad at last drain, valid
                                   * after psi_valid */
  bool     psi_valid;             /* psi_prev seeded */
  uint8_t  accel_fsr_match;       /* 1 = accel Tedaldi cal was applied
                                   * this sample (cal loaded AND
                                   * cal.fsr_xl_g == live driver FSR).
                                   * 0 = identity fallback either
                                   * because cal is not loaded OR the
                                   * live FSR drifted away from
                                   * cal.fsr_xl_g.  Madgwick still
                                   * runs with a roughly-correct
                                   * gravity direction in both cases
                                   * (identity branch uses the live
                                   * FSR for float scaling), but tilt
                                   * precision is degraded. */
  uint64_t last_drain_ok_us;      /* daemon-clock µs of the last drain
                                   * that processed ≥ 1 sample;
                                   * 0 before first ok drain */

  /* Diagnostics */

  uint32_t sample_count;
  uint32_t drop_count;

  /* Last sample's OUT_TEMP raw value.  Captured during drain so the
   * Section F `_imu drift` / `_imu show` verbs can report ambient
   * temperature without re-reading the sensor topic themselves.
   * LSM6DSL convention: T_c = 25 + raw / 256.
   */

  int16_t  last_temperature_raw;

  /* Offline calibration data (M_x1000 + bias initial values).  Loaded
   * from /mnt/flash/imu_cal.txt at open; falls back to Identity +
   * zero on any error so the daemon runs uncalibrated (= pre-Phase
   * 2.5 behaviour) instead of refusing to start.
   */

  struct db_imu_cal_s cal;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

int  db_imu_open(struct db_imu_s *im);
void db_imu_close(struct db_imu_s *im);
bool db_imu_is_open(const struct db_imu_s *im);

/* Per-tick: drain the uORB topic, step the Madgwick fusion per sample,
 * then extract+unwrap yaw once into im->heading_mdeg.  `now_us` is the
 * daemon's monotonic reference and is recorded in `last_drain_ok_us`
 * on success so db_imu_is_stale() can detect bus stalls.  Inter-sample
 * `dt` is computed from the per-sample sensor_imu.timestamp (ISR-
 * captured, modular subtract for wrap-safety) so the fusion step is
 * independent of tick scheduling jitter.  Returns 0 or a negated errno
 * (-EAGAIN if no fresh sample arrived this tick — heading and stale
 * timestamp stay the same).
 */

int  db_imu_drain_and_update(struct db_imu_s *im, uint64_t now_us);

/* Stale detector for the Phase 3b PID injection guard.  Returns true
 * when no drain has produced ≥ 1 sample within the last
 * `threshold_us` daemon-clock micro-seconds, or when the IMU has
 * never produced a sample.  Use DB_IMU_DEFAULT_STALE_THRESHOLD_US
 * unless the caller has a reason to override.
 */

bool db_imu_is_stale(const struct db_imu_s *im, uint64_t now_us,
                     uint32_t threshold_us);

/* Madgwick diagnostic accessors.  Both are safe to call before the
 * first sample arrives (quaternion is (1,0,0,0) identity at open and
 * tilt_deg returns 0).  Float is intentional — these are CLI / test
 * scope, the PID path stays on int64.  `q[4]` is (w,x,y,z) row order.
 */

void  db_imu_get_quaternion(const struct db_imu_s *im, float q[4]);
float db_imu_get_tilt_deg(const struct db_imu_s *im);
float db_imu_get_beta(const struct db_imu_s *im);

/* Callback alternative for hot paths that already have one IMU
 * sample in hand (push API the plan describes — see
 * drivebase_imu.h notes).  Raw int16 values, accel passed through to
 * the Madgwick stage.
 */

void db_imu_push_sample(struct db_imu_s *im,
                        int16_t gx, int16_t gy, int16_t gz,
                        int16_t ax, int16_t ay, int16_t az,
                        uint8_t fsr_xl_idx, uint32_t ts_us);

/* Heading accessors */

int64_t db_imu_get_heading_mdeg(const struct db_imu_s *im);
void    db_imu_set_heading_mdeg(struct db_imu_s *im, int64_t heading);

/* Bias diagnostics.  *_x1000 is the underlying x1000 fractional
 * storage; the plain accessor truncates to int LSB for
 * backward-compatible CLI / test prints.
 */

int32_t db_imu_get_bias_z_lsb(const struct db_imu_s *im);
int32_t db_imu_get_bias_z_lsb_x1000(const struct db_imu_s *im);
bool    db_imu_is_calibrated(const struct db_imu_s *im);

/* Last drained sample's OUT_TEMP raw value (T_c = 25 + raw/256).  0
 * before the first drain — callers should check sample_count first
 * if they need a meaningful value.
 */

int16_t db_imu_get_temperature_raw(const struct db_imu_s *im);

/* Force-trigger calibration: hold the robot still for ~200 ms after
 * calling, then check db_imu_is_calibrated().  Internally just
 * resets the running average; a few hundred samples of "near-zero"
 * gyro will set calibrated=true.
 */

void db_imu_recalibrate(struct db_imu_s *im);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_IMU_H */
