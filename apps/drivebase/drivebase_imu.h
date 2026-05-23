/****************************************************************************
 * apps/drivebase/drivebase_imu.h
 *
 * IMU integration for the drivebase daemon (Issue #77 commit #10).
 * Drains LSM6DSL samples from /dev/uorb/sensor_imu0, runs a Z-axis
 * gyro integrator with bias compensation, and exposes
 *   db_imu_get_heading_mdeg() / set_heading() for the drivebase
 *   aggregator's optional gyro-locked heading mode.
 *
 * Only the 1D heading mode (z-only, complementary bias estimate) is
 * implemented in this commit; the 3D Madgwick variant the plan
 * describes lands in a follow-up once the bench need is established.
 * The btsensor / ImuViewer Madgwick (Issue #61/#64) is the eventual
 * port target.
 *
 * Coupled-loop wiring into drivebase_drivebase (heading_actual = gyro
 * instead of L−R / axle) is a single-flag change that the daemon FSM
 * (commit #11) will flip from the SET_USE_GYRO ioctl.
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

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_imu_s
{
  int      fd;
  bool     opened;
  bool     calibrated;

  /* Phase 2.5: per-axis gyro bias in x1000 fractional LSB (millis-LSB).
   *
   * Index 2 (Z) is the only axis the runtime idle estimator updates,
   * since the heading integrator consumes only the Z component.  X
   * and Y stay at the cal-file initial value (or zero when no cal is
   * loaded) plus FSR rescale — they contribute to corrected Z only
   * through the off-diagonal entries M_x1000[2][0..1], which are
   * typically <0.5 % of the diagonal so a few-LSB X/Y bias drift is
   * sub-mdeg/s on Z.
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
  int64_t  bias_acc_lsb_x1000;    /* Z-axis raw × 1000 summed during idle */
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

  /* Heading state, mdeg, signed.  Wraps at ±2^31 mdeg = ±5.96 M deg
   * which is well past any realistic continuous run.
   */

  int64_t  heading_mdeg;
  uint64_t last_sample_ts_us;

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

/* Per-tick: drain the topic, integrate Δheading, update bias.
 * `now_us` is the daemon's own monotonic reference; `dt_us` between
 * samples is taken from the LUMP frame timestamp so the integrator
 * is independent of tick scheduling jitter.  Returns 0 or a negated
 * errno (-EAGAIN if no fresh sample arrived this tick — heading
 * stays the same).
 */

int  db_imu_drain_and_update(struct db_imu_s *im, uint64_t now_us);

/* Callback alternative for hot paths that already have one IMU
 * sample in hand (push API the plan describes — see
 * drivebase_imu.h notes).  `g[3]` and `a[3]` are raw int16 values.
 */

void db_imu_push_sample(struct db_imu_s *im,
                        int16_t gx, int16_t gy, int16_t gz,
                        int16_t ax, int16_t ay, int16_t az,
                        uint64_t ts_us);

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
