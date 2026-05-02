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

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LSM6DSL default sensitivity at FS=2000 dps:  70 mdps per LSB.       */

#define DB_IMU_GYRO_LSB_TO_MDPS    70
#define DB_IMU_DRAIN_BATCH         16

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_imu_s
{
  int      fd;
  bool     opened;
  bool     calibrated;

  /* Z-axis bias.  Estimated as a running average of gz when |gyro|
   * stays below `bias_idle_threshold_lsb` for `bias_window_us`
   * continuously.  Stored in raw LSB so the per-sample correction is
   * a single subtraction.
   */

  int32_t  bias_z_lsb;
  uint32_t bias_idle_threshold_lsb;
  uint64_t bias_idle_streak_us;
  uint64_t bias_window_us;
  int64_t  bias_acc_lsb;
  uint32_t bias_acc_count;

  /* Heading state, mdeg, signed.  Wraps at ±2^31 mdeg = ±5.96 M deg
   * which is well past any realistic continuous run.
   */

  int64_t  heading_mdeg;
  uint64_t last_sample_ts_us;

  /* Diagnostics */

  uint32_t sample_count;
  uint32_t drop_count;
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

/* Bias diagnostics */

int32_t db_imu_get_bias_z_lsb(const struct db_imu_s *im);
bool    db_imu_is_calibrated(const struct db_imu_s *im);

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
