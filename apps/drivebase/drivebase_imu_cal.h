/****************************************************************************
 * apps/drivebase/drivebase_imu_cal.h
 *
 * Offline IMU calibration loader (Phase 2.5, Issue #145).
 *
 * Reads the Tedaldi (imu_tk) calibration result from a fixed path
 * (/mnt/flash/imu_cal.txt), parses a tiny key=value properties format,
 * and hands a populated struct db_imu_cal_s to drivebase_imu so the
 * per-sample matmul can dimensionalise raw int16 LSB into corrected
 * units.
 *
 * The on-device math is integer x1000 fixed-point throughout (see
 * [[project_phase_2_5_plan]] for the rationale): bias_lsb_x1000 keeps
 * fractional bias values so the 1 LSB quantisation drift (~1 deg/min
 * at FS=1000 dps) does not dominate the runtime integrator, and
 * M_x1000 keeps the misalignment+scale matrix as dimensionless x1000
 * integers near 1000 on the diagonal.
 *
 * On any load failure (file absent / parse error / schema mismatch /
 * FSR mismatch) the loader leaves the cal at Identity + zero bias
 * and sets loaded=false; the daemon then runs uncalibrated, identical
 * to the pre-Phase 2.5 path.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_IMU_CAL_H
#define __APPS_DRIVEBASE_DRIVEBASE_IMU_CAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_IMU_CAL_PATH             "/mnt/flash/imu_cal.txt"
#define DB_IMU_CAL_SCHEMA_VERSION   1

/****************************************************************************
 * Types
 ****************************************************************************/

/* All matrices are row-major.  Bias is per-axis (X, Y, Z) in raw-LSB
 * domain × 1000 (= millis-LSB).  M is dimensionless × 1000 (diagonal
 * near 1000 = sensor matches nominal sensitivity).  fsr_xl, fsr_gy
 * and odr_hz are the values used during the calibration session,
 * recorded so the loader can reject mismatches against the runtime
 * defaults.
 */

struct db_imu_cal_s
{
  int32_t  gyro_bias_lsb_x1000[3];
  int32_t  gyro_M_x1000[3][3];
  int32_t  accel_bias_lsb_x1000[3];
  int32_t  accel_M_x1000[3][3];
  uint16_t fsr_gy_dps;
  uint16_t odr_hz;
  uint8_t  fsr_xl_g;
  int8_t   ambient_temp_c;
  uint8_t  schema_version;
  bool     loaded;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Populate `cal` from /mnt/flash/imu_cal.txt.  Always returns a valid
 * struct: on any error the cal is reset to Identity + zero bias and
 * the function returns a negated errno (-ENOENT if the file is
 * absent, -EPROTO on schema mismatch, -EINVAL on parse / FSR
 * mismatch).  `cal->loaded` is true only when the on-disk file passed
 * every check.
 */

int  db_imu_cal_load(struct db_imu_cal_s *cal);

/* Reset cal to the no-op identity: M = Identity * 1000, bias = 0.
 * The runtime matmul with these values reproduces the pre-Phase 2.5
 * `g_corrected = g_raw - bias_z` behaviour bit-for-bit.
 */

void db_imu_cal_set_identity(struct db_imu_cal_s *cal);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_IMU_CAL_H */
