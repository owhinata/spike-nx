/****************************************************************************
 * apps/imu/imu_stationary.h
 *
 * Stationarity detection for IMU processing.
 * Ported from pybricks imu_lsm6ds3tr_c_stm32.c (MIT License,
 * Pybricks Authors).
 ****************************************************************************/

#ifndef __APPS_IMU_STATIONARY_H
#define __APPS_IMU_STATIONARY_H

#include "imu_types.h"

/* Callback when stationary period detected */

typedef void (*imu_stationary_cb_t)(const int32_t *gyro_sum,
                                    const int32_t *accel_sum,
                                    uint32_t num_samples);

void imu_stationary_init(float gyro_threshold, float accel_threshold,
                         uint32_t odr, imu_stationary_cb_t cb);
void imu_stationary_set_thresholds(float gyro_thresh, float accel_thresh);
void imu_stationary_update(const int16_t *data);
bool imu_stationary_is_stationary(void);
float imu_stationary_get_sample_time(void);

/* Issue #139: drop all raw accumulators (sample_count, gyro/accel sums,
 * slow-average sums, start-data) without disturbing the configured
 * thresholds / ODR / callback.  Call this when the driver FSR (and
 * hence the raw-LSB scale of incoming samples) changes mid-stream, so
 * the next stationary window starts fresh instead of mixing raw values
 * collected at the old scale with new-scale samples.
 */

void imu_stationary_reset(void);

#endif /* __APPS_IMU_STATIONARY_H */
