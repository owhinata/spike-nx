/****************************************************************************
 * apps/imu/imu_fusion.h
 *
 * Sensor fusion for IMU processing.
 * Ported from pybricks imu.c (MIT License, Pybricks Authors).
 ****************************************************************************/

#ifndef __APPS_IMU_FUSION_H
#define __APPS_IMU_FUSION_H

#include "imu_types.h"

void imu_fusion_init(void);
void imu_fusion_set_settings(imu_settings_t *settings);
void imu_fusion_set_base_orientation(imu_xyz_t *front, imu_xyz_t *top);

/* Called per-sample with gyro (deg/s) + accel (mm/s^2) */

void imu_fusion_update(imu_xyz_t *gyro_dps, imu_xyz_t *accel_mms2,
                       float sample_time);

/* Called when stationary period detected */

void imu_fusion_stationary_update(const int32_t *gyro_sum,
                                  const int32_t *accel_sum,
                                  uint32_t num_samples,
                                  float gyro_scale);

/* Getters */

void imu_fusion_get_accel(imu_xyz_t *out, bool calibrated);
void imu_fusion_get_gyro(imu_xyz_t *out, bool calibrated);
void imu_fusion_get_tilt(imu_xyz_t *out);
imu_side_t imu_fusion_get_up_side(bool calibrated);
int imu_fusion_get_single_axis_rotation(imu_xyz_t *axis, float *angle,
                                        bool calibrated);
float imu_fusion_get_heading(imu_heading_type_t type);
void imu_fusion_set_heading(float desired);
void imu_fusion_get_orientation(imu_matrix_3x3_t *out);
bool imu_fusion_is_ready(void);

#endif /* __APPS_IMU_FUSION_H */
