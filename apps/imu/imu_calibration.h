/****************************************************************************
 * apps/imu/imu_calibration.h
 *
 * Calibration persistence for IMU processing.
 * Ported from pybricks (MIT License, Pybricks Authors).
 ****************************************************************************/

#ifndef __APPS_IMU_CALIBRATION_H
#define __APPS_IMU_CALIBRATION_H

#include "imu_types.h"

void imu_calibration_init(imu_settings_t *settings);
void imu_calibration_set_defaults(imu_settings_t *settings);
int imu_calibration_save(const char *path);
int imu_calibration_load(const char *path);
imu_settings_t *imu_calibration_get_settings(void);

#endif /* __APPS_IMU_CALIBRATION_H */
