/****************************************************************************
 * apps/imu/imu_calibration.c
 *
 * Calibration persistence for IMU processing.
 * Ported from pybricks (MIT License, Pybricks Authors).
 ****************************************************************************/

#include <fcntl.h>
#include <unistd.h>
#include <string.h>

#include "imu_calibration.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static imu_settings_t g_settings;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imu_calibration_set_defaults(imu_settings_t *settings)
{
  settings->flags = 0;
  settings->gyro_stationary_threshold = 2.0f;
  settings->accel_stationary_threshold = 2500.0f;

  settings->gravity_pos.x = IMU_STANDARD_GRAVITY;
  settings->gravity_pos.y = IMU_STANDARD_GRAVITY;
  settings->gravity_pos.z = IMU_STANDARD_GRAVITY;

  settings->gravity_neg.x = -IMU_STANDARD_GRAVITY;
  settings->gravity_neg.y = -IMU_STANDARD_GRAVITY;
  settings->gravity_neg.z = -IMU_STANDARD_GRAVITY;

  settings->angular_velocity_bias_start.x = 0.0f;
  settings->angular_velocity_bias_start.y = 0.0f;
  settings->angular_velocity_bias_start.z = 0.0f;

  settings->angular_velocity_scale.x = 360.0f;
  settings->angular_velocity_scale.y = 360.0f;
  settings->angular_velocity_scale.z = 360.0f;

  settings->heading_correction_1d = 360.0f;
}

void imu_calibration_init(imu_settings_t *settings)
{
  imu_calibration_set_defaults(settings);
}

int imu_calibration_save(const char *path)
{
  int fd;
  ssize_t ret;

  fd = open(path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
  if (fd < 0)
    {
      return -1;
    }

  ret = write(fd, &g_settings, sizeof(g_settings));
  close(fd);

  if (ret != (ssize_t)sizeof(g_settings))
    {
      return -1;
    }

  return 0;
}

int imu_calibration_load(const char *path)
{
  int fd;
  ssize_t ret;
  imu_settings_t tmp;

  fd = open(path, O_RDONLY);
  if (fd < 0)
    {
      return -1;
    }

  ret = read(fd, &tmp, sizeof(tmp));
  close(fd);

  if (ret != (ssize_t)sizeof(tmp))
    {
      return -1;
    }

  g_settings = tmp;
  return 0;
}

imu_settings_t *imu_calibration_get_settings(void)
{
  return &g_settings;
}
