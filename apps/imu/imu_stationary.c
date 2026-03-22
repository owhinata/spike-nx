/****************************************************************************
 * apps/imu/imu_stationary.c
 *
 * Stationarity detection for IMU processing.
 * Ported from pybricks imu_lsm6ds3tr_c_stm32.c (MIT License,
 * Pybricks Authors).
 ****************************************************************************/

#include <string.h>
#include <nuttx/clock.h>

#include "imu_stationary.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct imu_stationary_s
{
  /* Slow moving average state */

  int32_t slow_sum[6];
  int16_t slow_avg[6];
  int32_t slow_count;

  /* Stationary detection state */

  int16_t start_data[6];
  int32_t gyro_sum[3];
  int32_t accel_sum[3];
  uint32_t sample_count;
  uint32_t time_start_us;
  bool stationary;

  /* Configuration */

  int16_t gyro_threshold;
  int16_t accel_threshold;
  uint32_t odr;
  float sample_time;

  /* Callback */

  imu_stationary_cb_t cb;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct imu_stationary_s g_stat;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline bool is_bounded(int16_t diff, int16_t threshold)
{
  return diff < threshold && diff > -threshold;
}

static uint32_t get_time_us(void)
{
  return (uint32_t)(clock_systime_ticks() * USEC_PER_TICK);
}

static void reset_buffer(void)
{
  g_stat.sample_count = 0;
  g_stat.time_start_us = get_time_us();
  memset(g_stat.gyro_sum, 0, sizeof(g_stat.gyro_sum));
  memset(g_stat.accel_sum, 0, sizeof(g_stat.accel_sum));
}

static void update_slow_average(const int16_t *data)
{
  int i;

  for (i = 0; i < 6; i++)
    {
      g_stat.slow_sum[i] += data[i];
    }

  g_stat.slow_count++;

  if (g_stat.slow_count == 125)
    {
      for (i = 0; i < 6; i++)
        {
          g_stat.slow_avg[i] = g_stat.slow_sum[i] / g_stat.slow_count;
          g_stat.slow_sum[i] = 0;
        }

      g_stat.slow_count = 0;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void imu_stationary_init(float gyro_threshold, float accel_threshold,
                         uint32_t odr, imu_stationary_cb_t cb)
{
  memset(&g_stat, 0, sizeof(g_stat));
  g_stat.gyro_threshold = (int16_t)gyro_threshold;
  g_stat.accel_threshold = (int16_t)accel_threshold;
  g_stat.odr = odr;
  g_stat.sample_time = 1.0f / odr;
  g_stat.cb = cb;
  g_stat.time_start_us = get_time_us();
}

void imu_stationary_set_thresholds(float gyro_thresh, float accel_thresh)
{
  g_stat.gyro_threshold = (int16_t)gyro_thresh;
  g_stat.accel_threshold = (int16_t)accel_thresh;
}

void imu_stationary_update(const int16_t *data)
{
  /* Update slow moving average */

  update_slow_average(data);

  /* Check whether still stationary compared to start sample */

  if (!is_bounded(data[0] - g_stat.start_data[0], g_stat.gyro_threshold) ||
      !is_bounded(data[1] - g_stat.start_data[1], g_stat.gyro_threshold) ||
      !is_bounded(data[2] - g_stat.start_data[2], g_stat.gyro_threshold) ||
      !is_bounded(data[3] - g_stat.start_data[3], g_stat.accel_threshold) ||
      !is_bounded(data[4] - g_stat.start_data[4], g_stat.accel_threshold) ||
      !is_bounded(data[5] - g_stat.start_data[5], g_stat.accel_threshold))
    {
      /* Not stationary - slow average becomes new start value */

      g_stat.stationary = false;
      memcpy(g_stat.start_data, g_stat.slow_avg, sizeof(g_stat.start_data));
      reset_buffer();
      return;
    }

  /* Accumulate stationary sums */

  g_stat.sample_count++;
  g_stat.gyro_sum[0] += data[0];
  g_stat.gyro_sum[1] += data[1];
  g_stat.gyro_sum[2] += data[2];
  g_stat.accel_sum[0] += data[3];
  g_stat.accel_sum[1] += data[4];
  g_stat.accel_sum[2] += data[5];

  /* Not enough samples yet */

  if (g_stat.sample_count < g_stat.odr)
    {
      return;
    }

  /* Stationary period complete */

  g_stat.stationary = true;

  /* Measure actual sample time from elapsed time */

  uint32_t elapsed = get_time_us() - g_stat.time_start_us;
  g_stat.sample_time = elapsed / 1000000.0f / g_stat.sample_count;

  /* Call callback with accumulated data */

  if (g_stat.cb)
    {
      g_stat.cb(g_stat.gyro_sum, g_stat.accel_sum, g_stat.sample_count);
    }

  /* Reset for next period */

  reset_buffer();
}

bool imu_stationary_is_stationary(void)
{
  return g_stat.stationary;
}

float imu_stationary_get_sample_time(void)
{
  return g_stat.sample_time;
}
