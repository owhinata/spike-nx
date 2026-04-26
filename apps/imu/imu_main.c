/****************************************************************************
 * apps/imu/imu_main.c
 *
 * NuttX NSH builtin application for IMU sensor fusion.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>
#include <math.h>

#include <nuttx/sensors/sensor.h>

#include <arch/board/board_lsm6dsl.h>

#include "imu_types.h"
#include "imu_stationary.h"
#include "imu_fusion.h"
#include "imu_calibration.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IMU_CAL_PATH      "/data/imu_cal.bin"
#define IMU_DAEMON_NAME   "imu_daemon"
#define IMU_POLL_TIMEOUT  1000
#define IMU_ODR           833

/* Driver startup defaults; matches DEFAULT_FSR_XL / DEFAULT_FSR_GY in
 * boards/spike-prime-hub/src/lsm6dsl_uorb.c.  The imu app does not
 * reconfigure FSR, so these are sufficient.
 */

#define IMU_ACCEL_FSR_G    8
#define IMU_GYRO_FSR_DPS   2000

#define IMU_GRAVITY_MS2    9.80665f

/* Scale factors for FSR -> physical units.  Per-LSB values.
 *
 *   accel_lsb_to_mms2(fsr_g):
 *     LSM6DSL accel sensitivity is fsr_g * 1000 / 32768 mg/LSB (the FSR
 *     label maps exactly to int16 full scale: e.g. ±8g -> 0.244 mg/LSB),
 *     converted to mm/s^2 with g = 9.80665.
 *
 *   gyro_lsb_to_dps(fsr_dps):
 *     LSM6DSL gyro sensitivity (datasheet Table 3) carries ~14.7%
 *     headroom over the nominal FSR, so the per-LSB value is *not*
 *     simply fsr/32768.  All five FSR rows scale linearly at 0.035
 *     mdps/LSB per nominal dps:
 *        ±125  -> 4.375 mdps/LSB
 *        ±250  -> 8.75  mdps/LSB
 *        ±500  -> 17.5  mdps/LSB
 *        ±1000 -> 35.0  mdps/LSB
 *        ±2000 -> 70.0  mdps/LSB
 */

static inline float accel_lsb_to_mms2(uint32_t fsr_g)
{
  return ((float)fsr_g * IMU_GRAVITY_MS2 * 1000.0f) / 32768.0f;
}

static inline float gyro_lsb_to_dps(uint32_t fsr_dps)
{
  return ((float)fsr_dps * 0.035f) / 1000.0f;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_daemon_running = false;
static volatile bool g_daemon_stop = false;
static int g_daemon_pid = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Cached scale factors so the stationary callback can convert the raw
 * gyro sum it receives back into deg/s without holding a per-call FSR.
 * Driver does not change FSR while the imu app is running.
 */

static float g_accel_mms2_per_lsb;
static float g_gyro_dps_per_lsb;

static void stationary_callback(const int32_t *gyro_sum,
                                const int32_t *accel_sum,
                                uint32_t num_samples)
{
  imu_fusion_stationary_update(gyro_sum, accel_sum, num_samples,
                               g_gyro_dps_per_lsb);
}

static int imu_daemon(int argc, char *argv[])
{
  struct pollfd fds[1];
  struct sensor_imu imu_data;
  int imu_fd;
  int ret;

  /* Open the combined IMU topic. */

  imu_fd = open("/dev/uorb/sensor_imu0", O_RDONLY | O_NONBLOCK);
  if (imu_fd < 0)
    {
      fprintf(stderr, "imu: cannot open sensor_imu0\n");
      g_daemon_running = false;
      return -1;
    }

  /* Compute scale factors from the driver's startup FSR. */

  g_accel_mms2_per_lsb = accel_lsb_to_mms2(IMU_ACCEL_FSR_G);
  g_gyro_dps_per_lsb   = gyro_lsb_to_dps(IMU_GYRO_FSR_DPS);

  /* Initialize modules */

  imu_settings_t *settings = imu_calibration_get_settings();
  imu_calibration_init(settings);
  imu_calibration_load(IMU_CAL_PATH);
  imu_fusion_init();
  imu_fusion_set_settings(settings);
  imu_stationary_init(settings->gyro_stationary_threshold /
                      g_gyro_dps_per_lsb,
                      settings->accel_stationary_threshold /
                      g_accel_mms2_per_lsb,
                      IMU_ODR, stationary_callback);

  g_daemon_running = true;
  g_daemon_stop = false;

  printf("imu: daemon started\n");

  /* Poll loop */

  while (!g_daemon_stop)
    {
      fds[0].fd = imu_fd;
      fds[0].events = POLLIN;

      ret = poll(fds, 1, IMU_POLL_TIMEOUT);
      if (ret < 0)
        {
          break;
        }

      if (ret == 0)
        {
          continue;
        }

      if (!(fds[0].revents & POLLIN))
        {
          continue;
        }

      while (read(imu_fd, &imu_data, sizeof(imu_data)) ==
             sizeof(imu_data))
        {
          int16_t raw[6];
          raw[0] = imu_data.gx;
          raw[1] = imu_data.gy;
          raw[2] = imu_data.gz;
          raw[3] = imu_data.ax;
          raw[4] = imu_data.ay;
          raw[5] = imu_data.az;

          imu_stationary_update(raw);

          imu_xyz_t accel_mms2 =
          {
            .x = (float)imu_data.ax * g_accel_mms2_per_lsb,
            .y = (float)imu_data.ay * g_accel_mms2_per_lsb,
            .z = (float)imu_data.az * g_accel_mms2_per_lsb,
          };

          imu_xyz_t gyro_dps =
          {
            .x = (float)imu_data.gx * g_gyro_dps_per_lsb,
            .y = (float)imu_data.gy * g_gyro_dps_per_lsb,
            .z = (float)imu_data.gz * g_gyro_dps_per_lsb,
          };

          float st = imu_stationary_get_sample_time();
          imu_fusion_update(&gyro_dps, &accel_mms2, st);
        }
    }

  close(imu_fd);

  /* Reset module state so status reports clean after stop */

  imu_stationary_init(0, 0, 0, NULL);
  imu_fusion_init();

  g_daemon_running = false;
  printf("imu: daemon stopped\n");
  return 0;
}

static void cmd_start(void)
{
  if (g_daemon_running)
    {
      printf("imu: already running\n");
      return;
    }

  g_daemon_pid = task_create(IMU_DAEMON_NAME, 100,
                             4096,
                             imu_daemon, NULL);
  if (g_daemon_pid < 0)
    {
      fprintf(stderr, "imu: failed to start daemon\n");
    }
}

static void cmd_stop(void)
{
  if (!g_daemon_running)
    {
      printf("imu: not running\n");
      return;
    }

  g_daemon_stop = true;
}

static void cmd_status(void)
{
  printf("running:    %s\n", g_daemon_running ? "yes" : "no");
  printf("stationary: %s\n",
         imu_stationary_is_stationary() ? "yes" : "no");
  printf("ready:      %s\n", imu_fusion_is_ready() ? "yes" : "no");
}

static void cmd_accel(bool raw)
{
  imu_xyz_t v;
  imu_fusion_get_accel(&v, !raw);
  printf("accel: x=%.1f y=%.1f z=%.1f mm/s^2\n", v.x, v.y, v.z);
}

static void cmd_gyro(bool raw)
{
  imu_xyz_t v;
  imu_fusion_get_gyro(&v, !raw);
  printf("gyro: x=%.3f y=%.3f z=%.3f deg/s\n", v.x, v.y, v.z);
}

/* Dump live IMU samples for `duration_ms` straight from the uORB topic.
 * `which` selects accel ('a') or gyro ('g'); `raw` prints chip-frame
 * int16 LSB instead of physical units.  Timestamp is the driver's
 * ISR-captured CLOCK_BOOTTIME us low 32 bits (mod 2^32, ~71m35s wrap).
 *
 * The IMU daemon must be running so the sensor is activated and samples
 * are flowing on the topic.
 */

static void cmd_dump(char which, bool raw, uint32_t duration_ms)
{
  if (!g_daemon_running)
    {
      fprintf(stderr, "imu: daemon not running; run 'imu start' first\n");
      return;
    }

  int fd = open("/dev/uorb/sensor_imu0", O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "imu: cannot open sensor_imu0\n");
      return;
    }

  /* Make sure scale factors reflect the driver's startup FSR; the
   * daemon also sets these during init but a stand-alone dump from a
   * fresh shell should still work.
   */

  if (g_accel_mms2_per_lsb == 0.0f)
    {
      g_accel_mms2_per_lsb = accel_lsb_to_mms2(IMU_ACCEL_FSR_G);
    }

  if (g_gyro_dps_per_lsb == 0.0f)
    {
      g_gyro_dps_per_lsb = gyro_lsb_to_dps(IMU_GYRO_FSR_DPS);
    }

  if (raw)
    {
      printf("# ts_us %s_x %s_y %s_z (raw int16)\n",
             which == 'a' ? "ax" : "gx",
             which == 'a' ? "ay" : "gy",
             which == 'a' ? "az" : "gz");
    }
  else
    {
      printf("# ts_us %s_x %s_y %s_z (%s)\n",
             which == 'a' ? "ax" : "gx",
             which == 'a' ? "ay" : "gy",
             which == 'a' ? "az" : "gz",
             which == 'a' ? "mm/s^2" : "deg/s");
    }

  struct timespec t0;
  clock_gettime(CLOCK_BOOTTIME, &t0);
  uint64_t deadline_us = (uint64_t)t0.tv_sec * 1000000ULL +
                         (uint64_t)t0.tv_nsec / 1000ULL +
                         (uint64_t)duration_ms * 1000ULL;

  uint32_t printed = 0;
  while (1)
    {
      struct timespec now;
      clock_gettime(CLOCK_BOOTTIME, &now);
      uint64_t now_us = (uint64_t)now.tv_sec * 1000000ULL +
                        (uint64_t)now.tv_nsec / 1000ULL;
      if (now_us >= deadline_us)
        {
          break;
        }

      struct pollfd pfd = { .fd = fd, .events = POLLIN };
      int remaining_ms = (int)((deadline_us - now_us) / 1000ULL);
      if (remaining_ms <= 0)
        {
          break;
        }

      int pret = poll(&pfd, 1, remaining_ms);
      if (pret <= 0)
        {
          continue;
        }

      struct sensor_imu s;
      while (read(fd, &s, sizeof(s)) == sizeof(s))
        {
          if (which == 'a')
            {
              if (raw)
                {
                  printf("%u %d %d %d\n", (unsigned)s.timestamp,
                         s.ax, s.ay, s.az);
                }
              else
                {
                  printf("%u %.3f %.3f %.3f\n", (unsigned)s.timestamp,
                         (double)((float)s.ax * g_accel_mms2_per_lsb),
                         (double)((float)s.ay * g_accel_mms2_per_lsb),
                         (double)((float)s.az * g_accel_mms2_per_lsb));
                }
            }
          else
            {
              if (raw)
                {
                  printf("%u %d %d %d\n", (unsigned)s.timestamp,
                         s.gx, s.gy, s.gz);
                }
              else
                {
                  printf("%u %.4f %.4f %.4f\n", (unsigned)s.timestamp,
                         (double)((float)s.gx * g_gyro_dps_per_lsb),
                         (double)((float)s.gy * g_gyro_dps_per_lsb),
                         (double)((float)s.gz * g_gyro_dps_per_lsb));
                }
            }

          printed++;
        }
    }

  close(fd);
  printf("# %u sample(s) over %u ms\n", (unsigned)printed,
         (unsigned)duration_ms);
}

static void cmd_mag(void)
{
  struct sensor_mag mag_data;
  int fd;

  fd = open("/dev/uorb/sensor_mag0", O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "imu: cannot open sensor_mag0\n");
      return;
    }

  if (read(fd, &mag_data, sizeof(mag_data)) == sizeof(mag_data))
    {
      printf("mag: x=%.3f y=%.3f z=%.3f uT\n",
             mag_data.x, mag_data.y, mag_data.z);
    }
  else
    {
      printf("mag: no data\n");
    }

  close(fd);
}

static void cmd_heading(imu_heading_type_t type)
{
  float h = imu_fusion_get_heading(type);
  printf("heading: %.2f deg (%s)\n", h,
         type == IMU_HEADING_1D ? "1D" : "3D");
}

static void cmd_tilt(void)
{
  imu_xyz_t v;
  imu_fusion_get_tilt(&v);
  printf("tilt: x=%.4f y=%.4f z=%.4f\n", v.x, v.y, v.z);
}

static void cmd_upside(void)
{
  imu_side_t side = imu_fusion_get_up_side(true);
  const char *names[] =
  {
    "front", "left", "top", "back", "right", "bottom"
  };

  /* Map enum to index:
   * FRONT=0, LEFT=1, TOP=2, BACK=4, RIGHT=5, BOTTOM=6
   */

  int idx;
  switch (side)
    {
      case IMU_SIDE_FRONT:  idx = 0; break;
      case IMU_SIDE_LEFT:   idx = 1; break;
      case IMU_SIDE_TOP:    idx = 2; break;
      case IMU_SIDE_BACK:   idx = 3; break;
      case IMU_SIDE_RIGHT:  idx = 4; break;
      case IMU_SIDE_BOTTOM: idx = 5; break;
      default:              idx = 2; break;
    }

  printf("up side: %s\n", names[idx]);
}

static void cmd_orient(void)
{
  imu_matrix_3x3_t m;
  imu_fusion_get_orientation(&m);
  printf("orientation:\n");
  printf("  [%.4f %.4f %.4f]\n", m.m11, m.m12, m.m13);
  printf("  [%.4f %.4f %.4f]\n", m.m21, m.m22, m.m23);
  printf("  [%.4f %.4f %.4f]\n", m.m31, m.m32, m.m33);
}

static void print_usage(void)
{
  printf("Usage: imu <command>\n");
  printf("Commands:\n");
  printf("  start        - start daemon\n");
  printf("  stop         - stop daemon\n");
  printf("  status       - print status\n");
  printf("  accel [raw] [ms] - print acceleration; with ms, dump samples\n");
  printf("  gyro  [raw] [ms] - print angular velocity; with ms, dump\n");
  printf("  mag          - print magnetic field\n");
  printf("  heading [1d|3d] - print heading\n");
  printf("  tilt         - print tilt vector\n");
  printf("  upside       - print which side is up\n");
  printf("  orient       - print rotation matrix\n");
  printf("  cal save     - save calibration\n");
  printf("  cal load     - load calibration\n");
  printf("  setbase      - set base orientation\n");
  printf("  resetheading - reset heading to 0\n");
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      print_usage();
      return 0;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      cmd_start();
    }
  else if (strcmp(argv[1], "stop") == 0)
    {
      cmd_stop();
    }
  else if (strcmp(argv[1], "status") == 0)
    {
      cmd_status();
    }
  else if (strcmp(argv[1], "accel") == 0)
    {
      bool raw = false;
      int dur_arg_idx = 2;
      if (argc > 2 && strcmp(argv[2], "raw") == 0)
        {
          raw = true;
          dur_arg_idx = 3;
        }

      if (argc > dur_arg_idx)
        {
          uint32_t ms = (uint32_t)strtoul(argv[dur_arg_idx], NULL, 10);
          cmd_dump('a', raw, ms);
        }
      else
        {
          cmd_accel(raw);
        }
    }
  else if (strcmp(argv[1], "gyro") == 0)
    {
      bool raw = false;
      int dur_arg_idx = 2;
      if (argc > 2 && strcmp(argv[2], "raw") == 0)
        {
          raw = true;
          dur_arg_idx = 3;
        }

      if (argc > dur_arg_idx)
        {
          uint32_t ms = (uint32_t)strtoul(argv[dur_arg_idx], NULL, 10);
          cmd_dump('g', raw, ms);
        }
      else
        {
          cmd_gyro(raw);
        }
    }
  else if (strcmp(argv[1], "mag") == 0)
    {
      cmd_mag();
    }
  else if (strcmp(argv[1], "heading") == 0)
    {
      imu_heading_type_t type = IMU_HEADING_3D;
      if (argc > 2 && strcmp(argv[2], "1d") == 0)
        {
          type = IMU_HEADING_1D;
        }

      cmd_heading(type);
    }
  else if (strcmp(argv[1], "tilt") == 0)
    {
      cmd_tilt();
    }
  else if (strcmp(argv[1], "upside") == 0)
    {
      cmd_upside();
    }
  else if (strcmp(argv[1], "orient") == 0)
    {
      cmd_orient();
    }
  else if (strcmp(argv[1], "cal") == 0)
    {
      if (argc < 3)
        {
          printf("Usage: imu cal <save|load>\n");
        }
      else if (strcmp(argv[2], "save") == 0)
        {
          if (imu_calibration_save(IMU_CAL_PATH) == 0)
            {
              printf("calibration saved\n");
            }
          else
            {
              fprintf(stderr, "imu: save failed\n");
            }
        }
      else if (strcmp(argv[2], "load") == 0)
        {
          if (imu_calibration_load(IMU_CAL_PATH) == 0)
            {
              imu_fusion_set_settings(imu_calibration_get_settings());
              printf("calibration loaded\n");
            }
          else
            {
              fprintf(stderr, "imu: load failed\n");
            }
        }
    }
  else if (strcmp(argv[1], "setbase") == 0)
    {
      imu_xyz_t front = { .x = 1.0f, .y = 0.0f, .z = 0.0f };
      imu_xyz_t top   = { .x = 0.0f, .y = 0.0f, .z = 1.0f };
      imu_fusion_set_base_orientation(&front, &top);
      printf("base orientation set to identity\n");
    }
  else if (strcmp(argv[1], "resetheading") == 0)
    {
      imu_fusion_set_heading(0.0f);
      printf("heading reset\n");
    }
  else
    {
      print_usage();
      return 1;
    }

  return 0;
}
