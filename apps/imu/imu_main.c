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

/* uORB sensor scale factors (chip configured to 8g / 2000dps) */

#define IMU_GYRO_SCALE    0.070f   /* mdps/LSB -> deg/s = raw * 0.070 */
#define IMU_ACCEL_SCALE   0.244f   /* mg/LSB -> mm/s^2 = raw * 0.244 * 9.81 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static volatile bool g_daemon_running = false;
static volatile bool g_daemon_stop = false;
static int g_daemon_pid = -1;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void stationary_callback(const int32_t *gyro_sum,
                                const int32_t *accel_sum,
                                uint32_t num_samples)
{
  imu_fusion_stationary_update(gyro_sum, accel_sum, num_samples,
                               IMU_GYRO_SCALE);
}

static int imu_daemon(int argc, char *argv[])
{
  struct pollfd fds[2];
  struct sensor_accel accel_data;
  struct sensor_gyro gyro_data;
  int accel_fd;
  int gyro_fd;
  int ret;

  /* Open sensor devices */

  accel_fd = open("/dev/uorb/sensor_accel0", O_RDONLY | O_NONBLOCK);
  if (accel_fd < 0)
    {
      fprintf(stderr, "imu: cannot open sensor_accel0\n");
      g_daemon_running = false;
      return -1;
    }

  gyro_fd = open("/dev/uorb/sensor_gyro0", O_RDONLY | O_NONBLOCK);
  if (gyro_fd < 0)
    {
      fprintf(stderr, "imu: cannot open sensor_gyro0\n");
      close(accel_fd);
      g_daemon_running = false;
      return -1;
    }

  /* Initialize modules */

  imu_settings_t *settings = imu_calibration_get_settings();
  imu_calibration_init(settings);
  imu_calibration_load(IMU_CAL_PATH);
  imu_fusion_init();
  imu_fusion_set_settings(settings);
  imu_stationary_init(settings->gyro_stationary_threshold / IMU_GYRO_SCALE,
                      settings->accel_stationary_threshold /
                      (IMU_ACCEL_SCALE * 9.81f),
                      IMU_ODR, stationary_callback);

  g_daemon_running = true;
  g_daemon_stop = false;

  printf("imu: daemon started\n");

  /* Poll loop */

  while (!g_daemon_stop)
    {
      fds[0].fd = accel_fd;
      fds[0].events = POLLIN;
      fds[1].fd = gyro_fd;
      fds[1].events = POLLIN;

      ret = poll(fds, 2, IMU_POLL_TIMEOUT);
      if (ret < 0)
        {
          break;
        }

      if (ret == 0)
        {
          continue;
        }

      bool have_accel = false;
      bool have_gyro = false;

      if (fds[0].revents & POLLIN)
        {
          if (read(accel_fd, &accel_data, sizeof(accel_data)) ==
              sizeof(accel_data))
            {
              have_accel = true;
            }
        }

      if (fds[1].revents & POLLIN)
        {
          if (read(gyro_fd, &gyro_data, sizeof(gyro_data)) ==
              sizeof(gyro_data))
            {
              have_gyro = true;
            }
        }

      if (have_accel && have_gyro)
        {
          /* Build raw int16 array for stationary detector:
           * data[0..2] = gyro XYZ raw, data[3..5] = accel XYZ raw
           * uORB gyro is rad/s, uORB accel is m/s^2
           * Convert back to raw LSB for stationary detection
           */

          int16_t raw[6];
          raw[0] = (int16_t)(gyro_data.x * IMU_RAD_TO_DEG / IMU_GYRO_SCALE);
          raw[1] = (int16_t)(gyro_data.y * IMU_RAD_TO_DEG / IMU_GYRO_SCALE);
          raw[2] = (int16_t)(gyro_data.z * IMU_RAD_TO_DEG / IMU_GYRO_SCALE);
          raw[3] = (int16_t)(accel_data.x * 1000.0f /
                             (IMU_ACCEL_SCALE * 9.81f));
          raw[4] = (int16_t)(accel_data.y * 1000.0f /
                             (IMU_ACCEL_SCALE * 9.81f));
          raw[5] = (int16_t)(accel_data.z * 1000.0f /
                             (IMU_ACCEL_SCALE * 9.81f));

          imu_stationary_update(raw);

          /* Convert to physical units for fusion:
           * accel: m/s^2 -> mm/s^2 (* 1000)
           * gyro:  rad/s -> deg/s  (* RAD_TO_DEG)
           */

          imu_xyz_t accel_mms2 =
          {
            .x = accel_data.x * 1000.0f,
            .y = accel_data.y * 1000.0f,
            .z = accel_data.z * 1000.0f,
          };

          imu_xyz_t gyro_dps =
          {
            .x = gyro_data.x * IMU_RAD_TO_DEG,
            .y = gyro_data.y * IMU_RAD_TO_DEG,
            .z = gyro_data.z * IMU_RAD_TO_DEG,
          };

          float st = imu_stationary_get_sample_time();
          imu_fusion_update(&gyro_dps, &accel_mms2, st);
        }
    }

  close(accel_fd);
  close(gyro_fd);

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
                             CONFIG_DEFAULT_TASK_STACKSIZE,
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
  printf("  accel [raw]  - print acceleration\n");
  printf("  gyro [raw]   - print angular velocity\n");
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
      bool raw = (argc > 2 && strcmp(argv[2], "raw") == 0);
      cmd_accel(raw);
    }
  else if (strcmp(argv[1], "gyro") == 0)
    {
      bool raw = (argc > 2 && strcmp(argv[2], "raw") == 0);
      cmd_gyro(raw);
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
