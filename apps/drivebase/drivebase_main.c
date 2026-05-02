/****************************************************************************
 * apps/drivebase/drivebase_main.c
 *
 * NSH CLI shell for the SPIKE Prime drivebase daemon (Issue #77).
 * This file is the user-facing front-end; the heavy lifting (RT control
 * loop, observer, trajectory, motor / IMU drain) lives in sibling
 * sources to be added in subsequent commits of the same Issue:
 *
 *   commit #4  drivebase_motor.c       LEGOSENSOR fd lifetime + actuation
 *   commit #5  drivebase_{angle,observer,control,trajectory,settings}.c
 *   commit #6  drivebase_servo.c       per-motor PID closing the loop
 *   commit #7  drivebase_drivebase.c   L/R aggregation + heading control
 *   commit #8  drivebase_rt.c          5 ms tick + jitter ring
 *   commit #9  drivebase_chardev_handler.c   command pickup / state publish
 *   commit #10 drivebase_imu.c         sensor_imu0 push + gyro heading
 *   commit #11 drivebase_daemon.c      lifecycle FSM + stall watchdog
 *
 * For the present commit (#3) the CLI only exposes `drivebase status`,
 * which exercises DRIVEBASE_GET_STATUS through the kernel chardev shim
 * and confirms the ABI plumbing end-to-end without requiring the daemon
 * to have attached yet.  All other subcommands print "not yet
 * implemented" so the usage page already tracks the planned surface.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/board/board_drivebase.h>

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int open_drivebase(void)
{
  int fd = open(DRIVEBASE_DEVPATH, O_RDWR);
  if (fd < 0)
    {
      fprintf(stderr, "open(%s): %s\n", DRIVEBASE_DEVPATH, strerror(errno));
    }
  return fd;
}

static int do_status(void)
{
  int fd = open_drivebase();
  if (fd < 0)
    {
      return 1;
    }

  struct drivebase_status_s st;
  memset(&st, 0, sizeof(st));

  int ret = ioctl(fd, DRIVEBASE_GET_STATUS, (unsigned long)&st);
  close(fd);

  if (ret < 0)
    {
      fprintf(stderr, "DRIVEBASE_GET_STATUS: %s\n", strerror(errno));
      return 1;
    }

  printf("daemon_attached  = %u\n", st.daemon_attached);
  printf("attach_generation= %lu\n", (unsigned long)st.attach_generation);
  printf("configured       = %u\n", st.configured);
  printf("motor_l_bound    = %u\n", st.motor_l_bound);
  printf("motor_r_bound    = %u\n", st.motor_r_bound);
  printf("imu_present      = %u\n", st.imu_present);
  printf("use_gyro         = %u\n", st.use_gyro);
  printf("tick_count       = %lu\n", (unsigned long)st.tick_count);
  printf("tick_overrun     = %lu\n", (unsigned long)st.tick_overrun_count);
  printf("tick_max_lag_us  = %lu\n", (unsigned long)st.tick_max_lag_us);
  printf("cmd_ring_depth   = %lu\n", (unsigned long)st.cmd_ring_depth);
  printf("cmd_drop_count   = %lu\n", (unsigned long)st.cmd_drop_count);
  printf("last_cmd_seq     = %lu\n", (unsigned long)st.last_cmd_seq);
  printf("last_pickup_us   = %lu\n", (unsigned long)st.last_pickup_us);
  printf("last_publish_us  = %lu\n", (unsigned long)st.last_publish_us);
  printf("encoder_drops    = %lu\n", (unsigned long)st.encoder_drop_count);

  return 0;
}

static void usage(void)
{
  fprintf(stderr,
          "usage:\n"
          "  drivebase                                   show this help\n"
          "  drivebase status                            DRIVEBASE_GET_STATUS snapshot\n"
          "  drivebase start                             [commit #11] launch daemon\n"
          "  drivebase stop                              [commit #11] teardown daemon\n"
          "  drivebase config <wheel_mm> <axle_mm>       [commit #9]  DRIVEBASE_CONFIG\n"
          "  drivebase straight <mm> [coast|brake|hold]  [commit #9]  DRIVE_STRAIGHT\n"
          "  drivebase turn <deg>                        [commit #9]  TURN\n"
          "  drivebase forever <speed_mmps> <turn_dps>   [commit #9]  DRIVE_FOREVER\n"
          "  drivebase stop-motion <coast|brake|hold>    [commit #9]  STOP\n"
          "  drivebase get-state                         [commit #9]  GET_STATE\n"
          "  drivebase set-gyro <none|1d|3d>             [commit #10] SET_USE_GYRO\n"
          "  drivebase jitter [reset]                    [commit #8]  JITTER_DUMP\n");
}

/****************************************************************************
 * Public Function: main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      usage();
      return 0;
    }

  const char *verb = argv[1];

  if (strcmp(verb, "status") == 0)
    {
      return do_status();
    }

  if (strcmp(verb, "help") == 0 ||
      strcmp(verb, "-h")   == 0 ||
      strcmp(verb, "--help") == 0)
    {
      usage();
      return 0;
    }

  /* All other verbs are wired in subsequent commits of Issue #77. */

  if (strcmp(verb, "start") == 0 || strcmp(verb, "stop") == 0 ||
      strcmp(verb, "config") == 0 || strcmp(verb, "straight") == 0 ||
      strcmp(verb, "turn") == 0 || strcmp(verb, "forever") == 0 ||
      strcmp(verb, "stop-motion") == 0 || strcmp(verb, "get-state") == 0 ||
      strcmp(verb, "set-gyro") == 0 || strcmp(verb, "jitter") == 0)
    {
      fprintf(stderr,
              "drivebase: '%s' not yet implemented (Issue #77 in progress)\n",
              verb);
      return 1;
    }

  fprintf(stderr, "drivebase: unknown verb '%s'\n", verb);
  usage();
  return 1;
}
