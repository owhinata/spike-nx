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

#include "drivebase_motor.h"
#include "drivebase_settings.h"
#include "drivebase_trajectory.h"
#include "drivebase_observer.h"
#include "drivebase_control.h"
#include "drivebase_angle.h"

/****************************************************************************
 * Private Functions: hidden _motor test verbs (Issue #77 development only)
 *
 * These verbs let us bench drivebase_motor.c primitives from NSH while
 * the surrounding daemon is being built up commit-by-commit.  They will
 * be removed once commit #11 lands the lifecycle FSM that owns init /
 * deinit and the drive verbs become the public surface.
 ****************************************************************************/

static int parse_side(const char *s, enum db_side_e *out)
{
  if ((s[0] == 'l' || s[0] == 'L') && s[1] == '\0')
    {
      *out = DB_SIDE_LEFT;
      return 0;
    }
  if ((s[0] == 'r' || s[0] == 'R') && s[1] == '\0')
    {
      *out = DB_SIDE_RIGHT;
      return 0;
    }
  return -1;
}

static int do_motor_subcmd(int argc, FAR char *argv[])
{
  /* Common: init both sides for every invocation, deinit at the end.
   * Fresh task per invocation = stateless from CLI's perspective.
   */

  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _motor "
              "{open|read [l|r]|select <l|r> <mode>|"
              "duty <l|r> <-10000..10000>|coast <l|r>|brake <l|r>}\n");
      return 1;
    }

  int rc = drivebase_motor_init();
  if (rc < 0)
    {
      fprintf(stderr, "drivebase_motor_init: %s\n", strerror(-rc));
      return 1;
    }

  const char *sub = argv[0];
  int ret = 0;

  if (strcmp(sub, "open") == 0)
    {
      printf("L: port_idx=%d (sensor_motor_l)\n",
             drivebase_motor_port_idx(DB_SIDE_LEFT));
      printf("R: port_idx=%d (sensor_motor_r)\n",
             drivebase_motor_port_idx(DB_SIDE_RIGHT));
    }
  else if (strcmp(sub, "read") == 0)
    {
      enum db_side_e sides[2] = { DB_SIDE_LEFT, DB_SIDE_RIGHT };
      int n_sides = 2;
      if (argc >= 2)
        {
          enum db_side_e s;
          if (parse_side(argv[1], &s) < 0)
            {
              fprintf(stderr, "bad side: %s\n", argv[1]);
              ret = 1;
              goto out;
            }
          sides[0] = s;
          n_sides  = 1;
        }
      for (int i = 0; i < n_sides; i++)
        {
          struct db_motor_sample_s sm;
          int dr;

          /* Retry drain for ~10 ms — the upper-half sensor framework
           * starts a fresh subscriber at "no data yet" and only fills
           * the per-fd cursor when a new publish arrives.  LUMP runs
           * ~1 kHz so 10 attempts × 1 ms is plenty.  The daemon does
           * not need this — it owns a long-lived fd that sees every
           * publish from the moment it subscribed.
           */

          for (int attempt = 0; attempt < 10; attempt++)
            {
              dr = drivebase_motor_drain(sides[i], &sm);
              if (dr != -EAGAIN)
                {
                  break;
                }
              usleep(1000);
            }

          const char *name = (sides[i] == DB_SIDE_LEFT) ? "L" : "R";
          if (dr == 0)
            {
              printf("%s port=%u mode=%u type=%u nval=%u "
                     "seq=%lu gen=%lu ts_us=%llu raw=%ld\n",
                     name, sm.port_idx, sm.mode_id, sm.data_type,
                     sm.num_values,
                     (unsigned long)sm.seq, (unsigned long)sm.generation,
                     (unsigned long long)sm.timestamp_us,
                     (long)sm.raw_value);
            }
          else
            {
              printf("%s drain: %s\n", name, strerror(-dr));
            }
        }
    }
  else if (strcmp(sub, "select") == 0)
    {
      enum db_side_e s;
      if (argc < 3 || parse_side(argv[1], &s) < 0)
        {
          fprintf(stderr, "usage: _motor select <l|r> <mode>\n");
          ret = 1;
          goto out;
        }
      int mode = atoi(argv[2]);
      int sr = drivebase_motor_select_mode(s, (uint8_t)mode);
      if (sr < 0)
        {
          fprintf(stderr, "select: %s\n", strerror(-sr));
          ret = 1;
        }
    }
  else if (strcmp(sub, "duty") == 0)
    {
      enum db_side_e s;
      if (argc < 3 || parse_side(argv[1], &s) < 0)
        {
          fprintf(stderr, "usage: _motor duty <l|r> <-10000..10000>\n");
          ret = 1;
          goto out;
        }
      int duty = atoi(argv[2]);
      if (duty < -10000 || duty > 10000)
        {
          fprintf(stderr, "duty out of range: %d\n", duty);
          ret = 1;
          goto out;
        }
      int sr = drivebase_motor_set_duty(s, (int16_t)duty);
      if (sr < 0)
        {
          fprintf(stderr, "set_duty: %s\n", strerror(-sr));
          ret = 1;
        }
    }
  else if (strcmp(sub, "coast") == 0)
    {
      enum db_side_e s;
      if (argc < 2 || parse_side(argv[1], &s) < 0)
        {
          fprintf(stderr, "usage: _motor coast <l|r>\n");
          ret = 1;
          goto out;
        }
      int sr = drivebase_motor_coast(s);
      if (sr < 0)
        {
          fprintf(stderr, "coast: %s\n", strerror(-sr));
          ret = 1;
        }
    }
  else if (strcmp(sub, "brake") == 0)
    {
      enum db_side_e s;
      if (argc < 2 || parse_side(argv[1], &s) < 0)
        {
          fprintf(stderr, "usage: _motor brake <l|r>\n");
          ret = 1;
          goto out;
        }
      int sr = drivebase_motor_brake(s);
      if (sr < 0)
        {
          fprintf(stderr, "brake: %s\n", strerror(-sr));
          ret = 1;
        }
    }
  else
    {
      fprintf(stderr, "_motor: unknown subcommand '%s'\n", sub);
      ret = 1;
    }

out:
  drivebase_motor_deinit();
  return ret;
}

/****************************************************************************
 * Private Functions: hidden _alg test verbs (Issue #77 development only)
 *
 * Sample the trapezoidal trajectory and the default settings tables
 * from NSH so commit #5 is verifiable without a host-side unit test.
 * Removed once the daemon's drive verbs subsume the surface in commit
 * #11.
 ****************************************************************************/

static int do_alg_traj(int argc, FAR char *argv[])
{
  if (argc < 5)
    {
      fprintf(stderr,
              "usage: drivebase _alg traj <x0_mdeg> <x1_mdeg> "
              "<vmax_mdegps> <accel_mdegps2> <decel_mdegps2>\n");
      return 1;
    }

  int64_t x0  = (int64_t)atoll(argv[0]);
  int64_t x1  = (int64_t)atoll(argv[1]);
  int32_t v   = (int32_t)atol(argv[2]);
  int32_t a   = (int32_t)atol(argv[3]);
  int32_t d   = (int32_t)atol(argv[4]);

  struct db_trajectory_s tr;
  db_trajectory_init_position(&tr, 0, x0, x1, v, a, d);

  printf("traj: dir=%d v_peak=%ld accel_dt=%llu cruise_dt=%llu "
         "decel_dt=%llu total=%llu us\n",
         tr.direction, (long)tr.v_peak_mdegps,
         (unsigned long long)tr.accel_dt_us,
         (unsigned long long)tr.cruise_dt_us,
         (unsigned long long)tr.decel_dt_us,
         (unsigned long long)tr.total_dt_us);
  printf("     x_accel_end=%lld x_cruise_end=%lld x1=%lld mdeg\n",
         (long long)tr.x_accel_end_mdeg,
         (long long)tr.x_cruise_end_mdeg,
         (long long)tr.x1_mdeg);

  /* Sample at 0%, 25%, 50%, 75%, 100%, 110% of total time */

  uint64_t pts[6];
  pts[0] = 0;
  pts[1] = tr.total_dt_us / 4;
  pts[2] = tr.total_dt_us / 2;
  pts[3] = (tr.total_dt_us * 3) / 4;
  pts[4] = tr.total_dt_us;
  pts[5] = tr.total_dt_us + tr.total_dt_us / 10;

  for (int i = 0; i < 6; i++)
    {
      struct db_trajectory_ref_s ref;
      db_trajectory_get_reference(&tr, pts[i], &ref);
      printf("  t=%llu us  x=%lld v=%ld a=%ld done=%d\n",
             (unsigned long long)pts[i],
             (long long)ref.x_mdeg, (long)ref.v_mdegps,
             (long)ref.a_mdegps2, (int)ref.done);
    }

  return 0;
}

static int do_alg_settings(int argc, FAR char *argv[])
{
  uint32_t wheel_d = (argc >= 1) ? (uint32_t)atoi(argv[0]) : 56;
  uint32_t axle_t  = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 100;

  const struct db_servo_gains_s        *g = db_settings_servo_gains();
  const struct db_traj_limits_s        *dl = db_settings_distance_limits(wheel_d);
  const struct db_traj_limits_s        *hl = db_settings_heading_limits(wheel_d, axle_t);
  const struct db_stall_settings_s     *st = db_settings_stall();
  const struct db_completion_settings_s *cm = db_settings_completion();

  printf("wheel_d=%lu mm  axle_t=%lu mm\n",
         (unsigned long)wheel_d, (unsigned long)axle_t);
  printf("servo: kp_pos=%ld ki_pos=%ld kd_pos=%ld "
         "kp_speed=%ld ki_speed=%ld deadband=%ld out=[%ld,%ld]\n",
         (long)g->kp_pos, (long)g->ki_pos, (long)g->kd_pos,
         (long)g->kp_speed, (long)g->ki_speed,
         (long)g->deadband_mdeg, (long)g->out_min, (long)g->out_max);
  printf("distance limits: v=%ld accel=%ld decel=%ld mdeg/s,/s/s\n",
         (long)dl->v_max_mdegps, (long)dl->accel_mdegps2,
         (long)dl->decel_mdegps2);
  printf("heading  limits: v=%ld accel=%ld decel=%ld mdeg/s,/s/s\n",
         (long)hl->v_max_mdegps, (long)hl->accel_mdegps2,
         (long)hl->decel_mdegps2);
  printf("stall: low_speed=%ld min_duty=%ld window=%lu ms\n",
         (long)st->stall_speed_mdegps, (long)st->stall_duty_min,
         (unsigned long)st->stall_window_ms);
  printf("completion: pos_tol=%ld speed_tol=%ld done_window=%lu ms "
         "smart_hold=%lu ms\n",
         (long)cm->pos_tolerance_mdeg,
         (long)cm->speed_tolerance_mdegps,
         (unsigned long)cm->done_window_ms,
         (unsigned long)cm->smart_passive_hold_ms);
  return 0;
}

static int do_alg_angle(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      fprintf(stderr,
              "usage: drivebase _alg angle <wheel_d_mm> <mdeg>\n");
      return 1;
    }
  uint32_t wheel_d = (uint32_t)atoi(argv[0]);
  int64_t mdeg     = (int64_t)atoll(argv[1]);
  int32_t mm       = db_angle_mdeg_to_mm(mdeg, wheel_d);
  int64_t back     = db_angle_mm_to_mdeg(mm, wheel_d);
  printf("wheel_d=%lu mm  mdeg=%lld -> mm=%ld -> mdeg=%lld\n",
         (unsigned long)wheel_d, (long long)mdeg,
         (long)mm, (long long)back);
  return 0;
}

static int do_alg_subcmd(int argc, FAR char *argv[])
{
  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _alg {traj|settings|angle} ...\n");
      return 1;
    }
  if (strcmp(argv[0], "traj") == 0)
    {
      return do_alg_traj(argc - 1, &argv[1]);
    }
  if (strcmp(argv[0], "settings") == 0)
    {
      return do_alg_settings(argc - 1, &argv[1]);
    }
  if (strcmp(argv[0], "angle") == 0)
    {
      return do_alg_angle(argc - 1, &argv[1]);
    }
  fprintf(stderr, "_alg: unknown subcommand '%s'\n", argv[0]);
  return 1;
}

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

  if (strcmp(verb, "_motor") == 0)
    {
      return do_motor_subcmd(argc - 2, &argv[2]);
    }

  if (strcmp(verb, "_alg") == 0)
    {
      return do_alg_subcmd(argc - 2, &argv[2]);
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
