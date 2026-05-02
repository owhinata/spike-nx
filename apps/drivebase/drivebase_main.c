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
#include "drivebase_servo.h"
#include "drivebase_drivebase.h"
#include "drivebase_rt.h"
#include "drivebase_chardev_handler.h"
#include "drivebase_imu.h"
#include "drivebase_daemon.h"

#include <pthread.h>
#include <time.h>

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

/* Forward decls needed because the _drive block (next) uses helpers
 * defined later in the file inside the _servo block.
 */

static uint64_t now_us(void);

/****************************************************************************
 * Private Functions: hidden _imu test verb (Issue #77 development only)
 *
 * Open /dev/uorb/sensor_imu0, drain a few ticks, print bias / heading.
 * Used to validate the integration math + bias estimator before the
 * daemon FSM (commit #11) wires SET_USE_GYRO into the drivebase loop.
 *
 *   drivebase _imu calibrate          ~250 ms idle: bias should stabilise
 *   drivebase _imu heading <ms>       integrate for <ms> and dump heading
 ****************************************************************************/

static int do_imu_subcmd(int argc, FAR char *argv[])
{
  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _imu {calibrate|heading <ms>}\n");
      return 1;
    }

  struct db_imu_s im;
  int rc = db_imu_open(&im);
  if (rc < 0)
    {
      fprintf(stderr, "db_imu_open: %s\n", strerror(-rc));
      return 1;
    }

  if (strcmp(argv[0], "calibrate") == 0)
    {
      printf("calibrating: keep robot still ~250 ms ...\n");
      uint32_t total_ms = 250;
      for (uint32_t t = 0; t < total_ms; t += 25)
        {
          usleep(25000);
          db_imu_drain_and_update(&im, now_us());
          printf("  t=%3lums bias_lsb=%ld samples=%lu cal=%d "
                 "heading=%lld mdeg\n",
                 (unsigned long)t,
                 (long)db_imu_get_bias_z_lsb(&im),
                 (unsigned long)im.sample_count,
                 (int)db_imu_is_calibrated(&im),
                 (long long)db_imu_get_heading_mdeg(&im));
        }
    }
  else if (strcmp(argv[0], "heading") == 0)
    {
      uint32_t ms = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 2000;

      /* Calibrate first. */

      printf("calibrating ~250 ms ...\n");
      for (uint32_t t = 0; t < 250; t += 25)
        {
          usleep(25000);
          db_imu_drain_and_update(&im, now_us());
        }
      printf("  bias_lsb=%ld cal=%d\n",
             (long)db_imu_get_bias_z_lsb(&im),
             (int)db_imu_is_calibrated(&im));

      /* Reset heading after calibration so the post-cal integral
       * starts from 0.
       */

      db_imu_set_heading_mdeg(&im, 0);

      uint32_t step_ms = ms / 10;
      if (step_ms < 25) step_ms = 25;
      for (uint32_t t = 0; t < ms; t += step_ms)
        {
          usleep(step_ms * 1000);
          db_imu_drain_and_update(&im, now_us());
          printf("  t=%4lums heading=%lld mdeg samples=%lu\n",
                 (unsigned long)t,
                 (long long)db_imu_get_heading_mdeg(&im),
                 (unsigned long)im.sample_count);
        }
    }
  else
    {
      fprintf(stderr, "_imu: unknown subcommand '%s'\n", argv[0]);
      db_imu_close(&im);
      return 1;
    }

  db_imu_close(&im);
  return 0;
}

/****************************************************************************
 * Private Functions: hidden _daemon test verb (Issue #77 development only)
 *
 * End-to-end smoke for the chardev IPC path.  Spawns a daemon thread
 * that runs db_chardev_handler_tick + db_drivebase_update from
 * drivebase_rt's RT loop, then issues drive ioctls from the main
 * thread to exercise PICKUP_CMD + dispatch + state publish.
 *
 *   drivebase _daemon attach        # smoke: attach + idle 500ms + detach
 *   drivebase _daemon straight 200  # config + drive_straight + watch state
 *   drivebase _daemon turn 90       # config + turn
 *   drivebase _daemon stop_lat      # measure STOP fast-path latency
 ****************************************************************************/

struct daemon_ctx_s
{
  struct db_drivebase_s          db;
  struct db_chardev_handler_s    handler;
  struct db_rt_s                 rt;
  pthread_mutex_t                lock;
};

static int daemon_tick_cb(uint64_t now_us, void *arg)
{
  struct daemon_ctx_s *ctx = (struct daemon_ctx_s *)arg;
  pthread_mutex_lock(&ctx->lock);
  /* drain commands & dispatch */
  db_chardev_handler_tick(&ctx->handler, now_us);
  /* run drivebase loop if configured */
  if (ctx->handler.configured)
    {
      db_drivebase_update(&ctx->db, now_us);
      /* re-publish state after the update so a client polling
       * GET_STATE sees the latest distance/heading immediately.
       */
      struct drivebase_state_s st;
      db_drivebase_get_state(&ctx->db, &st);
      st.tick_seq = (uint32_t)(now_us & 0xffffffff);
      ioctl(ctx->handler.fd, DRIVEBASE_DAEMON_PUBLISH_STATE,
            (unsigned long)&st);
    }
  pthread_mutex_unlock(&ctx->lock);
  return 0;
}

/* `daemon_ctx_s` carries struct db_drivebase_s inside, which is
 * ~3 KB on its own.  Stack-allocating it would push the CLI task's
 * 4 KB stack over the edge, so we keep one BSS instance.  The verb
 * is a developer smoke test that does not run while the production
 * daemon FSM owns the same drivebase state in g_daemon, so the
 * single static instance is safe by construction.
 */

static struct daemon_ctx_s g_daemon_test_ctx;

static int do_daemon_run(const char *kind, int32_t arg1, int32_t arg2,
                         uint32_t wheel_d_mm, uint32_t axle_t_mm)
{
  struct daemon_ctx_s *ctxp = &g_daemon_test_ctx;
  memset(ctxp, 0, sizeof(*ctxp));
  pthread_mutex_init(&ctxp->lock, NULL);
#define ctx (*ctxp)

  /* 1. Bring up motors (CLAIM both fds, verify type 48 ×2). */

  int rc = drivebase_motor_init();
  if (rc < 0)
    {
      fprintf(stderr, "drivebase_motor_init: %s\n", strerror(-rc));
      return 1;
    }
  drivebase_motor_select_mode(DB_SIDE_LEFT,  2);
  drivebase_motor_select_mode(DB_SIDE_RIGHT, 2);
  usleep(30000);

  int port_l = drivebase_motor_port_idx(DB_SIDE_LEFT);
  int port_r = drivebase_motor_port_idx(DB_SIDE_RIGHT);

  /* 2. Init drivebase up-front (skip the CONFIG ioctl path for now —
   *    that exercises a separate dispatch in the RT thread context
   *    that's covered in commit #11 once the daemon FSM owns it.
   *    Hard-code wheel_d_mm / axle_t_mm from the verb args.)
   */

  rc = db_drivebase_init(&ctx.db, wheel_d_mm, axle_t_mm);
  if (rc < 0)
    {
      fprintf(stderr, "db_drivebase_init: %s\n", strerror(-rc));
      drivebase_motor_deinit();
      return 1;
    }
  rc = db_drivebase_reset(&ctx.db, now_us());
  if (rc < 0)
    {
      fprintf(stderr, "db_drivebase_reset: %s\n", strerror(-rc));
      drivebase_motor_deinit();
      return 1;
    }

  /* 3. ATTACH to /dev/drivebase. */

  rc = db_chardev_handler_attach(&ctx.handler, &ctx.db, port_l, port_r,
                                 DRIVEBASE_ON_COMPLETION_COAST);
  if (rc < 0)
    {
      fprintf(stderr, "chardev_handler_attach: %s\n", strerror(-rc));
      drivebase_motor_deinit();
      return 1;
    }
  /* Tell the handler that drivebase is already configured so it won't
   * wait for a CONFIG envelope from the user before dispatching drive
   * verbs.
   */

  ctx.handler.configured = true;
  ctx.handler.wheel_d_mm = wheel_d_mm;
  ctx.handler.axle_t_mm  = axle_t_mm;

  /* 3. Spawn the RT tick task. */

  db_rt_init(&ctx.rt);
  rc = db_rt_start(&ctx.rt, CONFIG_APP_DRIVEBASE_RT_PRIORITY,
                   daemon_tick_cb, &ctx);
  if (rc < 0)
    {
      fprintf(stderr, "db_rt_start: %s\n", strerror(-rc));
      db_chardev_handler_detach(&ctx.handler);
      drivebase_motor_deinit();
      return 1;
    }

  /* 4. From the main thread, issue ioctls to the chardev so the daemon
   *    thread sees them through PICKUP_CMD on the next tick.
   */

  int dev = open(DRIVEBASE_DEVPATH, O_RDWR);
  if (dev < 0)
    {
      fprintf(stderr, "open dev: %s\n", strerror(errno));
      db_rt_stop(&ctx.rt, 100);
      db_chardev_handler_detach(&ctx.handler);
      drivebase_motor_deinit();
      return 1;
    }

  int exit_rc = 0;

  if (strcmp(kind, "attach") == 0)
    {
      printf("attach: ok\n");
      usleep(500000);
    }
  else
    {
      /* (CONFIG already applied at daemon init — issue drive verb
       * directly via ioctl through the kernel cmd_ring.)
       */

      if (strcmp(kind, "straight") == 0)
        {
          struct drivebase_drive_straight_s a = {
            .distance_mm   = arg1,
            .on_completion = DRIVEBASE_ON_COMPLETION_BRAKE,
          };
          if (ioctl(dev, DRIVEBASE_DRIVE_STRAIGHT,
                    (unsigned long)&a) < 0)
            {
              fprintf(stderr, "DRIVE_STRAIGHT: %s\n", strerror(errno));
              exit_rc = 1; goto out;
            }
        }
      else if (strcmp(kind, "turn") == 0)
        {
          struct drivebase_turn_s a = {
            .angle_deg     = arg1,
            .on_completion = DRIVEBASE_ON_COMPLETION_BRAKE,
          };
          if (ioctl(dev, DRIVEBASE_TURN, (unsigned long)&a) < 0)
            {
              fprintf(stderr, "TURN: %s\n", strerror(errno));
              exit_rc = 1; goto out;
            }
        }
      else if (strcmp(kind, "forever") == 0)
        {
          struct drivebase_drive_forever_s a = {
            .speed_mmps    = arg1,
            .turn_rate_dps = arg2,
          };
          if (ioctl(dev, DRIVEBASE_DRIVE_FOREVER,
                    (unsigned long)&a) < 0)
            {
              fprintf(stderr, "DRIVE_FOREVER: %s\n", strerror(errno));
              exit_rc = 1; goto out;
            }
        }
      else if (strcmp(kind, "stop_lat") == 0)
        {
          /* Start a forever, wait for motor to be moving, time STOP. */
          struct drivebase_drive_forever_s f = {
            .speed_mmps    = 200,
            .turn_rate_dps = 0,
          };
          ioctl(dev, DRIVEBASE_DRIVE_FOREVER, (unsigned long)&f);
          usleep(700000);  /* let motors reach speed */
          struct drivebase_state_s st_before;
          ioctl(dev, DRIVEBASE_GET_STATE, (unsigned long)&st_before);

          struct timespec t0, t1;
          clock_gettime(CLOCK_MONOTONIC, &t0);
          struct drivebase_stop_s st = {
            .on_completion = DRIVEBASE_ON_COMPLETION_COAST,
          };
          ioctl(dev, DRIVEBASE_STOP, (unsigned long)&st);
          clock_gettime(CLOCK_MONOTONIC, &t1);

          uint64_t lat_us =
              (uint64_t)(t1.tv_sec - t0.tv_sec) * 1000000ULL +
              (uint64_t)(t1.tv_nsec - t0.tv_nsec) / 1000ULL;
          printf("STOP latency: %llu us  (kernel emergency_cb path)\n",
                 (unsigned long long)lat_us);
          usleep(50000);
          struct drivebase_state_s st_after;
          ioctl(dev, DRIVEBASE_GET_STATE, (unsigned long)&st_after);
          printf("  pre-STOP  v=%ld dist=%ld\n",
                 (long)st_before.drive_speed_mmps,
                 (long)st_before.distance_mm);
          printf("  post-STOP v=%ld dist=%ld\n",
                 (long)st_after.drive_speed_mmps,
                 (long)st_after.distance_mm);
          goto out;
        }

      /* Poll state for up to ~3 sec. */

      for (int i = 0; i < 30; i++)
        {
          usleep(100000);
          struct drivebase_state_s st;
          if (ioctl(dev, DRIVEBASE_GET_STATE, (unsigned long)&st) < 0)
            {
              break;
            }
          printf("t=%4dms dist=%ld v=%ld angle=%ld done=%d cmd=%u\n",
                 (i + 1) * 100, (long)st.distance_mm,
                 (long)st.drive_speed_mmps, (long)st.angle_mdeg,
                 (int)st.is_done, st.active_command);
          if (st.is_done) break;
        }

      /* Final stop. */

      struct drivebase_stop_s sst = {
        .on_completion = DRIVEBASE_ON_COMPLETION_COAST,
      };
      ioctl(dev, DRIVEBASE_STOP, (unsigned long)&sst);
      usleep(20000);
    }

out:
  close(dev);
  db_rt_stop(&ctx.rt, 100);
  db_chardev_handler_detach(&ctx.handler);
  drivebase_motor_deinit();
#undef ctx
  return exit_rc;
}

static int do_daemon_subcmd(int argc, FAR char *argv[])
{
  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _daemon "
              "{attach|straight <mm>|turn <deg>|forever <mmps> <dps>|"
              "stop_lat} [wheel_mm] [axle_mm]\n");
      return 1;
    }

  const char *kind = argv[0];
  int32_t  arg1 = (argc >= 2) ? (int32_t)atol(argv[1]) : 0;
  int32_t  arg2 = (argc >= 3) ? (int32_t)atol(argv[2]) : 0;
  uint32_t wheel_d_mm = 56;
  uint32_t axle_t_mm  = 112;     /* SPIKE driving base reference frame  */
  if (argc >= 4) wheel_d_mm = (uint32_t)atoi(argv[3]);
  if (argc >= 5) axle_t_mm  = (uint32_t)atoi(argv[4]);
  return do_daemon_run(kind, arg1, arg2, wheel_d_mm, axle_t_mm);
}

/****************************************************************************
 * Private Functions: hidden _rt test verb (Issue #77 development only)
 *
 * Spawns the SCHED_FIFO 5 ms tick task with a no-op callback so the
 * jitter ring fills up under realistic scheduling pressure (BTstack,
 * sound DAC, USB CDC, LUMP kthreads all running).  Stops after the
 * requested duration and prints the histogram.
 ****************************************************************************/

static int rt_noop_cb(uint64_t now_us, void *arg)
{
  (void)now_us;
  (void)arg;
  return 0;
}

static int do_rt_subcmd(int argc, FAR char *argv[])
{
  uint32_t duration_ms = (argc >= 1) ? (uint32_t)atoi(argv[0]) : 2000;

  struct db_rt_s rt;
  db_rt_init(&rt);
  int rc = db_rt_start(&rt, CONFIG_APP_DRIVEBASE_RT_PRIORITY,
                       rt_noop_cb, NULL);
  if (rc < 0)
    {
      fprintf(stderr, "db_rt_start: %s\n", strerror(-rc));
      return 1;
    }

  /* Sleep at the CLI task's priority while the RT task burns ticks. */

  usleep(duration_ms * 1000);

  db_rt_stop(&rt, 100);

  struct drivebase_jitter_dump_s d;
  db_rt_get_jitter(&rt, &d);

  printf("rt: ticks=%lu max_lag=%lu us miss=%lu\n",
         (unsigned long)d.total_ticks,
         (unsigned long)d.max_lag_us,
         (unsigned long)d.deadline_miss_count);
  printf("    hist <50/50-100/100-200/200-500/500-1k/1k-2k/2k-5k/5k+:\n"
         "         %5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu\n",
         (unsigned long)d.hist_us[0], (unsigned long)d.hist_us[1],
         (unsigned long)d.hist_us[2], (unsigned long)d.hist_us[3],
         (unsigned long)d.hist_us[4], (unsigned long)d.hist_us[5],
         (unsigned long)d.hist_us[6], (unsigned long)d.hist_us[7]);

  if (d.total_ticks > 0)
    {
      uint32_t cum = 0;
      uint32_t p50_th  = (d.total_ticks + 1) / 2;
      uint32_t p99_th  = (d.total_ticks * 99 + 99) / 100;
      uint32_t p999_th = (d.total_ticks * 999 + 999) / 1000;
      static const uint32_t bucket_hi[8] =
        { 50, 100, 200, 500, 1000, 2000, 5000, UINT32_MAX };
      uint32_t p50 = 0, p99 = 0, p999 = 0;
      for (uint32_t i = 0; i < 8; i++)
        {
          cum += d.hist_us[i];
          if (p50  == 0 && cum >= p50_th)  p50  = bucket_hi[i];
          if (p99  == 0 && cum >= p99_th)  p99  = bucket_hi[i];
          if (p999 == 0 && cum >= p999_th) p999 = bucket_hi[i];
        }
      printf("    p50<= %lu  p99<= %lu  p999<= %lu  (us)\n",
             (unsigned long)p50, (unsigned long)p99,
             (unsigned long)p999);
    }
  return 0;
}

/****************************************************************************
 * Private Functions: hidden _drive test verb (Issue #77 development only)
 *
 * Standalone L+R closed-loop runs through drivebase_drivebase before
 * the daemon FSM (commit #11) wires the user-facing verbs.  Each
 * invocation does motor_init + drivebase_init + reset + drive +
 * tick-loop + stop + deinit in one task.
 ****************************************************************************/

/* `struct db_drivebase_s` is ~3 KB (two servos × ~1.4 KB each).
 * Stack-allocating it overflows the 4 KB CLI stack, so the test
 * verb keeps a single BSS instance.  Same safety argument as
 * g_daemon_test_ctx — the verb is a developer smoke that shares
 * BSS with g_daemon and must not be invoked while the production
 * daemon FSM is running.
 */

static struct db_drivebase_s g_drive_test_db;

static int do_drive_run(const char *kind, int32_t arg1, int32_t arg2,
                        uint32_t duration_ms, uint8_t on_completion,
                        uint32_t wheel_d_mm, uint32_t axle_t_mm)
{
  int rc = drivebase_motor_init();
  if (rc < 0)
    {
      fprintf(stderr, "drivebase_motor_init: %s\n", strerror(-rc));
      return 1;
    }

  drivebase_motor_select_mode(DB_SIDE_LEFT,  2);
  drivebase_motor_select_mode(DB_SIDE_RIGHT, 2);
  usleep(30000);

  struct db_drivebase_s *dbp = &g_drive_test_db;
  memset(dbp, 0, sizeof(*dbp));
#define db (*dbp)
  rc = db_drivebase_init(&db, wheel_d_mm, axle_t_mm);
  if (rc < 0)
    {
      fprintf(stderr, "db_drivebase_init: %s\n", strerror(-rc));
      drivebase_motor_deinit();
      return 1;
    }

  uint64_t t0 = now_us();
  rc = db_drivebase_reset(&db, t0);
  if (rc < 0)
    {
      fprintf(stderr, "db_drivebase_reset: %s\n", strerror(-rc));
      drivebase_motor_deinit();
      return 1;
    }

  if (strcmp(kind, "straight") == 0)
    {
      rc = db_drivebase_drive_straight(&db, t0, arg1, on_completion);
    }
  else if (strcmp(kind, "turn") == 0)
    {
      rc = db_drivebase_turn(&db, t0, arg1, on_completion);
    }
  else if (strcmp(kind, "curve") == 0)
    {
      rc = db_drivebase_drive_curve(&db, t0, arg1, arg2, on_completion);
    }
  else if (strcmp(kind, "forever") == 0)
    {
      rc = db_drivebase_drive_forever(&db, t0, arg1, arg2);
    }
  else
    {
      rc = db_drivebase_stop(&db, t0, on_completion);
    }
  if (rc < 0)
    {
      fprintf(stderr, "drive %s: %s\n", kind, strerror(-rc));
      db_drivebase_stop(&db, now_us(), DRIVEBASE_ON_COMPLETION_COAST);
      drivebase_motor_deinit();
      return 1;
    }

  /* 5 ms tick loop */

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  uint32_t total_ticks  = duration_ms / 5;
  uint32_t status_every = total_ticks / 12;
  if (status_every == 0) status_every = 1;

  for (uint32_t i = 0; i < total_ticks; i++)
    {
      next.tv_nsec += 5000000;
      if (next.tv_nsec >= 1000000000)
        {
          next.tv_nsec -= 1000000000;
          next.tv_sec  += 1;
        }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

      int ur = db_drivebase_update(&db, now_us());
      if (ur < 0)
        {
          fprintf(stderr, "tick %lu: update %s\n",
                  (unsigned long)i, strerror(-ur));
          break;
        }

      if ((i % status_every) == 0)
        {
          struct drivebase_state_s st;
          db_drivebase_get_state(&db, &st);
          printf("t=%4lums dist=%ld mm v=%ld mmps angle=%ld mdeg "
                 "tr=%ld dps done=%d stall=%d cmd=%u\n",
                 (unsigned long)(i * 5),
                 (long)st.distance_mm, (long)st.drive_speed_mmps,
                 (long)st.angle_mdeg, (long)st.turn_rate_dps,
                 (int)st.is_done, (int)st.is_stalled,
                 st.active_command);
        }
    }

  db_drivebase_stop(&db, now_us(), DRIVEBASE_ON_COMPLETION_COAST);
  drivebase_motor_deinit();
#undef db
  return 0;
}

static int do_drive_subcmd(int argc, FAR char *argv[])
{
  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _drive {straight|turn|curve|forever|stop}\n"
              "       <arg1> [arg2] [duration_ms] [coast|brake|hold]\n"
              "       [wheel_mm] [axle_mm]\n"
              "  straight: arg1 = distance_mm\n"
              "  turn:     arg1 = angle_deg (positive = CCW from above)\n"
              "  curve:    arg1 = radius_mm, arg2 = angle_deg\n"
              "  forever:  arg1 = speed_mmps, arg2 = turn_rate_dps\n");
      return 1;
    }

  const char *kind = argv[0];
  int32_t arg1 = 0, arg2 = 0;
  uint32_t duration = 3000;
  uint8_t on_completion = DRIVEBASE_ON_COMPLETION_BRAKE;
  uint32_t wheel_d = 56;     /* SPIKE Medium wheel default                */
  uint32_t axle_t  = 112;    /* SPIKE driving base reference frame        */

  if (argc >= 2) arg1 = (int32_t)atol(argv[1]);
  if (argc >= 3) arg2 = (int32_t)atol(argv[2]);
  if (argc >= 4) duration = (uint32_t)atoi(argv[3]);
  if (argc >= 5)
    {
      if (strcmp(argv[4], "coast") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_COAST;
      else if (strcmp(argv[4], "brake") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_BRAKE;
      else if (strcmp(argv[4], "hold") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_HOLD;
    }
  if (argc >= 6) wheel_d = (uint32_t)atoi(argv[5]);
  if (argc >= 7) axle_t  = (uint32_t)atoi(argv[6]);

  return do_drive_run(kind, arg1, arg2, duration, on_completion,
                      wheel_d, axle_t);
}

/****************************************************************************
 * Private Functions: hidden _servo test verb (Issue #77 development only)
 *
 * Run a single side's closed loop standalone for a few seconds while
 * the daemon FSM (commit #11) is being built up.  Each invocation
 * does init+loop+deinit in one task; the loop drives the servo's
 * 5 ms tick from a clock_nanosleep absolute-deadline ladder so we
 * can observe steady-state behaviour without the full daemon yet.
 ****************************************************************************/

static uint64_t now_us(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static int do_servo_run(enum db_side_e side, const char *target_kind,
                        int32_t target, uint32_t duration_ms,
                        uint8_t on_completion)
{
  int rc = drivebase_motor_init();
  if (rc < 0)
    {
      fprintf(stderr, "drivebase_motor_init: %s\n", strerror(-rc));
      return 1;
    }

  /* Select POS mode 2 on the side we drive so the encoder is signed
   * int32 deg.
   */

  drivebase_motor_select_mode(side, 2);

  /* Give the device ~30 ms to ack the SELECT and warm the subscriber
   * cursor — at 1 kHz LUMP publish rate this is ~30 frames, plenty
   * for the new mode to take effect.
   */

  usleep(30000);

  /* Same BSS-not-stack reasoning as g_drive_test_db: the servo struct
   * is ~1.4 KB which crowds a 4 KB CLI stack once printf frames pile
   * on top.  Single static instance — verb is dev-only and shares
   * BSS with g_daemon.servo[] by intent.
   */

  static struct db_servo_s g_servo_test;
  struct db_servo_s *servop = &g_servo_test;
  memset(servop, 0, sizeof(*servop));
#define servo (*servop)
  db_servo_init(&servo, side);
  rc = db_servo_reset(&servo, now_us());
  if (rc < 0)
    {
      fprintf(stderr, "db_servo_reset: %s\n", strerror(-rc));
      drivebase_motor_deinit();
      return 1;
    }

  uint64_t t0 = now_us();
  if (strcmp(target_kind, "position") == 0)
    {
      const struct db_traj_limits_s *dl = db_settings_distance_limits(56);
      db_servo_position_relative(&servo, t0,
                                 (int64_t)target * 1000,  /* deg→mdeg */
                                 dl->v_max_mdegps,
                                 dl->accel_mdegps2,
                                 dl->decel_mdegps2,
                                 on_completion);
    }
  else if (strcmp(target_kind, "forever") == 0)
    {
      const struct db_traj_limits_s *dl = db_settings_distance_limits(56);
      db_servo_forever(&servo, t0,
                       target * 1000 /* mdegps */,
                       dl->accel_mdegps2);
    }
  else
    {
      db_servo_stop(&servo, t0, on_completion);
    }

  /* 5 ms tick loop.  clock_nanosleep absolute deadlines drift-free. */

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);
  uint32_t total_ticks = duration_ms / 5;
  uint32_t status_every = total_ticks / 10;
  if (status_every == 0) status_every = 1;

  for (uint32_t i = 0; i < total_ticks; i++)
    {
      next.tv_nsec += 5000000;
      if (next.tv_nsec >= 1000000000)
        {
          next.tv_nsec -= 1000000000;
          next.tv_sec  += 1;
        }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);

      int ur = db_servo_update(&servo, now_us());
      if (ur < 0)
        {
          fprintf(stderr, "tick %lu: update %s\n",
                  (unsigned long)i, strerror(-ur));
          break;
        }

      if ((i % status_every) == 0)
        {
          struct db_servo_status_s st;
          db_servo_get_status(&servo, &st);
          printf("t=%4lums ref_x=%lld act_x=%lld v=%ld duty=%ld "
                 "act=%u done=%d stall=%d\n",
                 (unsigned long)(i * 5),
                 (long long)st.ref_x_mdeg,
                 (long long)st.act_x_mdeg,
                 (long)st.act_v_mdegps, (long)st.applied_duty,
                 st.actuation, (int)st.done, (int)st.stalled);
        }
    }

  /* Final stop unless caller wanted HOLD. */

  db_servo_stop(&servo, now_us(), DRIVEBASE_ON_COMPLETION_COAST);
  drivebase_motor_deinit();
#undef servo
  return 0;
}

static int do_servo_subcmd(int argc, FAR char *argv[])
{
  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _servo {position|forever|stop} <l|r> "
              "<value> [duration_ms] [coast|brake|hold|csmart|bsmart]\n"
              "  position: value = relative deg target\n"
              "  forever:  value = signed deg/s\n"
              "  stop:     value ignored\n");
      return 1;
    }

  if (argc < 3)
    {
      fprintf(stderr, "_servo %s: need <l|r> <value>\n", argv[0]);
      return 1;
    }

  enum db_side_e side;
  if (parse_side(argv[1], &side) < 0)
    {
      fprintf(stderr, "bad side: %s\n", argv[1]);
      return 1;
    }
  int32_t value     = (int32_t)atol(argv[2]);
  uint32_t duration = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 1500;

  uint8_t on_completion = DRIVEBASE_ON_COMPLETION_BRAKE;
  if (argc >= 5)
    {
      if (strcmp(argv[4], "coast") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_COAST;
      else if (strcmp(argv[4], "brake") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_BRAKE;
      else if (strcmp(argv[4], "hold") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_HOLD;
      else if (strcmp(argv[4], "csmart") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_COAST_SMART;
      else if (strcmp(argv[4], "bsmart") == 0)
        on_completion = DRIVEBASE_ON_COMPLETION_BRAKE_SMART;
    }

  return do_servo_run(side, argv[0], value, duration, on_completion);
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
  uint32_t axle_t  = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 112;

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

  if (strcmp(verb, "_servo") == 0)
    {
      return do_servo_subcmd(argc - 2, &argv[2]);
    }

  if (strcmp(verb, "_drive") == 0)
    {
      return do_drive_subcmd(argc - 2, &argv[2]);
    }

  if (strcmp(verb, "_rt") == 0)
    {
      return do_rt_subcmd(argc - 2, &argv[2]);
    }

  if (strcmp(verb, "_daemon") == 0)
    {
      return do_daemon_subcmd(argc - 2, &argv[2]);
    }

  if (strcmp(verb, "_imu") == 0)
    {
      return do_imu_subcmd(argc - 2, &argv[2]);
    }

  if (strcmp(verb, "help") == 0 ||
      strcmp(verb, "-h")   == 0 ||
      strcmp(verb, "--help") == 0)
    {
      usage();
      return 0;
    }

  /* User-facing verbs.  start/stop talk to drivebase_daemon_*; the
   * rest go through real ioctl on /dev/drivebase, served by the
   * kernel chardev shim from commit #2.
   */

  if (strcmp(verb, "start") == 0)
    {
      uint32_t wheel_d_mm = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 56;
      uint32_t axle_t_mm  = (argc >= 4) ? (uint32_t)atoi(argv[3]) : 112;
      int rc = drivebase_daemon_start(wheel_d_mm, axle_t_mm);
      if (rc < 0)
        {
          fprintf(stderr, "drivebase start: %s\n", strerror(-rc));
          return 1;
        }
      printf("drivebase: started (pid=%d wheel=%lu axle=%lu)\n",
             rc, (unsigned long)wheel_d_mm, (unsigned long)axle_t_mm);
      return 0;
    }

  if (strcmp(verb, "stop") == 0)
    {
      int rc = drivebase_daemon_stop(2000);
      if (rc < 0 && rc != -EAGAIN)
        {
          fprintf(stderr, "drivebase stop: %s\n", strerror(-rc));
          return 1;
        }
      printf("drivebase: %s\n",
             rc == -EAGAIN ? "not running" : "stopped");
      return 0;
    }

  /* All remaining verbs need an open /dev/drivebase. */

  int dev = -1;
  if (strcmp(verb, "config")     == 0 || strcmp(verb, "straight")  == 0 ||
      strcmp(verb, "turn")       == 0 || strcmp(verb, "forever")   == 0 ||
      strcmp(verb, "stop-motion")== 0 || strcmp(verb, "get-state") == 0 ||
      strcmp(verb, "set-gyro")   == 0 || strcmp(verb, "jitter")    == 0)
    {
      dev = open(DRIVEBASE_DEVPATH, O_RDWR);
      if (dev < 0)
        {
          fprintf(stderr, "open(%s): %s\n", DRIVEBASE_DEVPATH,
                  strerror(errno));
          return 1;
        }
    }

  if (strcmp(verb, "config") == 0)
    {
      if (argc < 4)
        {
          fprintf(stderr, "usage: drivebase config <wheel_mm> <axle_mm>\n");
          close(dev); return 1;
        }
      struct drivebase_config_s c =
        { .wheel_diameter_mm = (uint32_t)atoi(argv[2]),
          .axle_track_mm     = (uint32_t)atoi(argv[3]) };
      int rc = ioctl(dev, DRIVEBASE_CONFIG, (unsigned long)&c);
      close(dev);
      if (rc < 0)
        {
          fprintf(stderr, "DRIVEBASE_CONFIG: %s\n", strerror(errno));
          return 1;
        }
      return 0;
    }

  if (strcmp(verb, "straight") == 0)
    {
      if (argc < 3)
        {
          fprintf(stderr, "usage: drivebase straight <mm> [coast|brake|hold]\n");
          close(dev); return 1;
        }
      struct drivebase_drive_straight_s a =
        { .distance_mm = (int32_t)atoi(argv[2]),
          .on_completion = DRIVEBASE_ON_COMPLETION_BRAKE };
      if (argc >= 4)
        {
          if      (strcmp(argv[3], "coast") == 0) a.on_completion = DRIVEBASE_ON_COMPLETION_COAST;
          else if (strcmp(argv[3], "brake") == 0) a.on_completion = DRIVEBASE_ON_COMPLETION_BRAKE;
          else if (strcmp(argv[3], "hold")  == 0) a.on_completion = DRIVEBASE_ON_COMPLETION_HOLD;
        }
      int rc = ioctl(dev, DRIVEBASE_DRIVE_STRAIGHT, (unsigned long)&a);
      close(dev);
      if (rc < 0) { fprintf(stderr, "DRIVE_STRAIGHT: %s\n", strerror(errno)); return 1; }
      return 0;
    }

  if (strcmp(verb, "turn") == 0)
    {
      if (argc < 3)
        {
          fprintf(stderr, "usage: drivebase turn <deg>\n");
          close(dev); return 1;
        }
      struct drivebase_turn_s a =
        { .angle_deg = (int32_t)atoi(argv[2]),
          .on_completion = DRIVEBASE_ON_COMPLETION_BRAKE };
      int rc = ioctl(dev, DRIVEBASE_TURN, (unsigned long)&a);
      close(dev);
      if (rc < 0) { fprintf(stderr, "TURN: %s\n", strerror(errno)); return 1; }
      return 0;
    }

  if (strcmp(verb, "forever") == 0)
    {
      if (argc < 4)
        {
          fprintf(stderr, "usage: drivebase forever <mmps> <dps>\n");
          close(dev); return 1;
        }
      struct drivebase_drive_forever_s a =
        { .speed_mmps    = (int32_t)atoi(argv[2]),
          .turn_rate_dps = (int32_t)atoi(argv[3]) };
      int rc = ioctl(dev, DRIVEBASE_DRIVE_FOREVER, (unsigned long)&a);
      close(dev);
      if (rc < 0) { fprintf(stderr, "DRIVE_FOREVER: %s\n", strerror(errno)); return 1; }
      return 0;
    }

  if (strcmp(verb, "stop-motion") == 0)
    {
      uint8_t oc = DRIVEBASE_ON_COMPLETION_COAST;
      if (argc >= 3)
        {
          if      (strcmp(argv[2], "coast") == 0) oc = DRIVEBASE_ON_COMPLETION_COAST;
          else if (strcmp(argv[2], "brake") == 0) oc = DRIVEBASE_ON_COMPLETION_BRAKE;
          else if (strcmp(argv[2], "hold")  == 0) oc = DRIVEBASE_ON_COMPLETION_HOLD;
        }
      struct drivebase_stop_s a = { .on_completion = oc };
      int rc = ioctl(dev, DRIVEBASE_STOP, (unsigned long)&a);
      close(dev);
      if (rc < 0) { fprintf(stderr, "STOP: %s\n", strerror(errno)); return 1; }
      return 0;
    }

  if (strcmp(verb, "get-state") == 0)
    {
      struct drivebase_state_s st;
      int rc = ioctl(dev, DRIVEBASE_GET_STATE, (unsigned long)&st);
      close(dev);
      if (rc < 0) { fprintf(stderr, "GET_STATE: %s\n", strerror(errno)); return 1; }
      printf("dist=%ld mm v=%ld mmps angle=%ld mdeg tr=%ld dps "
             "done=%u stall=%u cmd=%u tick=%lu\n",
             (long)st.distance_mm, (long)st.drive_speed_mmps,
             (long)st.angle_mdeg, (long)st.turn_rate_dps,
             st.is_done, st.is_stalled, st.active_command,
             (unsigned long)st.tick_seq);
      return 0;
    }

  if (strcmp(verb, "set-gyro") == 0)
    {
      if (argc < 3)
        {
          fprintf(stderr, "usage: drivebase set-gyro <none|1d|3d>\n");
          close(dev); return 1;
        }
      uint8_t mode = DRIVEBASE_USE_GYRO_NONE;
      if      (strcmp(argv[2], "none") == 0) mode = DRIVEBASE_USE_GYRO_NONE;
      else if (strcmp(argv[2], "1d")   == 0) mode = DRIVEBASE_USE_GYRO_1D;
      else if (strcmp(argv[2], "3d")   == 0) mode = DRIVEBASE_USE_GYRO_3D;
      struct drivebase_set_use_gyro_s a = { .use_gyro = mode };
      int rc = ioctl(dev, DRIVEBASE_SET_USE_GYRO, (unsigned long)&a);
      close(dev);
      if (rc < 0) { fprintf(stderr, "SET_USE_GYRO: %s\n", strerror(errno)); return 1; }
      return 0;
    }

  if (strcmp(verb, "jitter") == 0)
    {
      struct drivebase_jitter_dump_s d;
      int rc = ioctl(dev, DRIVEBASE_JITTER_DUMP, (unsigned long)&d);
      close(dev);
      if (rc < 0) { fprintf(stderr, "JITTER_DUMP: %s\n", strerror(errno)); return 1; }
      printf("ticks=%lu max_lag=%lu us miss=%lu\n",
             (unsigned long)d.total_ticks,
             (unsigned long)d.max_lag_us,
             (unsigned long)d.deadline_miss_count);
      printf("hist <50/50-100/100-200/200-500/500-1k/1k-2k/2k-5k/5k+:\n"
             "     %5lu %5lu %5lu %5lu %5lu %5lu %5lu %5lu\n",
             (unsigned long)d.hist_us[0], (unsigned long)d.hist_us[1],
             (unsigned long)d.hist_us[2], (unsigned long)d.hist_us[3],
             (unsigned long)d.hist_us[4], (unsigned long)d.hist_us[5],
             (unsigned long)d.hist_us[6], (unsigned long)d.hist_us[7]);
      return 0;
    }

  fprintf(stderr, "drivebase: unknown verb '%s'\n", verb);
  usage();
  return 1;
}
