/****************************************************************************
 * apps/linetrace/linetrace_main.c
 *
 * NSH builtin user-space PID line follower for the SPIKE Prime drivebase.
 * Persistent daemon model:
 *
 *   linetrace start                  spawn daemon (idle: speed=0, kp=0)
 *   linetrace cal                    daemon samples for 3 s, CLI prints result
 *   linetrace run <speed> <kp> [t]   update daemon params in place
 *   linetrace run 0 0                command "stop motion" (daemon stays alive)
 *   linetrace stop                   coast wheels, daemon exits
 *   linetrace status                 print daemon state + last loop values
 *
 * The daemon owns the color sensor CLAIM and the /dev/drivebase fd.  The
 * 100 Hz loop reads /dev/uorb/sensor_color (REFLT mode), computes a P term
 * from the deviation against `target`, and issues DRIVEBASE_DRIVE_FOREVER
 * each iteration.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <sched.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <sys/ioctl.h>

#include <nuttx/sensors/sensor.h>

#include <arch/board/board_drivebase.h>
#include <arch/board/board_legosensor.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define COLOR_DEVPATH       "/dev/uorb/sensor_color"
#define COLOR_MODE_REFLT    1

#define DEFAULT_TARGET      50
#define DEFAULT_MAX_TURN    180
#define DEFAULT_HZ          100

#define CAL_DURATION_MS     3000
#define CAL_TIMEOUT_MS      5000
#define SELECT_POLL_MS      500
#define STOP_WAIT_MS        2000

#define DAEMON_NAME         "linetrace_d"
#define DAEMON_PRIORITY     100
#define DAEMON_STACK        4096

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct
{
  int speed_mmps;
  int kp_x100;
  int ki_x100;
  int kd_x100;
  int target;
  int max_turn;
  int hz;
} g_params;

static struct
{
  uint64_t iter;
  int      last_refl;
  int      last_err;
  int      last_p_term;
  int      last_i_term;
  int      last_d_term;
  int      last_turn_dps;
  int      last_ioctl_rc;
  int      last_ioctl_errno;
} g_stats;

/* PID state — owned by the control loop.  do_run sets the reset flag
 * (matching the g_cal_request / g_brake_request pattern); the loop
 * picks it up on the next tick and zeros the integrator + clears the
 * derivative seed so the first sample after a parameter change does
 * not produce a spurious D kick.
 */

static int           g_pid_i_acc;
static int           g_pid_prev_err;
static bool          g_pid_has_prev;
static volatile bool g_pid_reset_request;

static volatile bool g_daemon_running = false;
static volatile bool g_daemon_stop    = false;
static int           g_daemon_pid     = -1;

static volatile bool g_cal_request   = false;
static volatile bool g_cal_done      = false;
static volatile bool g_brake_request = false;
static struct
{
  int min_v;
  int max_v;
  int count;
} g_cal_result;

/****************************************************************************
 * Sensor / drivebase helpers
 ****************************************************************************/

static int color_open_select(void)
{
  int fd = open(COLOR_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    {
      fprintf(stderr, "linetrace: open(%s): %s\n",
              COLOR_DEVPATH, strerror(errno));
      return -errno;
    }

  if (ioctl(fd, LEGOSENSOR_CLAIM, 0) < 0)
    {
      fprintf(stderr, "linetrace: CLAIM: %s\n", strerror(errno));
      close(fd);
      return -EIO;
    }

  struct legosensor_select_arg_s sel = { .mode = COLOR_MODE_REFLT };
  if (ioctl(fd, LEGOSENSOR_SELECT, (unsigned long)&sel) < 0)
    {
      fprintf(stderr, "linetrace: SELECT mode %u: %s\n",
              COLOR_MODE_REFLT, strerror(errno));
      close(fd);
      return -EIO;
    }

  struct pollfd pfd = { .fd = fd, .events = POLLIN };
  struct timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  int remaining = SELECT_POLL_MS;
  while (remaining > 0)
    {
      int pr = poll(&pfd, 1, remaining);
      if (pr <= 0)
        {
          break;
        }

      struct lump_sample_s s;
      ssize_t n = read(fd, &s, sizeof(s));
      if (n == sizeof(s) && s.mode_id == COLOR_MODE_REFLT && s.len > 0)
        {
          return fd;
        }

      struct timespec t1;
      clock_gettime(CLOCK_MONOTONIC, &t1);
      long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                        (t1.tv_nsec - t0.tv_nsec) / 1000000;
      remaining = SELECT_POLL_MS - (int)elapsed_ms;
    }

  fprintf(stderr,
          "linetrace: color SELECT mode %u not confirmed in %d ms\n",
          COLOR_MODE_REFLT, SELECT_POLL_MS);
  close(fd);
  return -ETIMEDOUT;
}

static int color_read_latest(int fd)
{
  int last = -1;
  for (;;)
    {
      struct lump_sample_s s;
      ssize_t n = read(fd, &s, sizeof(s));
      if (n != sizeof(s))
        {
          break;
        }

      if (s.mode_id == COLOR_MODE_REFLT && s.len > 0)
        {
          last = s.data.i8[0];
        }
    }
  return last;
}

static int drivebase_check_idle(int fd)
{
  struct drivebase_state_s st;
  memset(&st, 0, sizeof(st));
  if (ioctl(fd, DRIVEBASE_GET_STATE, (unsigned long)&st) < 0)
    {
      fprintf(stderr, "linetrace: GET_STATE: %s\n", strerror(errno));
      return -EIO;
    }

  if (st.active_command != DRIVEBASE_ACTIVE_NONE &&
      st.active_command != DRIVEBASE_ACTIVE_FOREVER)
    {
      fprintf(stderr,
              "linetrace: refused — drivebase active_command=%u "
              "(need NONE or FOREVER)\n",
              st.active_command);
      return -EBUSY;
    }

  return 0;
}

static int drivebase_send_forever(int fd, int32_t speed_mmps,
                                  int32_t turn_dps)
{
  struct drivebase_drive_forever_s a = {
    .speed_mmps    = speed_mmps,
    .turn_rate_dps = turn_dps,
  };
  return ioctl(fd, DRIVEBASE_DRIVE_FOREVER, (unsigned long)&a);
}

static void drivebase_send_stop(int fd, uint8_t on_completion)
{
  struct drivebase_stop_s a = {
    .on_completion = on_completion,
  };
  (void)ioctl(fd, DRIVEBASE_STOP, (unsigned long)&a);
}

static int clamp_int(int v, int lo, int hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/****************************************************************************
 * Daemon entry
 ****************************************************************************/

static void run_calibration(int color_fd)
{
  int min_v = 100;
  int max_v = 0;
  int count = 0;

  struct timespec t0;
  clock_gettime(CLOCK_MONOTONIC, &t0);

  while (!g_daemon_stop)
    {
      struct timespec t1;
      clock_gettime(CLOCK_MONOTONIC, &t1);
      long elapsed_ms = (t1.tv_sec - t0.tv_sec) * 1000 +
                        (t1.tv_nsec - t0.tv_nsec) / 1000000;
      if (elapsed_ms >= CAL_DURATION_MS)
        {
          break;
        }

      int cv = color_read_latest(color_fd);
      if (cv >= 0)
        {
          if (cv < min_v) min_v = cv;
          if (cv > max_v) max_v = cv;
          count++;
        }

      usleep(10000);
    }

  g_cal_result.min_v = min_v;
  g_cal_result.max_v = max_v;
  g_cal_result.count = count;
  g_cal_done    = true;
  g_cal_request = false;
}

static int linetrace_daemon(int argc, char *argv[])
{
  (void)argc;
  (void)argv;

  int color_fd = color_open_select();
  if (color_fd < 0)
    {
      g_daemon_running = false;
      return 1;
    }

  int db_fd = open(DRIVEBASE_DEVPATH, O_RDWR);
  if (db_fd < 0)
    {
      fprintf(stderr, "linetrace: open(%s): %s\n",
              DRIVEBASE_DEVPATH, strerror(errno));
      close(color_fd);
      g_daemon_running = false;
      return 1;
    }

  int last_refl = -1;

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);

  while (!g_daemon_stop)
    {
      if (g_cal_request)
        {
          run_calibration(color_fd);
          clock_gettime(CLOCK_MONOTONIC, &next);
          continue;
        }

      if (g_brake_request)
        {
          drivebase_send_forever(db_fd, 0, 0);
          drivebase_send_stop(db_fd, DRIVEBASE_ON_COMPLETION_BRAKE);
          g_params.speed_mmps = 0;
          g_params.kp_x100    = 0;
          g_params.ki_x100    = 0;
          g_params.kd_x100    = 0;
          g_pid_reset_request = true;
          g_brake_request     = false;
          clock_gettime(CLOCK_MONOTONIC, &next);
          continue;
        }

      /* PID reset on demand (do_run / brake wrote new params).  Done
       * before integrating this tick so the new gain set sees a
       * clean integrator + suppressed first-sample derivative kick.
       */

      if (g_pid_reset_request)
        {
          g_pid_i_acc         = 0;
          g_pid_has_prev      = false;
          g_pid_reset_request = false;
        }

      int v = color_read_latest(color_fd);
      if (v >= 0)
        {
          last_refl = v;
        }

      int err    = (last_refl >= 0) ? (g_params.target - last_refl) : 0;
      int dt_ms  = 1000 / g_params.hz;       /* hz validated 10..200    */
      int p_term = (g_params.kp_x100 * err) / 100;

      /* I-term: accumulate err·ms.  Anti-windup clamps |i_acc| so
       * |i_term| <= max_turn even at full ki — see the plan/Issue
       * for the trade-off vs drivebase_control.c's gated approach.
       * ki_x100 is validated to ±10000 in do_run, so the divide is
       * always in safe range and the abs() cannot hit INT_MIN.
       */

      g_pid_i_acc += err * dt_ms;
      if (g_params.ki_x100 != 0)
        {
          int ki_abs  = (g_params.ki_x100 < 0) ?
                        -g_params.ki_x100 : g_params.ki_x100;
          int i_limit = (g_params.max_turn * 100 * 1000) / ki_abs;
          g_pid_i_acc = clamp_int(g_pid_i_acc, -i_limit, i_limit);
        }
      else
        {
          g_pid_i_acc = 0;
        }
      int i_term = (int)(((int64_t)g_params.ki_x100 * g_pid_i_acc) /
                         (100 * 1000));

      /* D-term: kd × derr / dt_sec.  Suppress on first sample after
       * a reset so the initial step from 0 prev_err does not produce
       * a spurious derivative kick.
       */

      int d_term = 0;
      if (g_pid_has_prev && dt_ms > 0)
        {
          int derr = err - g_pid_prev_err;
          d_term   = (int)(((int64_t)g_params.kd_x100 * derr * 10) /
                           dt_ms);
        }
      g_pid_prev_err = err;
      g_pid_has_prev = true;

      int sum_dps  = (int)((int64_t)p_term + i_term + d_term);
      int turn_dps = clamp_int(sum_dps, -g_params.max_turn,
                                         g_params.max_turn);

      int rc = drivebase_send_forever(db_fd, g_params.speed_mmps, turn_dps);

      g_stats.iter++;
      g_stats.last_refl        = last_refl;
      g_stats.last_err         = err;
      g_stats.last_p_term      = p_term;
      g_stats.last_i_term      = i_term;
      g_stats.last_d_term      = d_term;
      g_stats.last_turn_dps    = turn_dps;
      g_stats.last_ioctl_rc    = rc;
      g_stats.last_ioctl_errno = (rc < 0) ? errno : 0;

      long period_ns = 1000000000L / g_params.hz;
      next.tv_nsec += period_ns;
      while (next.tv_nsec >= 1000000000L)
        {
          next.tv_nsec -= 1000000000L;
          next.tv_sec  += 1;
        }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

  drivebase_send_stop(db_fd, DRIVEBASE_ON_COMPLETION_COAST);
  close(color_fd);
  close(db_fd);
  g_daemon_running = false;
  return 0;
}

/****************************************************************************
 * Subcommand: start
 ****************************************************************************/

static int do_start(void)
{
  if (g_daemon_running)
    {
      fprintf(stderr, "linetrace: already running\n");
      return 1;
    }

  int db_fd = open(DRIVEBASE_DEVPATH, O_RDWR);
  if (db_fd < 0)
    {
      fprintf(stderr, "linetrace: open(%s): %s\n",
              DRIVEBASE_DEVPATH, strerror(errno));
      return 1;
    }
  int chk = drivebase_check_idle(db_fd);
  close(db_fd);
  if (chk != 0)
    {
      return 1;
    }

  g_params.speed_mmps = 0;
  g_params.kp_x100    = 0;
  g_params.ki_x100    = 0;
  g_params.kd_x100    = 0;
  g_params.target     = DEFAULT_TARGET;
  g_params.max_turn   = DEFAULT_MAX_TURN;
  g_params.hz         = DEFAULT_HZ;

  memset(&g_stats, 0, sizeof(g_stats));
  g_stats.last_refl = -1;

  g_pid_i_acc         = 0;
  g_pid_prev_err      = 0;
  g_pid_has_prev      = false;
  g_pid_reset_request = false;

  g_cal_request = false;
  g_cal_done    = false;

  g_daemon_stop    = false;
  g_daemon_running = true;

  g_daemon_pid = task_create(DAEMON_NAME, DAEMON_PRIORITY, DAEMON_STACK,
                             linetrace_daemon, NULL);
  if (g_daemon_pid < 0)
    {
      fprintf(stderr, "linetrace: task_create: %s\n", strerror(errno));
      g_daemon_running = false;
      return 1;
    }

  printf("linetrace: started (pid=%d) — idle (speed=0 kp=0 target=%d)\n",
         g_daemon_pid, DEFAULT_TARGET);
  printf("linetrace: 'linetrace cal' / 'linetrace run <speed> <kp> [target]'\n");
  return 0;
}

/****************************************************************************
 * Subcommand: stop
 ****************************************************************************/

static int do_stop(void)
{
  if (!g_daemon_running)
    {
      printf("linetrace: not running\n");
      return 0;
    }

  g_daemon_stop = true;

  int waited = 0;
  while (g_daemon_running && waited < STOP_WAIT_MS)
    {
      usleep(20000);
      waited += 20;
    }

  if (g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: daemon did not exit within %d ms\n",
              STOP_WAIT_MS);
      return 1;
    }

  printf("linetrace: stopped\n");
  return 0;
}

/****************************************************************************
 * Subcommand: cal
 ****************************************************************************/

static int do_cal(void)
{
  if (!g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: not running — 'linetrace start' first\n");
      return 1;
    }

  if (g_params.speed_mmps != 0)
    {
      fprintf(stderr,
              "linetrace: cal requires speed=0 — 'linetrace run 0 0' first\n");
      return 1;
    }

  if (g_cal_request)
    {
      fprintf(stderr, "linetrace: cal already in flight\n");
      return 1;
    }

  printf("linetrace: sampling for %d ms — sweep the sensor over "
         "black/white\n", CAL_DURATION_MS);

  g_cal_done    = false;
  g_cal_request = true;

  int waited = 0;
  while (!g_cal_done && waited < CAL_TIMEOUT_MS)
    {
      usleep(50000);
      waited += 50;
    }

  if (!g_cal_done)
    {
      fprintf(stderr, "linetrace: cal timed out (no daemon response)\n");
      g_cal_request = false;
      return 1;
    }

  if (g_cal_result.count == 0)
    {
      fprintf(stderr, "linetrace: cal received no samples\n");
      return 1;
    }

  int mid = (g_cal_result.min_v + g_cal_result.max_v) / 2;
  printf("linetrace: cal %d samples: black=%d white=%d midpoint=%d\n",
         g_cal_result.count, g_cal_result.min_v, g_cal_result.max_v, mid);
  printf("           suggested: linetrace run <speed_mmps> <kp> %d\n", mid);
  return 0;
}

/****************************************************************************
 * Subcommand: run
 ****************************************************************************/

/* Parse a PID gain (kp/ki/kd) into the x100 fixed-point representation.
 * Validates as `double` first so out-of-range or non-finite inputs
 * (e.g. `1e100`, `nan`) are rejected before the `(int)` cast can hit
 * undefined behaviour.  Range matches g_params field bounds.
 */

static int parse_gain(const char *s, int *out_x100)
{
  char  *end;
  double v = strtod(s, &end);
  if (end == s || *end != '\0' || !isfinite(v) ||
      v < -100.0 || v > 100.0)
    {
      return -1;
    }
  *out_x100 = (int)(v * 100.0 + (v >= 0.0 ? 0.5 : -0.5));
  return 0;
}

static int do_run(int argc, char **argv)
{
  if (!g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: not running — 'linetrace start' first\n");
      return 1;
    }

  if (argc < 2)
    {
      fprintf(stderr,
              "usage: linetrace run <speed_mmps> <kp> [target] "
              "[--ki K] [--kd K] [--max-turn dps] [--hz N]\n");
      return 1;
    }

  int  speed_mmps = atoi(argv[0]);
  int  kp_x100    = 0;
  if (parse_gain(argv[1], &kp_x100) < 0)
    {
      fprintf(stderr,
              "linetrace: kp must be in [-100.00, 100.00]\n");
      return 1;
    }
  int  ki_x100    = g_params.ki_x100;
  int  kd_x100    = g_params.kd_x100;
  int  target     = g_params.target;
  int  max_turn   = g_params.max_turn;
  int  hz         = g_params.hz;
  int  i          = 2;

  if (i < argc && argv[i][0] != '-')
    {
      target = atoi(argv[i]);
      i++;
    }

  while (i < argc)
    {
      if (strcmp(argv[i], "--max-turn") == 0 && i + 1 < argc)
        {
          max_turn = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc)
        {
          hz = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--ki") == 0 && i + 1 < argc)
        {
          if (parse_gain(argv[i + 1], &ki_x100) < 0)
            {
              fprintf(stderr,
                      "linetrace: --ki must be in [-100.00, 100.00]\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--kd") == 0 && i + 1 < argc)
        {
          if (parse_gain(argv[i + 1], &kd_x100) < 0)
            {
              fprintf(stderr,
                      "linetrace: --kd must be in [-100.00, 100.00]\n");
              return 1;
            }
          i += 2;
        }
      else
        {
          fprintf(stderr, "linetrace: unknown option '%s'\n", argv[i]);
          return 1;
        }
    }

  if (hz < 10 || hz > 200)
    {
      fprintf(stderr, "linetrace: --hz must be in [10, 200]\n");
      return 1;
    }

  if (target < 0 || target > 100)
    {
      fprintf(stderr, "linetrace: target must be in [0, 100]\n");
      return 1;
    }

  if (max_turn < 1 || max_turn > 1000)
    {
      fprintf(stderr, "linetrace: --max-turn must be in [1, 1000]\n");
      return 1;
    }

  g_params.speed_mmps = speed_mmps;
  g_params.kp_x100    = kp_x100;
  g_params.ki_x100    = ki_x100;
  g_params.kd_x100    = kd_x100;
  g_params.target     = target;
  g_params.max_turn   = max_turn;
  g_params.hz         = hz;
  g_pid_reset_request = true;

  printf("linetrace: speed=%d mm/s kp=%.2f ki=%.2f kd=%.2f "
         "target=%d max_turn=%d hz=%d\n",
         speed_mmps, kp_x100 / 100.0, ki_x100 / 100.0, kd_x100 / 100.0,
         target, max_turn, hz);
  return 0;
}

/****************************************************************************
 * Subcommand: brake
 ****************************************************************************/

static int do_brake(void)
{
  if (!g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: not running — 'linetrace start' first\n");
      return 1;
    }

  g_brake_request = true;

  int waited = 0;
  while (g_brake_request && waited < 1000)
    {
      usleep(20000);
      waited += 20;
    }

  if (g_brake_request)
    {
      fprintf(stderr, "linetrace: brake not acknowledged\n");
      return 1;
    }

  printf("linetrace: brake — FOREVER(0,0) + STOP{BRAKE}\n");
  return 0;
}

/****************************************************************************
 * Subcommand: status
 ****************************************************************************/

static int do_status(void)
{
  printf("running:       %s\n", g_daemon_running ? "yes" : "no");
  if (!g_daemon_running)
    {
      return 0;
    }

  printf("pid:           %d\n", g_daemon_pid);
  printf("speed:         %d mm/s\n", g_params.speed_mmps);
  printf("kp:            %.2f\n", g_params.kp_x100 / 100.0);
  printf("ki:            %.2f\n", g_params.ki_x100 / 100.0);
  printf("kd:            %.2f\n", g_params.kd_x100 / 100.0);
  printf("target:        %d\n", g_params.target);
  printf("max_turn:      %d dps\n", g_params.max_turn);
  printf("hz:            %d\n", g_params.hz);
  printf("iter:          %llu\n", (unsigned long long)g_stats.iter);
  printf("last_refl:     %d\n", g_stats.last_refl);
  printf("last_err:      %d\n", g_stats.last_err);
  printf("last_p_term:   %d\n", g_stats.last_p_term);
  printf("last_i_term:   %d\n", g_stats.last_i_term);
  printf("last_d_term:   %d\n", g_stats.last_d_term);
  printf("last_turn_dps: %d\n", g_stats.last_turn_dps);
  printf("last_ioctl_rc: %d (errno=%d)\n",
         g_stats.last_ioctl_rc, g_stats.last_ioctl_errno);
  printf("cal:           %s\n",
         g_cal_request ? "in_flight" : (g_cal_done ? "done" : "idle"));
  return 0;
}

/****************************************************************************
 * usage
 ****************************************************************************/

static void usage(void)
{
  fprintf(stderr,
    "usage: linetrace start\n"
    "       linetrace cal\n"
    "       linetrace run <speed_mmps> <kp> [target] "
    "[--ki K] [--kd K] [--max-turn dps] [--hz N]\n"
    "       linetrace brake\n"
    "       linetrace stop\n"
    "       linetrace status\n"
    "\n"
    "  start:  spawn resident daemon (idle: speed=0 kp=0 target=%d)\n"
    "  cal:    daemon samples 3 s, prints black/white/midpoint\n"
    "          (requires speed=0; use 'linetrace run 0 0' if needed)\n"
    "  run:    update daemon params live\n"
    "          run 100 0 61          straight at 100 mm/s\n"
    "          run 100 3 61          P-control follow with kp=3\n"
    "          run 100 3 61 --ki 0.5 --kd 0.1\n"
    "                                PID follow (kp=3 ki=0.5 kd=0.1)\n"
    "          run 0 0               command stop (daemon stays alive)\n"
    "          gain ranges: kp/ki/kd in [-100.00, 100.00] (0.01 step)\n"
    "          defaults inherit from previous values; first run after\n"
    "          start uses target=%d, --max-turn=%d, --hz=%d\n"
    "  brake:  FOREVER(0,0) + STOP{BRAKE}; reset speed=0 kp=0\n"
    "          daemon stays alive — 'linetrace run ...' to re-engage\n"
    "  stop:   coast wheels, daemon exits\n"
    "  status: print daemon state and last loop iteration values\n",
    DEFAULT_TARGET, DEFAULT_TARGET, DEFAULT_MAX_TURN, DEFAULT_HZ);
}

/****************************************************************************
 * main
 ****************************************************************************/

int main(int argc, FAR char *argv[])
{
  if (argc < 2)
    {
      usage();
      return 0;
    }

  if (strcmp(argv[1], "start") == 0)
    {
      return do_start();
    }

  if (strcmp(argv[1], "stop") == 0)
    {
      return do_stop();
    }

  if (strcmp(argv[1], "cal") == 0)
    {
      return do_cal();
    }

  if (strcmp(argv[1], "run") == 0)
    {
      return do_run(argc - 2, argv + 2);
    }

  if (strcmp(argv[1], "brake") == 0)
    {
      return do_brake();
    }

  if (strcmp(argv[1], "status") == 0)
    {
      return do_status();
    }

  usage();
  return 1;
}
