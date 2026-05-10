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
 * 100 Hz loop reads /dev/uorb/sensor_color (RGB+I mode 5, INT16 channel 4 =
 * intensity, 0..1024), computes a P term from the deviation against
 * `target`, and issues DRIVEBASE_DRIVE_FOREVER each iteration.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
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
#define COLOR_MODE_RGBI     5

#define DEFAULT_TARGET      512
#define DEFAULT_HZ          100

/* Dynamic speed normalization scales (Issue #126).  Hardcoded based on
 * Issue #121 v=300/200Hz observations: peak |err| ~36 (line edge),
 * peak per-tick |derr| ~16 (kd=0.1, d_max~320 / kd_x100 / 10 * dt_ms).
 * ERR_FLOOR=100 represents a "large miss"; DERR_FLOOR=30 is roughly 2x
 * the observed peak so the term does not saturate on noise.  DERR_FLOOR
 * is calibrated for v/hz ~1.5 mm/tick (the recommended profile from
 * Issue #121); if hz is changed dramatically, retune via --v-beta.
 */
#define ERR_FLOOR           100
#define DERR_FLOOR          30

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
  int speed_mmps;     /* upper bound when dynamic speed is engaged */
  int kp_x100;
  int ki_x100;
  int kd_x100;
  int target;
  int hz;
  int v_min_mmps;     /* dynamic speed floor; == speed_mmps disables scaling */
  int v_alpha_x100;   /* coefficient on |err|;  0 disables err contribution  */
  int v_beta_x100;    /* coefficient on |derr|; 0 disables derr contribution */
} g_params;

/* PID observability state (Issue #118).
 *
 * Snapshot fields hold the most recent tick's value (intensity, err, i_acc).
 * Cumulative `sat_count` increments whenever the saturation clamp acts
 * during a PID tick.  Interval-aggregate fields track per-interval
 * statistics (min/max/abs-sum, zero-crossings, abs-max) that the
 * `pidstat` subcommand consumes to defeat aliasing on err/d_term/turn.
 * All accesses are guarded by `g_stats_lock` for row consistency
 * between the daemon writer and the CLI reader; printf runs lock-free
 * on a local snapshot copy.
 */
static struct
{
  uint64_t iter;
  int      last_intensity;
  int      last_err;
  int      last_i_acc;
  int      last_ioctl_rc;
  int      last_ioctl_errno;
  uint32_t sat_count;

  /* Interval aggregates (reset on every CLI snapshot).  Daemon only
   * accumulates while g_pidstat_active is true so 1-interval bound on
   * sums is guaranteed (max ~100 ticks for 1s/100Hz default).
   */
  int      interval_err_min;
  int      interval_err_max;
  int32_t  interval_err_abs_sum;
  uint16_t interval_zc;
  int      interval_prev_err;
  bool     interval_has_prev;
  int32_t  interval_d_abs_max;
  int64_t  interval_d_abs_sum;
  int32_t  interval_turn_abs_max;
  int32_t  interval_turn_abs_sum;
  int      interval_v_min;
  int      interval_v_max;
  int64_t  interval_v_sum;
  uint32_t interval_tick_count;
} g_stats;

static pthread_mutex_t g_stats_lock = PTHREAD_MUTEX_INITIALIZER;
static bool            g_pidstat_active = false;

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

/* Drivebase ownership intent (Issue #117).  Set true on a successful
 * `linetrace run` (any speed/gain combination, including `run 0 0`),
 * cleared on `linetrace brake` and at daemon start.  Sticky — `run
 * 0 0` keeps the drivebase actively held at FOREVER(0, 0) until the
 * user explicitly brakes.  CLI thread writes (do_start / do_run);
 * daemon thread writes from the brake handler.  Single-byte volatile
 * reads/writes are atomic on Cortex-M4 — no lock needed.
 */

static volatile bool g_engaged       = false;
static struct
{
  int min_v;
  int max_v;
  int count;
} g_cal_result;

/****************************************************************************
 * Sensor / drivebase helpers
 ****************************************************************************/

/* RGBI sample shape gate.  data_type / num_values come from the mode-info
 * cache and are decoupled from the actual payload byte count, so we also
 * require `len >= 4 * sizeof(int16_t)` to defend against a short DATA
 * frame leaving s.data.i16[3] zero-filled by legosensor_build_envelope().
 */

static bool color_sample_is_rgbi(const struct lump_sample_s *s)
{
  return s->mode_id    == COLOR_MODE_RGBI &&
         s->data_type  == LUMP_DATA_INT16 &&
         s->num_values >= 4 &&
         s->len        >= 4 * sizeof(int16_t);
}

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

  struct legosensor_select_arg_s sel = { .mode = COLOR_MODE_RGBI };
  if (ioctl(fd, LEGOSENSOR_SELECT, (unsigned long)&sel) < 0)
    {
      fprintf(stderr, "linetrace: SELECT mode %u: %s\n",
              COLOR_MODE_RGBI, strerror(errno));
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
      if (n == sizeof(s) && color_sample_is_rgbi(&s))
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
          COLOR_MODE_RGBI, SELECT_POLL_MS);
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

      if (color_sample_is_rgbi(&s))
        {
          int v = s.data.i16[3];      /* intensity is the 4th channel */
          if (v < 0)    v = 0;
          if (v > 1024) v = 1024;
          last = v;
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
  int min_v = 1024;
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

  int  last_intensity   = -1;
  bool was_engaged = false;   /* drivebase ownership intent (Issue #117) */

  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &next);

  while (!g_daemon_stop)
    {
      if (g_cal_request)
        {
          /* Cal takes ~3 sec.  If we were still engaged when the
           * user fired `linetrace cal` (e.g. immediately after
           * `run 0 0` before the disengage tick had a chance to
           * fire), release the drivebase first so it does not sit
           * on the previous FOREVER command for the cal sweep.
           */

          if (was_engaged)
            {
              drivebase_send_stop(db_fd, DRIVEBASE_ON_COMPLETION_COAST);
              was_engaged = false;
            }
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
          /* Honor the explicit BRAKE; clear ownership + edge tracker
           * so the next tick's idle-edge path does not overwrite
           * the BRAKE with COAST.
           */
          g_engaged           = false;
          was_engaged         = false;
          clock_gettime(CLOCK_MONOTONIC, &next);
          continue;
        }

      /* Engagement gate (Issue #117): when not engaged (initial state
       * after `start`, or after `brake`), stop hammering the drivebase
       * chardev with FOREVER(0,0) every tick.  Send STOP{COAST} once
       * on the disengage edge so the user can issue unrelated
       * drivebase commands without contention.  `run 0 0` is an
       * explicit "active hold at zero" and stays engaged — the user
       * must `brake` to release.
       */

      if (!g_engaged)
        {
          if (was_engaged)
            {
              drivebase_send_stop(db_fd, DRIVEBASE_ON_COMPLETION_COAST);
              was_engaged = false;
            }

          /* Idle ticks still refresh sensor reading + err so the user
           * can use `linetrace status` to inspect intensity and
           * error against the current target (e.g. positioning the
           * robot over the line before `run`).  PID outputs are
           * forced to 0 so status clearly indicates "no drive output"
           * instead of showing stale values from the last engaged
           * tick.
           */

          int v = color_read_latest(color_fd);
          if (v >= 0)
            {
              last_intensity = v;
            }
          int err = (last_intensity >= 0) ? (g_params.target - last_intensity) : 0;

          pthread_mutex_lock(&g_stats_lock);
          g_stats.iter++;
          g_stats.last_intensity  = last_intensity;
          g_stats.last_err   = err;
          g_stats.last_i_acc = 0;

          if (g_pidstat_active)
            {
              /* Idle: PID outputs are zero; only err contributes to
               * interval aggregates.  d_term/turn_dps additions are
               * unconditional max-update + 0-sum, kept symmetric with
               * the engaged path for clarity.
               */

              if (err < g_stats.interval_err_min)
                  g_stats.interval_err_min = err;
              if (err > g_stats.interval_err_max)
                  g_stats.interval_err_max = err;
              int64_t err_abs = (err >= 0) ? (int64_t)err : -(int64_t)err;
              g_stats.interval_err_abs_sum += (int32_t)err_abs;
              if (g_stats.interval_has_prev)
                {
                  int p_sign = (g_stats.interval_prev_err > 0) -
                               (g_stats.interval_prev_err < 0);
                  int c_sign = (err > 0) - (err < 0);
                  if (p_sign != 0 && c_sign != 0 && p_sign != c_sign &&
                      g_stats.interval_zc < UINT16_MAX)
                    {
                      g_stats.interval_zc++;
                    }
                }
              g_stats.interval_prev_err = err;
              g_stats.interval_has_prev = true;
              g_stats.interval_tick_count++;
            }
          pthread_mutex_unlock(&g_stats_lock);
          goto sleep_to_next_tick;
        }

      was_engaged = true;

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

      int s_in = color_read_latest(color_fd);
      if (s_in >= 0)
        {
          last_intensity = s_in;
        }

      int err    = (last_intensity >= 0) ? (g_params.target - last_intensity) : 0;
      int dt_ms  = 1000 / g_params.hz;       /* hz validated 10..200    */
      int p_term = (g_params.kp_x100 * err) / 100;

      /* derr exposed at top scope so dynamic-speed and d_term share the
       * same per-tick differential.  has_prev is the legacy first-sample
       * suppressor — when false, derr_v stays 0 and feeds both d_term
       * and dynamic-speed neutrally.
       */

      int  derr_v           = 0;
      bool has_prev_for_derr = g_pid_has_prev;
      if (has_prev_for_derr)
        {
          derr_v = err - g_pid_prev_err;
        }

      /* I-term initial integration + conservative clamp using
       * speed_mmps as upper bound for the dynamic-max_turn anti-windup.
       * Step 7 below re-clamps with the (tighter) max_turn_apply once
       * dynamic speed is known.  The two-stage clamp keeps i_acc within
       * int32 even when ki_x100 is large and err spikes, without
       * having to reorder the integrator past dynamic speed compute.
       * ki_x100 is validated to ±10000 in do_run; ki=0 short-circuits
       * to i_acc=0 to avoid /0 in i_limit.
       */

      g_pid_i_acc += err * dt_ms;
      int i_term = 0;
      if (g_params.ki_x100 != 0)
        {
          int ki_abs  = (g_params.ki_x100 < 0) ?
                        -g_params.ki_x100 : g_params.ki_x100;
          int i_limit = (g_params.speed_mmps * 100 * 1000) / ki_abs;
          g_pid_i_acc = clamp_int(g_pid_i_acc, -i_limit, i_limit);
          i_term      = (int)(((int64_t)g_params.ki_x100 * g_pid_i_acc) /
                              (100 * 1000));
        }
      else
        {
          g_pid_i_acc = 0;
        }

      /* D-term reuses derr_v.  has_prev=false leaves derr_v=0 so the
       * first-sample suppressor still works.
       */

      int d_term = 0;
      if (has_prev_for_derr && dt_ms > 0)
        {
          d_term = (int)(((int64_t)g_params.kd_x100 * derr_v * 10) /
                         dt_ms);
        }
      g_pid_prev_err = err;
      g_pid_has_prev = true;

      /* Dynamic speed (Issue #126): scale base speed down by |err| +
       * |derr| controller demand.  Gated on (speed>0) and at least one
       * non-zero gain and v_min<speed; otherwise speed_apply == base
       * (legacy fixed-speed behavior).  Formula:
       *
       *   denom_x1000 = 1000 + α·|err|·10/ERR_FLOOR + β·|derr|·10/DERR_FLOOR
       *   v = (base · 1000) / denom_x1000, clamped to [v_min, base]
       *
       * α and β are x100 fixed-point in [0, 10000], validated by do_run.
       */

      int speed_apply = g_params.speed_mmps;
      if (speed_apply > 0 &&
          (g_params.v_alpha_x100 != 0 || g_params.v_beta_x100 != 0) &&
          g_params.v_min_mmps < g_params.speed_mmps)
        {
          int abs_err  = (err    >= 0) ? err    : -err;
          int abs_derr = (derr_v >= 0) ? derr_v : -derr_v;
          int64_t denom_x1000 = 1000
            + ((int64_t)g_params.v_alpha_x100 * abs_err  * 10) / ERR_FLOOR
            + ((int64_t)g_params.v_beta_x100  * abs_derr * 10) / DERR_FLOOR;
          int v_dyn = (int)(((int64_t)g_params.speed_mmps * 1000) /
                            denom_x1000);
          if (v_dyn < g_params.v_min_mmps) v_dyn = g_params.v_min_mmps;
          if (v_dyn > g_params.speed_mmps) v_dyn = g_params.speed_mmps;
          speed_apply = v_dyn;
        }

      /* Dynamic max_turn: tracks speed_apply 1:1 (Issue #121 empirical
       * scaling law max_turn ≈ v).  speed_apply==0 ⇒ max_turn_apply==0
       * ⇒ turn_dps clamps to 0 ⇒ FOREVER(0,0) (legacy `run 0 0` stop
       * semantics).  ki/i_limit guard below also skips when
       * max_turn_apply==0 to avoid /0.
       */

      int max_turn_apply = (speed_apply > 0) ? speed_apply : 0;

      /* Re-clamp i_acc with the tighter dynamic limit + recompute i_term
       * so this tick's authority matches the (possibly reduced) cap.
       * derivative_kick is not introduced — i_term is continuous in
       * i_acc, so a snap of i_acc just shifts i_term by the same factor.
       */

      if (g_params.ki_x100 != 0 && max_turn_apply > 0)
        {
          int ki_abs  = (g_params.ki_x100 < 0) ?
                        -g_params.ki_x100 : g_params.ki_x100;
          int i_limit = (max_turn_apply * 100 * 1000) / ki_abs;
          g_pid_i_acc = clamp_int(g_pid_i_acc, -i_limit, i_limit);
          i_term      = (int)(((int64_t)g_params.ki_x100 * g_pid_i_acc) /
                              (100 * 1000));
        }
      else if (g_params.ki_x100 == 0)
        {
          g_pid_i_acc = 0;
          i_term      = 0;
        }

      int sum_dps  = (int)((int64_t)p_term + i_term + d_term);
      int turn_dps = clamp_int(sum_dps, -max_turn_apply, max_turn_apply);
      int saturated = (sum_dps != turn_dps) ? 1 : 0;

      int rc = drivebase_send_forever(db_fd, speed_apply, turn_dps);

      pthread_mutex_lock(&g_stats_lock);
      g_stats.iter++;
      g_stats.last_intensity        = last_intensity;
      g_stats.last_err         = err;
      g_stats.last_i_acc       = g_pid_i_acc;
      g_stats.last_ioctl_rc    = rc;
      g_stats.last_ioctl_errno = (rc < 0) ? errno : 0;
      g_stats.sat_count       += (uint32_t)saturated;

      if (g_pidstat_active)
        {
          /* abs via 64-bit cast guards INT_MIN UB; sum stays in int32
           * range because pidstat-only gating bounds 1 interval to
           * 100*hz_max ticks (default <=100).
           */

          int64_t err_abs  = (err >= 0)      ? (int64_t)err
                                             : -(int64_t)err;
          int64_t d_abs    = (d_term >= 0)   ? (int64_t)d_term
                                             : -(int64_t)d_term;
          int64_t turn_abs = (turn_dps >= 0) ? (int64_t)turn_dps
                                             : -(int64_t)turn_dps;

          if (err < g_stats.interval_err_min)
              g_stats.interval_err_min = err;
          if (err > g_stats.interval_err_max)
              g_stats.interval_err_max = err;
          g_stats.interval_err_abs_sum += (int32_t)err_abs;

          if (g_stats.interval_has_prev)
            {
              int p_sign = (g_stats.interval_prev_err > 0) -
                           (g_stats.interval_prev_err < 0);
              int c_sign = (err > 0) - (err < 0);
              if (p_sign != 0 && c_sign != 0 && p_sign != c_sign &&
                  g_stats.interval_zc < UINT16_MAX)
                {
                  g_stats.interval_zc++;
                }
            }
          g_stats.interval_prev_err = err;
          g_stats.interval_has_prev = true;

          if ((int32_t)d_abs > g_stats.interval_d_abs_max)
              g_stats.interval_d_abs_max = (int32_t)d_abs;
          g_stats.interval_d_abs_sum += d_abs;

          if ((int32_t)turn_abs > g_stats.interval_turn_abs_max)
              g_stats.interval_turn_abs_max = (int32_t)turn_abs;
          g_stats.interval_turn_abs_sum += (int32_t)turn_abs;

          if (speed_apply < g_stats.interval_v_min)
              g_stats.interval_v_min = speed_apply;
          if (speed_apply > g_stats.interval_v_max)
              g_stats.interval_v_max = speed_apply;
          g_stats.interval_v_sum += speed_apply;

          g_stats.interval_tick_count++;
        }
      pthread_mutex_unlock(&g_stats_lock);

    sleep_to_next_tick:
      {
        /* Block-scoped: a label may not directly precede a
         * declaration in C, so wrap period_ns in a compound stmt.
         * Idle ticks reach here via `goto` to keep the
         * clock_nanosleep absolute-deadline ladder intact.
         */

        long period_ns = 1000000000L / g_params.hz;
        next.tv_nsec += period_ns;
        while (next.tv_nsec >= 1000000000L)
          {
            next.tv_nsec -= 1000000000L;
            next.tv_sec  += 1;
          }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
      }
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

  g_params.speed_mmps   = 0;
  g_params.kp_x100      = 0;
  g_params.ki_x100      = 0;
  g_params.kd_x100      = 0;
  g_params.target       = DEFAULT_TARGET;
  g_params.hz           = DEFAULT_HZ;
  g_params.v_min_mmps   = 0;
  g_params.v_alpha_x100 = 0;
  g_params.v_beta_x100  = 0;

  memset(&g_stats, 0, sizeof(g_stats));
  g_stats.last_intensity = -1;

  g_pid_i_acc         = 0;
  g_pid_prev_err      = 0;
  g_pid_has_prev      = false;
  g_pid_reset_request = false;
  g_engaged           = false;

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
         "dark/bright\n", CAL_DURATION_MS);

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
  printf("linetrace: cal %d samples: dark=%d bright=%d midpoint=%d\n",
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
              "[--ki K] [--kd K] [--hz N] "
              "[--v-min mm/s] [--v-alpha A] [--v-beta B]\n");
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
  int  hz         = g_params.hz;

  /* Dynamic-speed flags reset to no-op every run.  Unlike --ki/--kd/
   * --hz which inherit prior values, --v-* are an opt-in safety switch
   * — silently carrying over a dynamic profile from a previous run is a
   * footgun (e.g. user types `run 200 3.6` expecting fixed speed and
   * the daemon keeps a stale --v-min from yesterday's tuning session).
   */

  int  v_min_mmps   = speed_mmps;   /* default: dynamic OFF */
  int  v_alpha_x100 = 0;
  int  v_beta_x100  = 0;
  int  i            = 2;

  if (i < argc && argv[i][0] != '-')
    {
      target = atoi(argv[i]);
      i++;
    }

  while (i < argc)
    {
      if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc)
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
      else if (strcmp(argv[i], "--v-min") == 0 && i + 1 < argc)
        {
          v_min_mmps = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--v-alpha") == 0 && i + 1 < argc)
        {
          if (parse_gain(argv[i + 1], &v_alpha_x100) < 0 ||
              v_alpha_x100 < 0)
            {
              fprintf(stderr,
                      "linetrace: --v-alpha must be in [0.00, 100.00]\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--v-beta") == 0 && i + 1 < argc)
        {
          if (parse_gain(argv[i + 1], &v_beta_x100) < 0 ||
              v_beta_x100 < 0)
            {
              fprintf(stderr,
                      "linetrace: --v-beta must be in [0.00, 100.00]\n");
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

  if (target < 0 || target > 1024)
    {
      fprintf(stderr, "linetrace: target must be in [0, 1024]\n");
      return 1;
    }

  /* v_min validation: only meaningful when speed>0.  speed==0 means
   * "stop motion" and dynamic-speed flags are silently ignored to keep
   * `linetrace run 0 0` semantics intact.  v_min=0 is rejected because
   * it would let the controller stop the robot in tight curves —
   * separate semantics that deserve their own flag if ever needed.
   */

  if (speed_mmps > 0)
    {
      if (v_min_mmps < 1 || v_min_mmps > speed_mmps)
        {
          fprintf(stderr,
                  "linetrace: --v-min must be in [1, %d]\n",
                  speed_mmps);
          return 1;
        }
    }
  else
    {
      v_min_mmps   = 0;
      v_alpha_x100 = 0;
      v_beta_x100  = 0;
    }

  g_params.speed_mmps   = speed_mmps;
  g_params.kp_x100      = kp_x100;
  g_params.ki_x100      = ki_x100;
  g_params.kd_x100      = kd_x100;
  g_params.target       = target;
  g_params.hz           = hz;
  g_params.v_min_mmps   = v_min_mmps;
  g_params.v_alpha_x100 = v_alpha_x100;
  g_params.v_beta_x100  = v_beta_x100;
  g_pid_reset_request   = true;
  g_engaged             = true;

  printf("linetrace: speed=%d mm/s kp=%.2f ki=%.2f kd=%.2f "
         "target=%d hz=%d v_min=%d v_alpha=%.2f v_beta=%.2f\n",
         speed_mmps, kp_x100 / 100.0, ki_x100 / 100.0, kd_x100 / 100.0,
         target, hz, v_min_mmps,
         v_alpha_x100 / 100.0, v_beta_x100 / 100.0);
  return 0;
}

/****************************************************************************
 * Subcommand: target (Issue #119)
 *
 * Mutate the intensity setpoint without engaging the drivebase.  Lets
 * the user set the operational threshold before `run`, so
 * `linetrace status` shows last_err against the real target while
 * positioning the robot.  The `max_turn` mutator that originally
 * shipped with #119 was retired in #126: max_turn is now derived per
 * tick from `speed_apply` (Issue #121's max_turn ≈ v scaling rule).
 ****************************************************************************/

static int parse_int_arg(const char *tag, char **argv, int argc,
                         int *out)
{
  if (argc != 1)
    {
      fprintf(stderr, "usage: linetrace %s <value>\n", tag);
      return -1;
    }

  char *end = NULL;
  long  v   = strtol(argv[0], &end, 10);
  if (end == argv[0] || *end != '\0' || v < INT_MIN || v > INT_MAX)
    {
      fprintf(stderr, "linetrace: %s value must be an integer\n", tag);
      return -1;
    }

  *out = (int)v;
  return 0;
}

static int do_set_target(int argc, char **argv)
{
  if (!g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: not running — 'linetrace start' first\n");
      return 1;
    }

  int target_new;
  if (parse_int_arg("target", argv, argc, &target_new) < 0)
    {
      return 1;
    }

  if (target_new < 0 || target_new > 1024)
    {
      fprintf(stderr, "linetrace: target must be in [0, 1024]\n");
      return 1;
    }

  g_params.target = target_new;
  printf("linetrace: target=%d\n", g_params.target);
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
 * Subcommand: pidstat
 ****************************************************************************/

static int do_pidstat(int argc, char **argv)
{
  long duration_ms = 0;
  long interval_ms = 1000;

  /* Argument parsing */

  if (argc >= 1)
    {
      char *end;
      long v = strtol(argv[0], &end, 10);
      if (*end != '\0' || v < 0)
        {
          fprintf(stderr,
                  "linetrace: pidstat duration_ms must be >= 0\n");
          return 1;
        }
      duration_ms = v;
    }
  if (argc >= 2)
    {
      char *end;
      long v = strtol(argv[1], &end, 10);
      if (*end != '\0' || v <= 0)
        {
          fprintf(stderr,
                  "linetrace: pidstat interval_ms must be > 0\n");
          return 1;
        }
      interval_ms = v;
    }
  if (argc > 2)
    {
      fprintf(stderr,
              "usage: linetrace pidstat [duration_ms [interval_ms]]\n");
      return 1;
    }

  int min_interval_ms = (g_params.hz > 0) ? (1000 / g_params.hz) : 10;
  if (min_interval_ms < 1)
    {
      min_interval_ms = 1;
    }
  if (duration_ms > 0 && interval_ms < min_interval_ms)
    {
      fprintf(stderr,
              "linetrace: interval_ms %ld too small "
              "(min %d ms for hz=%d)\n",
              interval_ms, min_interval_ms, g_params.hz);
      return 1;
    }

  /* Daemon check + interval reset + activate (atomic under lock).
   * The lock guarantees the daemon's next publish will see the
   * cleared aggregates and the active flag together.
   */

  pthread_mutex_lock(&g_stats_lock);
  if (!g_daemon_running)
    {
      pthread_mutex_unlock(&g_stats_lock);
      fprintf(stderr, "linetrace: daemon not running\n");
      return 1;
    }
  g_stats.interval_err_min      = INT_MAX;
  g_stats.interval_err_max      = INT_MIN;
  g_stats.interval_err_abs_sum  = 0;
  g_stats.interval_zc           = 0;
  g_stats.interval_has_prev     = false;
  g_stats.interval_d_abs_max    = 0;
  g_stats.interval_d_abs_sum    = 0;
  g_stats.interval_turn_abs_max = 0;
  g_stats.interval_turn_abs_sum = 0;
  g_stats.interval_v_min        = INT_MAX;
  g_stats.interval_v_max        = INT_MIN;
  g_stats.interval_v_sum        = 0;
  g_stats.interval_tick_count   = 0;
  uint32_t begin_sat    = g_stats.sat_count;
  uint32_t begin_iter32 = (uint32_t)(g_stats.iter & 0xFFFFFFFFu);
  g_pidstat_active      = true;
  pthread_mutex_unlock(&g_stats_lock);

  printf("%8s %9s %6s %7s %8s %8s %8s %5s %8s %8s %10s %9s %9s "
         "%7s %7s %7s %6s\n",
         "time_ms", "iter", "intens", "err",
         "err_min", "err_max", "err_avg", "zc",
         "d_max", "d_avg", "i_acc",
         "turn_max", "turn_avg",
         "v_max", "v_avg", "v_min",
         "sat");

  struct timespec t0;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  next = t0;

  uint32_t reported_ticks = 0;
  uint32_t end_iter32     = begin_iter32;
  uint32_t end_sat        = begin_sat;
  long     elapsed_ms     = 0;

  for (;;)
    {
      pthread_mutex_lock(&g_stats_lock);
      bool     running      = g_daemon_running;
      uint64_t iter64       = g_stats.iter;
      int      last_intensity    = g_stats.last_intensity;
      int      last_err     = g_stats.last_err;
      int      last_i_acc   = g_stats.last_i_acc;
      uint32_t cur_sat      = g_stats.sat_count;
      uint32_t n            = g_stats.interval_tick_count;
      int      err_min      = g_stats.interval_err_min;
      int      err_max      = g_stats.interval_err_max;
      int32_t  err_sum      = g_stats.interval_err_abs_sum;
      uint16_t zc           = g_stats.interval_zc;
      int32_t  d_abs_max    = g_stats.interval_d_abs_max;
      int64_t  d_abs_sum    = g_stats.interval_d_abs_sum;
      int32_t  turn_abs_max = g_stats.interval_turn_abs_max;
      int32_t  turn_abs_sum = g_stats.interval_turn_abs_sum;
      int      v_min_iv     = g_stats.interval_v_min;
      int      v_max_iv     = g_stats.interval_v_max;
      int64_t  v_sum_iv     = g_stats.interval_v_sum;
      g_stats.interval_err_min      = INT_MAX;
      g_stats.interval_err_max      = INT_MIN;
      g_stats.interval_err_abs_sum  = 0;
      g_stats.interval_zc           = 0;
      g_stats.interval_d_abs_max    = 0;
      g_stats.interval_d_abs_sum    = 0;
      g_stats.interval_turn_abs_max = 0;
      g_stats.interval_turn_abs_sum = 0;
      g_stats.interval_v_min        = INT_MAX;
      g_stats.interval_v_max        = INT_MIN;
      g_stats.interval_v_sum        = 0;
      g_stats.interval_tick_count   = 0;
      pthread_mutex_unlock(&g_stats_lock);

      struct timespec tn;
      clock_gettime(CLOCK_MONOTONIC, &tn);
      elapsed_ms = (tn.tv_sec - t0.tv_sec) * 1000L +
                   (tn.tv_nsec - t0.tv_nsec) / 1000000L;

      /* Format aggregate columns: '-' when no ticks were captured
       * (idle on entry or before the first daemon publish landed).
       */

      char b_emin[16], b_emax[16], b_eavg[16];
      char b_dmax[16], b_davg[16];
      char b_tmax[16], b_tavg[16];
      char b_vmax[16], b_vavg[16], b_vmin[16];

      if (n == 0)
        {
          snprintf(b_emin, sizeof(b_emin), "%8s", "-");
          snprintf(b_emax, sizeof(b_emax), "%8s", "-");
          snprintf(b_eavg, sizeof(b_eavg), "%8s", "-");
          snprintf(b_dmax, sizeof(b_dmax), "%8s", "-");
          snprintf(b_davg, sizeof(b_davg), "%8s", "-");
          snprintf(b_tmax, sizeof(b_tmax), "%9s", "-");
          snprintf(b_tavg, sizeof(b_tavg), "%9s", "-");
          snprintf(b_vmax, sizeof(b_vmax), "%7s", "-");
          snprintf(b_vavg, sizeof(b_vavg), "%7s", "-");
          snprintf(b_vmin, sizeof(b_vmin), "%7s", "-");
        }
      else
        {
          int err_avg_x10 = (int)(((int64_t)err_sum * 10 +
                                   (int64_t)n / 2) / (int64_t)n);
          int d_avg       = (int)(d_abs_sum / (int64_t)n);
          int turn_avg    = (int)(turn_abs_sum / (int32_t)n);
          int v_avg       = (int)(v_sum_iv / (int64_t)n);
          snprintf(b_emin, sizeof(b_emin), "%8d", err_min);
          snprintf(b_emax, sizeof(b_emax), "%8d", err_max);
          snprintf(b_eavg, sizeof(b_eavg), "%8d", err_avg_x10);
          snprintf(b_dmax, sizeof(b_dmax), "%8ld", (long)d_abs_max);
          snprintf(b_davg, sizeof(b_davg), "%8d", d_avg);
          snprintf(b_tmax, sizeof(b_tmax), "%9ld", (long)turn_abs_max);
          snprintf(b_tavg, sizeof(b_tavg), "%9d", turn_avg);
          snprintf(b_vmax, sizeof(b_vmax), "%7d", v_max_iv);
          snprintf(b_vavg, sizeof(b_vavg), "%7d", v_avg);
          snprintf(b_vmin, sizeof(b_vmin), "%7d", v_min_iv);
        }

      printf("%8ld %9lu %6d %7d %s %s %s %5u %s %s %10d %s %s %s %s %s "
             "%6lu\n",
             elapsed_ms,
             (unsigned long)(iter64 & 0xFFFFFFFFu),
             last_intensity, last_err,
             b_emin, b_emax, b_eavg, (unsigned)zc,
             b_dmax, b_davg,
             last_i_acc,
             b_tmax, b_tavg,
             b_vmax, b_vavg, b_vmin,
             (unsigned long)(cur_sat - begin_sat));

      reported_ticks += n;
      end_iter32      = (uint32_t)(iter64 & 0xFFFFFFFFu);
      end_sat         = cur_sat;

      if (!running)
        {
          printf("# pidstat: daemon stopped\n");
          break;
        }

      if (duration_ms == 0 || elapsed_ms >= duration_ms)
        {
          break;
        }

      long period_ns = interval_ms * 1000000L;
      next.tv_nsec += period_ns;
      while (next.tv_nsec >= 1000000000L)
        {
          next.tv_nsec -= 1000000000L;
          next.tv_sec  += 1;
        }
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL);
    }

  pthread_mutex_lock(&g_stats_lock);
  g_pidstat_active = false;
  pthread_mutex_unlock(&g_stats_lock);

  if (duration_ms > 0)
    {
      uint32_t expected =
          (uint32_t)((duration_ms * (long)g_params.hz) / 1000L);
      printf("# pidstat: sat=%lu iter=%lu..%lu duration_ms=%ld "
             "reported_ticks=%lu expected=%lu\n",
             (unsigned long)(end_sat - begin_sat),
             (unsigned long)begin_iter32,
             (unsigned long)end_iter32,
             elapsed_ms,
             (unsigned long)reported_ticks,
             (unsigned long)expected);
    }

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

  pthread_mutex_lock(&g_stats_lock);
  uint64_t iter        = g_stats.iter;
  int      last_intensity   = g_stats.last_intensity;
  int      last_err    = g_stats.last_err;
  int      ioctl_rc    = g_stats.last_ioctl_rc;
  int      ioctl_errno = g_stats.last_ioctl_errno;
  pthread_mutex_unlock(&g_stats_lock);

  printf("pid:           %d\n", g_daemon_pid);
  printf("speed:         %d mm/s\n", g_params.speed_mmps);
  printf("engaged:       %s\n", g_engaged ? "yes" : "no");
  printf("kp:            %.2f\n", g_params.kp_x100 / 100.0);
  printf("ki:            %.2f\n", g_params.ki_x100 / 100.0);
  printf("kd:            %.2f\n", g_params.kd_x100 / 100.0);
  printf("target:        %d\n", g_params.target);
  printf("hz:            %d\n", g_params.hz);
  printf("v_min:         %d mm/s\n", g_params.v_min_mmps);
  printf("v_alpha:       %.2f\n", g_params.v_alpha_x100 / 100.0);
  printf("v_beta:        %.2f\n", g_params.v_beta_x100 / 100.0);
  printf("iter:          %llu\n", (unsigned long long)iter);
  printf("last_intensity: %d\n", last_intensity);
  printf("last_err:      %d\n", last_err);
  printf("last_ioctl_rc: %d (errno=%d)\n", ioctl_rc, ioctl_errno);
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
    "[--ki K] [--kd K] [--hz N]\n"
    "                     [--v-min mm/s] [--v-alpha A] [--v-beta B]\n"
    "       linetrace target <N>\n"
    "       linetrace brake\n"
    "       linetrace stop\n"
    "       linetrace pidstat [duration_ms [interval_ms]]\n"
    "       linetrace status\n"
    "\n"
    "  start:   spawn resident daemon (idle: speed=0 kp=0 target=%d)\n"
    "  cal:     daemon samples 3 s, prints dark/bright/midpoint\n"
    "           (requires speed=0; use 'linetrace run 0 0' if needed)\n"
    "  run:     update daemon params live\n"
    "           run 100 0 512         straight at 100 mm/s\n"
    "           run 100 0.3 512       P-control follow with kp=0.3\n"
    "           run 100 0.3 512 --ki 0.05 --kd 0.01\n"
    "                                 PID follow (kp=0.3 ki=0.05 kd=0.01)\n"
    "           run 300 3.6 512 --v-min 150 --v-alpha 1.0 --v-beta 0.5\n"
    "                                 dynamic speed: 150-300 mm/s,\n"
    "                                 max_turn auto-tracks speed\n"
    "           run 0 0               command stop (daemon stays alive)\n"
    "           gain ranges: kp/ki/kd in [-100.00, 100.00] (0.01 step)\n"
    "           v-alpha/v-beta in [0.00, 100.00]; v-min in [1, speed]\n"
    "           kp/ki/kd/target/hz inherit from previous run;\n"
    "           v-min/v-alpha/v-beta reset to 'dynamic OFF' every run\n"
    "           defaults: target=%d hz=%d (first run after start)\n"
    "  target:  set target intensity without engaging the drivebase\n"
    "           (e.g. 'target 400' so 'status' shows last_err against\n"
    "           the operational target while positioning).  [0, 1024]\n"
    "  brake:   FOREVER(0,0) + STOP{BRAKE}; reset speed=0 kp=0\n"
    "           daemon stays alive — 'linetrace run ...' to re-engage\n"
    "  stop:    coast wheels, daemon exits\n"
    "  pidstat: stream PID internals for gain tuning (default 1000ms)\n"
    "           intensity/err/i_acc are snapshots; err_min/max/avg, zc,\n"
    "           d_max/avg, turn_max/avg, v_max/avg/v_min are aggregates\n"
    "           over the preceding interval.  err_avg is x10 fixed-point.\n"
    "           sat is cumulative delta from pidstat start.  '-'\n"
    "           appears when no ticks landed in the interval.\n"
    "           NOTE: do not issue run/brake while pidstat runs;\n"
    "           a 1-interval engaged/idle straddle dilutes averages.\n"
    "           summary's reported_ticks counts only printed intervals\n"
    "           (a partial interval at end-of-stream is dropped).\n"
    "  status:  print daemon state and last err/intensity/i_acc snapshot\n",
    DEFAULT_TARGET, DEFAULT_TARGET, DEFAULT_HZ);
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

  if (strcmp(argv[1], "pidstat") == 0)
    {
      return do_pidstat(argc - 2, argv + 2);
    }

  if (strcmp(argv[1], "target") == 0)
    {
      return do_set_target(argc - 2, argv + 2);
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
