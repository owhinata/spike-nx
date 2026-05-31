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

/* LQG path (Issue #169 / P1a) runs sinf/sqrtf at the control rate (100 Hz).
 * Guard against a misconfigured board silently falling back to soft-float.
 * Mirror the production two-clause form already used by the 500 Hz IMU
 * fusion daemon (apps/drivebase/drivebase_imu.c): the reliable libm symbol
 * on this board is CONFIG_LIBM_NEWLIB, with CONFIG_LIBM as the generic
 * fallback (this board sets only CONFIG_LIBM_NEWLIB).
 */

#if !defined(CONFIG_ARCH_FPU)
#  error "linetrace LQG requires CONFIG_ARCH_FPU (sinf/sqrtf at 100 Hz)"
#endif
#if !defined(CONFIG_LIBM_NEWLIB) && !defined(CONFIG_LIBM)
#  error "linetrace LQG requires libm (CONFIG_LIBM_NEWLIB)"
#endif

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <signal.h>
#include <stdatomic.h>
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

#ifdef CONFIG_APP_CAPTURE
#  include "capture.h"
#  include "capture_schema_linetrace_lap_run.h"
#endif

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

/* LQG controller (Issue #169 / P1a) ------------------------------------- */

#define PI_F                3.14159265358979323846f

/* Compiled LQG defaults — sim-locked against tools/lqr_linetrace/lqr_sim.py
 * at the bench c=150 counts/mm, L=52 mm (CARE residual ~1e-13, zeta 0.707
 * all speeds; LQG IAE 0.6/0.3 vs PID 1.9/3.4, 0% overshoot).  All
 * CLI-overridable; c/L are calibrated against the P0a sensor angle.
 */

#define LQG_DEF_C           150.0f   /* |intensity slope| counts/mm        */
#define LQG_DEF_L           52.0f    /* sensor look-ahead mm               */
#define LQG_EDGE_LEFT       (-1)
#define LQG_EDGE_RIGHT      (+1)
#define LQG_DEF_EDGE        LQG_EDGE_RIGHT
#define LQG_DEF_Q1R         0.888f   /* q1/r (r normalised to 1)           */
#define LQG_DEF_Q2          0.0f     /* heading weight (0 -> zeta 0.707)   */
#define LQG_DEF_KF_QPOS     1.0f     /* Kalman Q[0][0]                     */
#define LQG_DEF_KF_QHEAD    1.0e-4f  /* Kalman Q[1][1]                     */
#define LQG_DEF_KF_RMEAS    49.0f    /* Kalman R (scalar)                  */
#define LQG_DEF_SAT_BAND    3.4f     /* |ey_hat| linear-band half-width mm */
#define LQG_DEF_LOST_TICKS  5        /* (retained for CLI compat; LOST now */
                                     /* triggers on out-of-band DURATION)  */
#define LQG_DEF_LOST_TMO_MS 500      /* LOST seek duration before auto-stop*/

/* LQG-recommended operating target (Issue #169 HW-verify fix).  P0a
 * measured the edge midpoint at ~634 (black ~249 / white ~1020).  The PID
 * shares DEFAULT_TARGET=512; the LQG observer needs the SYMMETRIC midpoint
 * so the linear band is not asymmetrically truncated on the black side
 * (at 512 the black rail sits only ~1.7 mm off-line vs ~3.4 mm of band, so
 * a normal look-ahead-induced spot excursion saturates and the observer
 * loses the line — proven in tools/lqr_linetrace/lqr_sim.py --report
 * satband).  do_lqg defaults to this when no target arg is given; PID is
 * untouched.
 */

#define LQG_DEF_TARGET      634
#define LQG_DEF_BLACK       249      /* measured BLACK intensity floor      */
#define LQG_DEF_WHITE       1020     /* measured WHITE intensity ceiling    */
#define LQG_DEF_BAND_MARGIN 100      /* counts inside black/white = sat     */

/* Out-of-band DURATION before declaring LOST (Issue #169 HW-verify fix).
 * The previous design counted CONSECUTIVE saturated ticks (sat_streak),
 * but a SAT<->NORMAL flap on a near-rail measurement reset the streak so
 * LOST never engaged on HW.  We accumulate saturated TIME instead; once it
 * exceeds this, LOST + seek + auto-stop engage.  Defaults to lost_ticks
 * worth of ms at the active rate.
 */

#define LQG_DEF_LOST_TIME_MS 50      /* saturated-time -> LOST (ms)        */

/* Kalman gain table grid (Riccati is NEVER run per tick; do_lqg builds the
 * whole table, the loop linear-interpolates).  Kf varies only ~1% across
 * 50..400 mm/s, so 24 points + linear is far finer than needed.
 */

#define LQG_KF_PTS          24
#define LQG_V_LO            50       /* mm/s, table lower bound            */
#define LQG_V_HI            400      /* mm/s, table upper bound            */
#define LQG_KF_ITERS        4000     /* Riccati iteration cap              */
#define LQG_KF_EPS          1.0e-7f  /* Riccati convergence threshold      */

/* Intensity rail thresholds — hard sensor rails (kept for reference).  The
 * observer no longer keys saturation off these alone: at the operating
 * target the look-ahead intensity leaves the LINEAR band well before the
 * hard rails (a near-white 975 is not a valid measurement), so saturation
 * is detected by the band-margin test [black+margin, white-margin] instead
 * (Issue #169 HW-verify fix; plan §4).
 */

#define SAT_RAIL_LO         8
#define SAT_RAIL_HI         1016

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
 * Lap capture FSM (Issue #166)
 *
 * Records one packed `capture_linetrace_lap_run_record_s` per loop tick
 * (idle or engaged) into a heap buffer armed by the CLI, then exports it
 * through the existing /dev/btcap pipeline as a .cap file.  This block is
 * compiled only when the capture library is available; the `linetrace
 * cap` verbs report "capture not built" otherwise.
 ****************************************************************************/

#ifdef CONFIG_APP_CAPTURE

#define CAP_RECORD_SIZE  (sizeof(struct capture_linetrace_lap_run_record_s))

/* Heap budget honours both the roadmap "<= 64 KB" cap AND the capture
 * subsystem's shared CONFIG_APP_CAPTURE_MAX_HEAP_BYTES budget: take the
 * smaller of the two so lowering the config knob cannot let linetrace
 * exceed it.  64 KiB / 19 B = 3449 records; at the 100 Hz default loop
 * that is ~34 s of lap.
 */

#ifdef CONFIG_APP_CAPTURE_MAX_HEAP_BYTES
#  define CAP_MAX_BYTES  ((65536u < CONFIG_APP_CAPTURE_MAX_HEAP_BYTES) ? \
                          65536u : (unsigned)CONFIG_APP_CAPTURE_MAX_HEAP_BYTES)
#else
#  define CAP_MAX_BYTES  65536u
#endif

#define CAP_MAX_RECORDS  (CAP_MAX_BYTES / CAP_RECORD_SIZE)

/* edge sentinel: P0b has no edge concept yet (P1a sets LEFT/RIGHT). */

#define CAP_EDGE_UNSET   0

enum linetrace_cap_state_e
{
  LT_CAP_IDLE = 0,
  LT_CAP_ARMED,
  LT_CAP_DONE,
  LT_CAP_EXPORTING,
};

static struct
{
  enum linetrace_cap_state_e state;
  uint8_t  *buf;          /* malloc'd on arm, freed on idle/abort      */
  uint32_t  capacity;     /* records the buffer can hold               */
  uint32_t  count;        /* records appended so far                   */
  uint64_t  t0_us;        /* CLOCK_BOOTTIME at arm, for ts_us delta    */
  int       last_error;   /* errno surfaced to status / export verb    */
  uint8_t   overflow;     /* 1 once a full buffer auto-stopped capture */
  pid_t     export_pid;   /* task that owns the buffer while EXPORTING; */
                          /* -1 otherwise.  Used to detect a hard-     */
                          /* killed exporter and reclaim the buffer.   */
} g_cap;

static pthread_mutex_t g_cap_lock = PTHREAD_MUTEX_INITIALIZER;

/* Lock-free "capture is armed" hint.  The daemon's per-tick append fast
 * path reads this WITHOUT taking g_cap_lock so an idle (not-armed)
 * capture adds no lock contention / jitter to the 100 Hz control loop
 * (memory feedback_observability_jitter).  It is set true only after the
 * buffer is fully armed, and cleared on every transition out of ARMED.
 * The hint can momentarily lag the true state, so the append path still
 * re-checks `state == LT_CAP_ARMED` under the lock before any memcpy —
 * the hint is a fast-path filter, never the authority.  Single-byte
 * volatile reads/writes are atomic on Cortex-M4.
 */

static volatile bool g_cap_armed_hint = false;

/* SIGINT/SIGTERM abort flag for the export verb.  Flag-only handler
 * (signal-async-safe).  Installing the handlers is also what makes a
 * `kill <pid>` (SIGTERM) actually interrupt the blocking capture
 * syscalls — NuttX has no default SIGTERM terminate action.  The export
 * verb checks the flag before each blocking capture stage and winds down
 * via capture_abort if set.
 */

static volatile sig_atomic_t g_cap_export_aborting;

static void cap_export_sig_handler(int sig)
{
  (void)sig;
  g_cap_export_aborting = 1;
}

static const char *cap_state_str(enum linetrace_cap_state_e s)
{
  switch (s)
    {
      case LT_CAP_IDLE:      return "idle";
      case LT_CAP_ARMED:     return "armed";
      case LT_CAP_DONE:      return "done";
      case LT_CAP_EXPORTING: return "exporting";
      default:               return "?";
    }
}

/* Caller holds g_cap_lock.  Free the buffer (if any) and reset all
 * capture metadata to the IDLE baseline, recording `err` as last_error.
 * Single point so reap / start / export-cleanup / abort never leave a
 * stale field (e.g. `overflow` or `t0_us`).  Also clears the lock-free
 * armed hint.
 */

static void cap_reset_idle_locked(int err)
{
  free(g_cap.buf);
  g_cap.buf        = NULL;
  g_cap.capacity   = 0;
  g_cap.count      = 0;
  g_cap.t0_us      = 0;
  g_cap.overflow   = 0;
  g_cap.export_pid = -1;
  g_cap.last_error = err;
  g_cap.state      = LT_CAP_IDLE;
  g_cap_armed_hint = false;
}

/* Caller holds g_cap_lock.  If a capture is stuck in EXPORTING but the
 * task that owned it is gone (hard-killed before its cleanup ran), free
 * the buffer and return to IDLE so the FSM is never wedged.  kill(pid, 0)
 * is the standard liveness probe (-1/ESRCH when the task no longer
 * exists).  Only the no-cleanup SIGKILL path needs this; a handled
 * SIGINT/SIGTERM frees via the export verb's own cleanup.
 */

static void cap_reap_dead_exporter_locked(void)
{
  if (g_cap.state == LT_CAP_EXPORTING && g_cap.export_pid > 0 &&
      kill(g_cap.export_pid, 0) < 0 && errno == ESRCH)
    {
      cap_reset_idle_locked(-ECANCELED);
    }
}

#endif /* CONFIG_APP_CAPTURE */

/****************************************************************************
 * LQG controller state (Issue #169 / P1a)
 ****************************************************************************/

/* Active controller select.  do_run sets CTRL_PID, do_lqg sets CTRL_LQG.
 * Single-byte volatile; barrier-ordered after the param writes so the
 * daemon never reads CTRL_LQG before a config slot is fully published.
 */

enum lt_controller_e { CTRL_PID = 0, CTRL_LQG = 1 };
static volatile uint8_t g_controller = CTRL_PID;

/* Active loop period source: the sleep ladder reads g_loop_hz, NOT
 * g_params.hz, so switching controllers switches the loop rate cleanly.
 * do_run sets it from g_params.hz (so the PID-active period is numerically
 * identical to pre-P1a); do_lqg sets it from the published slot hz.
 */

static volatile int g_loop_hz = DEFAULT_HZ;

/* LQG tunables — written by do_lqg (single CLI writer), validated, then
 * copied into the inactive double-buffered slot and published by an
 * acquire/release atomic generation counter.  Floats kept in physical
 * units (counts/mm, mm, dimensionless q-ratios).  g_lqg is the CLI-side
 * staging/working copy ONLY; the daemon NEVER reads it — it reads the
 * published slot.  (target is the lone live field, read from
 * g_params.target — see do_set_target.)
 */

struct lt_lqg_params_s
{
  int   speed_mmps;        /* base forward speed (fixed; dynamic is P5)  */
  int   target;            /* intensity setpoint (kept for status only)  */
  int   hz;                /* loop rate; drives the active period        */
  float c;                 /* |intensity slope| counts/mm (always > 0)   */
  float L;                 /* look-ahead mm                              */
  int   edge;              /* EDGE_LEFT(-1) or EDGE_RIGHT(+1)            */
  float q1r;               /* q1/r (state-feedback)                      */
  float q2;                /* heading weight (0 -> zeta 0.707 all v)     */
  float kf_qpos;           /* Kalman Q[0][0]                             */
  float kf_qhead;          /* Kalman Q[1][1]                             */
  float kf_rmeas;          /* Kalman R (scalar)                          */
  float sat_band;          /* |ey_p| beyond which we enter SATURATED      */
  int   lost_ticks;        /* (CLI compat; LOST now keys off lost_time_ms)*/
  int   seek_dps;          /* turn rate magnitude while seeking in LOST  */
  int   lost_timeout_ms;   /* LOST seek duration before auto-brake + stop*/
  int   black;             /* measured BLACK intensity floor (counts)    */
  int   white;             /* measured WHITE intensity ceiling (counts)  */
  int   band_margin;       /* counts inside black/white treated as sat   */
  int   lost_time_ms;      /* saturated-time accumulation -> LOST (ms)   */
};
static struct lt_lqg_params_s g_lqg;

/* Observer + SM state — owned by the daemon control loop only. */

enum lt_sat_e { SAT_NORMAL = 0, SAT_SATURATED = 1, SAT_LOST = 2 };
static float    g_lqg_ey;          /* e_y_hat   [mm]                      */
static float    g_lqg_eth;         /* e_theta_hat [rad]                   */
static float    g_lqg_prev_w;      /* last APPLIED omega [rad/s]          */
static int      g_lqg_sat_state;   /* enum lt_sat_e                       */
static int      g_lqg_last_side;   /* sign of last in-band ey_hat (+/-1)  */
static uint32_t g_lqg_lost_cnt;    /* cumulative LOST events              */
static int      g_lqg_lost_ms_acc; /* ms accumulated in current LOST seek */
static int      g_lqg_oob_ms_acc;  /* saturated-time accumulator -> LOST  */

/* Double-buffered precomputed config slot (params + Kalman gain table).
 * Riccati is NEVER run per tick.  do_lqg fills the INACTIVE slot (params +
 * Kf table built from those params), release-fences, then bumps g_lqg_gen;
 * the daemon acquire-fences on gen and reads BOTH params and table from
 * the SAME slot, so a new param set can never be paired with an old table
 * (hot LQG->LQG reconfigure safety).  (gen & 1) = live slot.
 */

struct lt_kf_row_s   { float kf0; float kf1; };
struct lt_kf_table_s
{
  float v_lo;                       /* captured speed bounds for interp   */
  float v_hi;
  struct lt_kf_row_s row[LQG_KF_PTS];
};
struct lt_lqg_slot_s
{
  struct lt_lqg_params_s p;         /* immutable per-engage param snapshot */
  struct lt_kf_table_s   kf;        /* Kf table built from p (signed c)    */
};
static struct lt_lqg_slot_s g_lqg_slot[2];
static atomic_uint          g_lqg_gen;   /* (gen & 1) = live slot          */

/* Parallel observability snapshot, guarded by the EXISTING g_stats_lock so
 * lqgstat row reads stay consistent with the daemon writer.  Kept separate
 * from g_stats so the PID interval-aggregate fields are untouched.
 */

static struct
{
  uint64_t iter;
  int      intensity;
  float    ey_hat;
  float    eth_hat_deg;
  float    innov;
  float    K1;
  float    K2;
  int      turn_dps;
  int      sat_state;
  uint32_t lost_cnt;
  int      last_ioctl_rc;
  int      last_ioctl_errno;
} g_lqg_stats;

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

static float clamp_float(float v, float lo, float hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

/****************************************************************************
 * LQG: steady-state Kalman gain + table build (Issue #169 / P1a)
 ****************************************************************************/

/* Float port of lqr_sim.py kalman_steady_gain() for the 2-state model
 * with the look-ahead measurement err = C x, C = [-c, -c*L].  Iterates
 * the Riccati difference equation to convergence (cheap for 2 states).
 * `c` is the SIGNED slope (edge*c): Kf's sign tracks the edge so the
 * correct step (which also uses the signed c) updates in the right
 * direction for BOTH edges.  Returns 0 on success (converged + finite),
 * < 0 on non-convergence or non-finite result.
 */

static int lqg_kalman_steady_gain(float v, float dt, float c, float L,
                                  float q_pos, float q_head, float r_meas,
                                  float *kf0_out, float *kf1_out)
{
  /* Ad = [[1, v*dt], [0, 1]]; Cc = [-c, -c*L]; Q = diag(q_pos, q_head). */

  float a01 = v * dt;
  float cc0 = -c;
  float cc1 = -c * L;

  /* P starts at identity. */

  float p00 = 1.0f, p01 = 0.0f, p10 = 0.0f, p11 = 1.0f;
  float kf0 = 0.0f, kf1 = 0.0f;
  float prev0 = 0.0f, prev1 = 0.0f;
  bool  have_prev = false;
  bool  converged = false;
  int   i;

  for (i = 0; i < LQG_KF_ITERS; i++)
    {
      /* Pp = Ad P Ad' + Q.  Ad P = [[p00 + a01*p10, p01 + a01*p11],
       *                             [p10,           p11          ]].
       * (Ad P) Ad' adds a01 * (col-1) to col-0 of each row's transpose;
       * expand the 2x2 product explicitly.
       */

      float ap00 = p00 + a01 * p10;
      float ap01 = p01 + a01 * p11;
      float ap10 = p10;
      float ap11 = p11;

      float pp00 = ap00 + a01 * ap01 + q_pos;   /* (Ad P Ad')[0][0]   */
      float pp01 = ap01;                         /* (Ad P Ad')[0][1]   */
      float pp10 = ap10 + a01 * ap11;            /* (Ad P Ad')[1][0]   */
      float pp11 = ap11 + q_head;                /* (Ad P Ad')[1][1]   */

      /* PpCt = Pp * Cc' (2x1).  S = Cc Pp Cc' + R (scalar). */

      float ppct0 = pp00 * cc0 + pp01 * cc1;
      float ppct1 = pp10 * cc0 + pp11 * cc1;
      float s     = cc0 * ppct0 + cc1 * ppct1 + r_meas;

      kf0 = ppct0 / s;
      kf1 = ppct1 / s;

      /* P = (I - Kf C) Pp. */

      float imkc00 = 1.0f - kf0 * cc0;
      float imkc01 = -kf0 * cc1;
      float imkc10 = -kf1 * cc0;
      float imkc11 = 1.0f - kf1 * cc1;

      p00 = imkc00 * pp00 + imkc01 * pp10;
      p01 = imkc00 * pp01 + imkc01 * pp11;
      p10 = imkc10 * pp00 + imkc11 * pp10;
      p11 = imkc10 * pp01 + imkc11 * pp11;

      if (have_prev &&
          fabsf(kf0 - prev0) + fabsf(kf1 - prev1) < LQG_KF_EPS)
        {
          converged = true;
          break;
        }

      prev0     = kf0;
      prev1     = kf1;
      have_prev = true;
    }

  if (!converged || !isfinite(kf0) || !isfinite(kf1))
    {
      return -1;
    }

  *kf0_out = kf0;
  *kf1_out = kf1;
  return 0;
}

/* Build the whole Kf table from the param set.  Uses the SIGNED c
 * (edge*c) so the table carries the engage-time edge.  Returns 0 on
 * success, < 0 if ANY grid point fails to converge / is non-finite.
 */

static int lqg_build_kf_table(struct lt_kf_table_s *t,
                              const struct lt_lqg_params_s *p)
{
  t->v_lo = (float)LQG_V_LO;
  t->v_hi = (float)LQG_V_HI;

  float dt       = 1.0f / (float)p->hz;
  float c_signed = (float)p->edge * p->c;
  int   i;

  for (i = 0; i < LQG_KF_PTS; i++)
    {
      float v = t->v_lo +
                (t->v_hi - t->v_lo) * (float)i / (float)(LQG_KF_PTS - 1);
      float kf0;
      float kf1;
      if (lqg_kalman_steady_gain(v, dt, c_signed, p->L, p->kf_qpos,
                                 p->kf_qhead, p->kf_rmeas, &kf0, &kf1) < 0)
        {
          return -1;       /* propagate non-convergence -> reject engage */
        }

      t->row[i].kf0 = kf0;
      t->row[i].kf1 = kf1;
    }

  return 0;
}

/* Linear-interpolate (kf0, kf1) from the table at speed v.  v is clamped
 * into [v_lo, v_hi]. */

static void lqg_interp_kf(const struct lt_kf_table_s *t, float v,
                          float *kf0, float *kf1)
{
  float vc   = clamp_float(v, t->v_lo, t->v_hi);
  float frac = (t->v_hi > t->v_lo) ?
               (vc - t->v_lo) / (t->v_hi - t->v_lo) : 0.0f;
  float fidx = frac * (float)(LQG_KF_PTS - 1);
  int   i0   = (int)fidx;
  if (i0 < 0)               i0 = 0;
  if (i0 > LQG_KF_PTS - 2)   i0 = LQG_KF_PTS - 2;
  float blend = fidx - (float)i0;

  *kf0 = t->row[i0].kf0 +
         (t->row[i0 + 1].kf0 - t->row[i0].kf0) * blend;
  *kf1 = t->row[i0].kf1 +
         (t->row[i0 + 1].kf1 - t->row[i0].kf1) * blend;
}

/* Seed the compiled LQG defaults into a param set (do_start + do_lqg's
 * first-engage-after-start inherit base). */

static void lqg_set_defaults(struct lt_lqg_params_s *p)
{
  p->speed_mmps      = 0;
  p->target          = LQG_DEF_TARGET;
  p->hz              = DEFAULT_HZ;
  p->c               = LQG_DEF_C;
  p->L               = LQG_DEF_L;
  p->edge            = LQG_DEF_EDGE;
  p->q1r             = LQG_DEF_Q1R;
  p->q2              = LQG_DEF_Q2;
  p->kf_qpos         = LQG_DEF_KF_QPOS;
  p->kf_qhead        = LQG_DEF_KF_QHEAD;
  p->kf_rmeas        = LQG_DEF_KF_RMEAS;
  p->sat_band        = LQG_DEF_SAT_BAND;
  p->lost_ticks      = LQG_DEF_LOST_TICKS;
  p->seek_dps        = 0;       /* 0 -> resolved to base speed in do_lqg  */
  p->lost_timeout_ms = LQG_DEF_LOST_TMO_MS;
  p->black           = LQG_DEF_BLACK;
  p->white           = LQG_DEF_WHITE;
  p->band_margin     = LQG_DEF_BAND_MARGIN;
  p->lost_time_ms    = LQG_DEF_LOST_TIME_MS;
}

/****************************************************************************
 * Lap capture per-tick append (Issue #166)
 ****************************************************************************/

#ifdef CONFIG_APP_CAPTURE

/* Append one lap record for this tick.  Called once per loop iteration
 * from both the idle path and the engaged path with the values already
 * computed this tick; `turn_cmd_dps` is 0 on the idle path.  The
 * DRIVEBASE_GET_STATE ioctl is done OUTSIDE the lock (it can block
 * briefly) so the lock hold stays much shorter than a tick.  A
 * double-check around the unlocked ioctl prevents writing into a freed
 * or finalized buffer if a CLI cap stop/brake/export lands in the
 * window.
 */

static void cap_append_tick(int db_fd, int intensity, int target,
                            int turn_cmd_dps)
{
  /* Lock-free fast path: an idle (not-armed) capture adds zero lock
   * contention to the control loop.  The hint may momentarily lag the
   * true state, so the locked re-checks below are still the authority.
   */

  if (!g_cap_armed_hint)
    {
      return;
    }

  pthread_mutex_lock(&g_cap_lock);
  if (g_cap.state != LT_CAP_ARMED)
    {
      pthread_mutex_unlock(&g_cap_lock);
      return;
    }

  if (g_cap.count >= g_cap.capacity)
    {
      g_cap.state      = LT_CAP_DONE;
      g_cap.overflow   = 1;
      g_cap_armed_hint = false;
      pthread_mutex_unlock(&g_cap_lock);
      return;
    }
  pthread_mutex_unlock(&g_cap_lock);

  /* Read drivebase state outside the lock. */

  int32_t heading_mdeg  = 0;
  int     turn_rate_dps = 0;
  int     speed_mmps    = 0;
  int     get_state_err = 0;

  struct drivebase_state_s st;
  memset(&st, 0, sizeof(st));
  if (ioctl(db_fd, DRIVEBASE_GET_STATE, (unsigned long)&st) < 0)
    {
      get_state_err = -errno;
    }
  else
    {
      heading_mdeg  = st.angle_mdeg;
      turn_rate_dps = (int)st.turn_rate_dps;
      speed_mmps    = (int)st.drive_speed_mmps;
    }

  /* Re-acquire + re-check: a cap stop / brake / export could have moved
   * the FSM out of ARMED (or filled the buffer) during the unlocked
   * ioctl above.
   */

  pthread_mutex_lock(&g_cap_lock);
  if (g_cap.state != LT_CAP_ARMED || g_cap.count >= g_cap.capacity)
    {
      pthread_mutex_unlock(&g_cap_lock);
      return;
    }

  if (get_state_err != 0)
    {
      g_cap.last_error = get_state_err;
    }

  struct timespec ts_now;
  clock_gettime(CLOCK_BOOTTIME, &ts_now);
  uint64_t now_us = (uint64_t)ts_now.tv_sec * 1000000ull
                  + (uint64_t)ts_now.tv_nsec / 1000ull;

  struct capture_linetrace_lap_run_record_s rec;
  rec.ts_us        = (uint32_t)(now_us - g_cap.t0_us);
  rec.intensity    = (uint16_t)clamp_int(intensity, 0, 65535);
  rec.target       = (uint16_t)clamp_int(target, 0, 65535);
  rec.turn_cmd_dps = (int16_t)clamp_int(turn_cmd_dps, -32768, 32767);
  rec.heading_mdeg = heading_mdeg;
  rec.turn_rate_dps = (int16_t)clamp_int(turn_rate_dps, -32768, 32767);
  rec.speed_mmps   = (int16_t)clamp_int(speed_mmps, -32768, 32767);
  rec.edge         = CAP_EDGE_UNSET;

  memcpy(&g_cap.buf[(size_t)g_cap.count * CAP_RECORD_SIZE], &rec,
         sizeof(rec));
  g_cap.count++;
  pthread_mutex_unlock(&g_cap_lock);
}

#endif /* CONFIG_APP_CAPTURE */

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

  /* LQG daemon-private state (Issue #169 / P1a).  last_lqg_gen drives the
   * generation-coupled observer re-seed; init UINT32_MAX so the first LQG
   * tick always re-seeds.  sat_streak counts consecutive SATURATED ticks
   * toward the LOST transition.
   */

  uint32_t last_lqg_gen = UINT32_MAX;
  int      sat_streak   = 0;

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

#ifdef CONFIG_APP_CAPTURE
          /* Braking ends the lap: freeze an in-flight capture (keep the
           * partial trace, it is still useful to P0c) rather than
           * discarding it.
           */

          pthread_mutex_lock(&g_cap_lock);
          if (g_cap.state == LT_CAP_ARMED)
            {
              g_cap.state      = LT_CAP_DONE;
              g_cap_armed_hint = false;
            }
          pthread_mutex_unlock(&g_cap_lock);
#endif
          /* Honor the explicit BRAKE; clear ownership + edge tracker
           * so the next tick's idle-edge path does not overwrite
           * the BRAKE with COAST.
           */
          g_engaged           = false;
          was_engaged         = false;
          /* Force a clean LQG observer re-seed on the next LQG engage
           * (Issue #169 / P1a).  Daemon-local — no cross-thread flag.
           */
          last_lqg_gen        = UINT32_MAX;
          sat_streak          = 0;
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

#ifdef CONFIG_APP_CAPTURE
          cap_append_tick(db_fd, last_intensity, g_params.target, 0);
#endif
          goto sleep_to_next_tick;
        }

      was_engaged = true;

      if (g_controller == CTRL_LQG)
        {
          /* ===== LQG observer + state-feedback (Issue #169 / P1a) =====
           *
           * Acquire-load the generation ONCE per tick, then read the
           * matched config slot (params + Kf table) atomically.  The
           * daemon NEVER reads the CLI-side g_lqg; do_lqg publishes a
           * complete immutable slot selected by g_lqg_gen, so a new
           * param set can never pair with an old table.
           */

          uint32_t gen = atomic_load_explicit(&g_lqg_gen,
                                              memory_order_acquire);
          const struct lt_lqg_slot_s *s = &g_lqg_slot[gen & 1];

          /* Generation-coupled re-seed: fires on the exact first tick
           * that observes a new generation (cold engage OR hot
           * reconfigure OR a brake-forced UINT32_MAX), so it can never
           * be lost the way a shared reset flag could.
           */

          if (gen != last_lqg_gen)
            {
              g_lqg_ey        = 0.0f;
              g_lqg_eth       = 0.0f;
              g_lqg_prev_w    = 0.0f;
              g_lqg_sat_state = SAT_NORMAL;
              g_lqg_lost_ms_acc = 0;
              g_lqg_oob_ms_acc  = 0;
              g_lqg_last_side = +1;
              sat_streak      = 0;
              last_lqg_gen    = gen;
            }

          int   v      = s->p.speed_mmps;     /* fixed for P1a (P5=dyn) */
          float dt     = 1.0f / (float)s->p.hz;
          float c_sgn  = (float)s->p.edge * s->p.c;
          float Lk     = s->p.L;
          int   max_turn = (v > 0) ? v : 0;

          /* 1. Read sensor; keep last on a -1 (no fresh sample) read. */

          int s_in = color_read_latest(color_fd);
          if (s_in >= 0)
            {
              last_intensity = s_in;
            }

          /* The setpoint is the LIVE shared field g_params.target (so
           * `linetrace target N` retunes mid-run without a republish),
           * matching the PID path.  err_meas = target - intensity =
           * -c*(ey + L*sin eth) within the linear band.
           */

          float err_meas = (last_intensity >= 0) ?
                           (float)(g_params.target - last_intensity) : 0.0f;

          /* 2. Snapshot Kf for v from the published slot table. */

          float kf0;
          float kf1;
          lqg_interp_kf(&s->kf, (float)v, &kf0, &kf1);

          /* 3. Predict (ZOH on the previously-applied omega). */

          float ey_p  = g_lqg_ey + (float)v * sinf(g_lqg_eth) * dt;
          float eth_p = g_lqg_eth + g_lqg_prev_w * dt;

          /* Saturation detection BEFORE the correct: never run the Kalman
           * correct on a saturated/clipped measurement (Issue #169
           * HW-verify fix; plan §4).
           *
           *  - railed: the intensity has left the LINEAR band of the
           *    sensor.  At the operating target the look-ahead intensity
           *    saturates well before the hard 0/1024 rails (a near-white
           *    975 is NOT a valid measurement and corrects garbage), so we
           *    key off the band-margin window [black+margin, white-margin]
           *    rather than SAT_RAIL_LO/HI alone.  This is the change that
           *    stops the HW divergence: outside this window the model
           *    err=-c*(ey+L sin eth) no longer holds, the innovation lies,
           *    and a single bad tick would inject a multi-mm ey jump.
           *  - out_of_band: predicted estimate ey_p outside the band.
           *    Used for the NORMAL->SATURATED transition (we trust the
           *    observer while it tracks).
           *  - meas_in_band: the MEASUREMENT-implied position (-err/c)
           *    inside the band AND not railed.  Used for SAT/LOST recovery,
           *    because the seek can bring the real sensor back into range
           *    while a drifted frozen estimate ey_p still reads
           *    out-of-band: gating recovery on ey_p would ignore a real
           *    return to the line and auto-stop.
           */

          bool railed = (last_intensity <= s->p.black + s->p.band_margin ||
                         last_intensity >= s->p.white - s->p.band_margin);
          float meas_pos = (!railed) ? (-err_meas / c_sgn) : 0.0f;
          bool out_of_band  = (fabsf(ey_p) > s->p.sat_band);
          bool meas_in_band = (!railed && fabsf(meas_pos) <= s->p.sat_band);

          /* Innovation clamp magnitude: the most a single legitimate
           * band-boundary correction may produce (|c_sgn| * sat_band).
           * Clamping innovation guarantees one bad tick can never inject
           * more than ~sat_band of ey even right at the band boundary
           * (Issue #169 HW-verify fix; plan §4).
           */

          float innov_clamp = fabsf(c_sgn) * s->p.sat_band;

          /* 4. State estimate per the saturation/LOST state machine.
           * `reseeded` marks a band re-entry tick: the estimate is
           * re-seeded from the measurement and the (stale-predict-based)
           * Kalman correct is SKIPPED for this tick — the next NORMAL
           * tick corrects from the fresh seed (plan §4).
           */

          float ey  = ey_p;
          float eth = eth_p;
          float innov = 0.0f;
          int   turn_dps = 0;
          bool  reseeded = false;

          if (g_lqg_sat_state == SAT_LOST)
            {
              /* LOST: stop trusting the observer; seek toward the line.
               * With dot ey = v*sin eth and omega=-K1*ey, a positive ey
               * needs a NEGATIVE turn to reduce it, so the corrective
               * seek is -last_side * seek_dps.
               */

              if (meas_in_band)
                {
                  /* Measurement back in band: re-seed from the
                   * measurement; resume NORMAL feedback on this tick from
                   * the seed, correct on the next NORMAL tick.
                   */

                  g_lqg_sat_state   = SAT_NORMAL;
                  sat_streak        = 0;
                  g_lqg_lost_ms_acc = 0;
                  g_lqg_oob_ms_acc  = 0;
                  ey  = clamp_float(meas_pos,
                                    -s->p.sat_band, s->p.sat_band);
                  eth = 0.0f;
                  g_lqg_ey  = ey;
                  g_lqg_eth = eth;
                  g_lqg_last_side = (ey >= 0.0f) ? +1 : -1;
                  reseeded = true;
                }
              else
                {
                  turn_dps = clamp_int(-g_lqg_last_side * s->p.seek_dps,
                                       -max_turn, max_turn);
                  if (max_turn == 0)
                    {
                      turn_dps = 0;
                    }

                  g_lqg_lost_ms_acc += 1000 / s->p.hz;

                  if (g_lqg_lost_ms_acc >= s->p.lost_timeout_ms)
                    {
                      /* Auto-stop: brake + disengage (daemon stays
                       * alive).  Reset observer + SM and force a clean
                       * re-seed on the next LQG engage.
                       */

                      g_lqg_lost_cnt++;
                      drivebase_send_forever(db_fd, 0, 0);
                      drivebase_send_stop(db_fd,
                                          DRIVEBASE_ON_COMPLETION_BRAKE);
                      g_engaged   = false;
                      was_engaged = false;
                      g_lqg_ey        = 0.0f;
                      g_lqg_eth       = 0.0f;
                      g_lqg_prev_w    = 0.0f;
                      g_lqg_sat_state = SAT_NORMAL;
                      sat_streak      = 0;
                      g_lqg_lost_ms_acc = 0;
                      g_lqg_oob_ms_acc  = 0;
                      last_lqg_gen    = UINT32_MAX;

                      pthread_mutex_lock(&g_stats_lock);
                      g_lqg_stats.iter++;
                      g_lqg_stats.intensity   = last_intensity;
                      g_lqg_stats.ey_hat      = 0.0f;
                      g_lqg_stats.eth_hat_deg = 0.0f;
                      g_lqg_stats.innov       = 0.0f;
                      g_lqg_stats.turn_dps    = 0;
                      g_lqg_stats.sat_state   = SAT_LOST;
                      g_lqg_stats.lost_cnt    = g_lqg_lost_cnt;
                      pthread_mutex_unlock(&g_stats_lock);

#ifdef CONFIG_APP_CAPTURE
                      /* LOST auto-stop is still one processed loop tick;
                       * append it (braked, so turn=0) before bailing so
                       * the capture stays one-record-per-tick like the
                       * idle / PID / normal-LQG paths (Issue #166 + #169).
                       */

                      cap_append_tick(db_fd, last_intensity,
                                      g_params.target, 0);
#endif

                      goto sleep_to_next_tick;
                    }
                }
            }

          if (g_lqg_sat_state != SAT_LOST && !reseeded)
            {
              bool saturated = (railed || out_of_band);

              if (!saturated)
                {
                  /* NORMAL correct with a CLAMPED scalar innovation.  The
                   * clamp bounds the single-tick ey jump even right at the
                   * band boundary, so a momentary bad measurement can never
                   * inject a multi-mm ey step (Issue #169 HW-verify fix).
                   */

                  float yhat = -c_sgn * ey_p - c_sgn * Lk * eth_p;
                  innov = err_meas - yhat;
                  innov = clamp_float(innov, -innov_clamp, innov_clamp);
                  ey  = ey_p + kf0 * innov;
                  eth = eth_p + kf1 * innov;
                  g_lqg_ey  = ey;
                  g_lqg_eth = eth;
                  g_lqg_last_side = (ey >= 0.0f) ? +1 : -1;
                  g_lqg_sat_state   = SAT_NORMAL;
                  sat_streak        = 0;
                  g_lqg_oob_ms_acc  = 0;
                  g_lqg_lost_ms_acc = 0;
                }
              else if (meas_in_band)
                {
                  /* Saturated estimate/measurement, but the spot is
                   * genuinely back inside the linear band: re-seed from the
                   * measurement, return to NORMAL, skip the stale correct
                   * (the next NORMAL tick corrects from the fresh seed).
                   */

                  g_lqg_sat_state   = SAT_NORMAL;
                  sat_streak        = 0;
                  g_lqg_oob_ms_acc  = 0;
                  g_lqg_lost_ms_acc = 0;
                  ey  = clamp_float(meas_pos,
                                    -s->p.sat_band, s->p.sat_band);
                  eth = eth_p;
                  g_lqg_ey  = ey;
                  g_lqg_eth = eth;
                  g_lqg_last_side = (ey >= 0.0f) ? +1 : -1;
                  reseeded = true;
                }
              else
                {
                  /* Saturated and out of band: FREEZE (predict only, skip
                   * the correct).  Accumulate saturated TIME — once it
                   * exceeds lost_time_ms, declare LOST.  Time-based (not a
                   * consecutive sat_streak) so a SAT<->NORMAL flap on a
                   * near-rail measurement can no longer keep resetting the
                   * counter and defeating LOST, which is exactly what kept
                   * LOST from ever firing on HW (Issue #169 HW-verify fix).
                   */

                  g_lqg_sat_state = SAT_SATURATED;
                  ey  = ey_p;
                  eth = eth_p;
                  g_lqg_ey  = ey;
                  g_lqg_eth = eth;
                  sat_streak++;
                  g_lqg_oob_ms_acc += 1000 / s->p.hz;
                  if (g_lqg_oob_ms_acc >= s->p.lost_time_ms)
                    {
                      g_lqg_sat_state   = SAT_LOST;
                      g_lqg_lost_ms_acc = 0;
                    }
                }
            }

          /* 5/6. State feedback + clamp (skipped when LOST emitted a
           * seek turn this tick).  Edge-agnostic: ey/eth are physical
           * signed states, edge sign lives only in the measurement row.
           */

          float K1 = sqrtf(s->p.q1r);
          float K2 = sqrtf(2.0f * (float)v * K1 + s->p.q2);

          if (g_lqg_sat_state != SAT_LOST)
            {
              float omega  = -K1 * ey - K2 * eth;          /* rad/s     */
              float turn_f = omega * (180.0f / PI_F);       /* deg/s     */
              turn_dps = clamp_int((int)lrintf(turn_f),
                                   -max_turn, max_turn);
            }

          /* Feed back the CLAMPED command (observer predict anti-windup). */

          g_lqg_prev_w = (float)turn_dps * (PI_F / 180.0f);

          /* 7. Drive — identical output path to the PID arm. */

          int rc = drivebase_send_forever(db_fd, v, turn_dps);

          /* 8. Publish observability. */

          pthread_mutex_lock(&g_stats_lock);
          g_lqg_stats.iter++;
          g_lqg_stats.intensity        = last_intensity;
          g_lqg_stats.ey_hat           = g_lqg_ey;
          g_lqg_stats.eth_hat_deg      = g_lqg_eth * (180.0f / PI_F);
          g_lqg_stats.innov            = innov;
          g_lqg_stats.K1               = K1;
          g_lqg_stats.K2               = K2;
          g_lqg_stats.turn_dps         = turn_dps;
          g_lqg_stats.sat_state        = g_lqg_sat_state;
          g_lqg_stats.lost_cnt         = g_lqg_lost_cnt;
          g_lqg_stats.last_ioctl_rc    = rc;
          g_lqg_stats.last_ioctl_errno = (rc < 0) ? errno : 0;
          pthread_mutex_unlock(&g_stats_lock);

#ifdef CONFIG_APP_CAPTURE
          /* Lap capture parity (Issue #166 + #169): the LQG arm has its
           * own pre-sleep exit, so it must append this engaged tick too —
           * the PID engaged path appends at its own goto below, the idle
           * path appends before its goto above.  Use the LQG-computed
           * turn_dps so the trace reflects the command actually applied.
           */

          cap_append_tick(db_fd, last_intensity, g_params.target, turn_dps);
#endif

          goto sleep_to_next_tick;
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

#ifdef CONFIG_APP_CAPTURE
      cap_append_tick(db_fd, last_intensity, g_params.target, turn_dps);
#endif

    sleep_to_next_tick:
      {
        /* Block-scoped: a label may not directly precede a
         * declaration in C, so wrap period_ns in a compound stmt.
         * Idle ticks reach here via `goto` to keep the
         * clock_nanosleep absolute-deadline ladder intact.
         *
         * Divisor is g_loop_hz (Issue #169 / P1a), not g_params.hz: it
         * carries the active controller's rate.  do_run sets g_loop_hz =
         * g_params.hz, so the PID-active period is numerically identical
         * to pre-P1a; do_lqg sets it from the published LQG slot hz.
         */

        long period_ns = 1000000000L / g_loop_hz;
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

#ifdef CONFIG_APP_CAPTURE
  /* Daemon exit: reap a hard-killed exporter first, then free a never-
   * exported capture (ARMED/DONE).  A live EXPORTING capture belongs to
   * the CLI export task — leave it; that task frees on its own cleanup.
   */

  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();
  if (g_cap.state == LT_CAP_ARMED || g_cap.state == LT_CAP_DONE)
    {
      cap_reset_idle_locked(0);
    }
  pthread_mutex_unlock(&g_cap_lock);
#endif

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

#ifdef CONFIG_APP_CAPTURE
  /* A start is only meaningful when no daemon runs (checked above).  The
   * one carry-over from a prior daemon life is the capture FSM: an
   * export task can outlive `linetrace stop` (see §3.6).  Reap a hard-
   * killed exporter, refuse the start if a LIVE export is still in
   * flight (must NOT erase its buffer ownership), otherwise free any
   * stale non-EXPORTING buffer and reset to IDLE.
   */

  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();
  if (g_cap.state == LT_CAP_EXPORTING)
    {
      pid_t pid = g_cap.export_pid;
      pthread_mutex_unlock(&g_cap_lock);
      fprintf(stderr,
              "linetrace: cap export in flight (pid=%d) — wait or "
              "`kill %d` before start\n",
              (int)pid, (int)pid);
      return 1;
    }

  cap_reset_idle_locked(0);
  pthread_mutex_unlock(&g_cap_lock);
#endif

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

  /* LQG controller defaults (Issue #169 / P1a).  Compiled, sim-locked.
   * Seed both the CLI-side working copy g_lqg and slot 0 so even a stray
   * CTRL_LQG read before the first `lqg` engage is well-formed.  Default
   * controller stays PID; seek_dps default = base speed (== 0 here) is
   * resolved per-engage in do_lqg.
   */

  lqg_set_defaults(&g_lqg);
  g_lqg_slot[0].p = g_lqg;
  (void)lqg_build_kf_table(&g_lqg_slot[0].kf, &g_lqg_slot[0].p);
  atomic_store_explicit(&g_lqg_gen, 0, memory_order_release);
  g_controller        = CTRL_PID;
  g_loop_hz           = DEFAULT_HZ;
  g_lqg_sat_state     = SAT_NORMAL;
  g_lqg_lost_cnt      = 0;
  g_lqg_lost_ms_acc   = 0;
  g_lqg_oob_ms_acc    = 0;
  g_lqg_last_side     = +1;
  memset(&g_lqg_stats, 0, sizeof(g_lqg_stats));

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

  /* Require the ACTIVE controller's speed to be zero (Issue #169 / P1a).
   * LQG stores its base speed in g_lqg.speed_mmps (not g_params), so a
   * `start; lqg 150; cal` would otherwise pass the g_params.speed_mmps==0
   * check and let the calibration sweep run while LQG is still driving.
   */

  int active_speed = (g_controller == CTRL_LQG) ?
                     g_lqg.speed_mmps : g_params.speed_mmps;
  if (active_speed != 0)
    {
      fprintf(stderr,
              "linetrace: cal requires speed=0 — 'linetrace %s 0%s' first\n",
              (g_controller == CTRL_LQG) ? "lqg" : "run",
              (g_controller == CTRL_LQG) ? "" : " 0");
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
  /* Active loop rate + controller flip (Issue #169 / P1a).  g_loop_hz =
   * g_params.hz keeps the PID-active period numerically identical to
   * pre-P1a.  The store barrier orders these before the CTRL_PID flip so
   * the daemon never reads CTRL_PID against a stale g_loop_hz.
   */

  g_loop_hz             = g_params.hz;
  __sync_synchronize();
  g_controller          = CTRL_PID;
  g_engaged             = true;

  printf("linetrace: speed=%d mm/s kp=%.2f ki=%.2f kd=%.2f "
         "target=%d hz=%d v_min=%d v_alpha=%.2f v_beta=%.2f\n",
         speed_mmps, kp_x100 / 100.0, ki_x100 / 100.0, kd_x100 / 100.0,
         target, hz, v_min_mmps,
         v_alpha_x100 / 100.0, v_beta_x100 / 100.0);
  return 0;
}

/****************************************************************************
 * Subcommand: lqg (Issue #169 / P1a)
 *
 * Engage the observer-based LQG line follower.  Mirrors do_run: validate
 * the working params, fill the INACTIVE double-buffered slot (params + Kf
 * table built from those params with the SIGNED c), then publish via a
 * release-store of the generation counter.  PID->LQG cold and LQG->LQG
 * hot reconfigure both go through here; the daemon's generation-coupled
 * re-seed handles the observer reset.  If the Kf build fails (Riccati
 * non-convergence / non-finite) the engage is rejected with NO gen bump
 * and NO state change.
 ****************************************************************************/

/* Parse a positive float flag value into *out (rejects non-finite). */

static int parse_float_arg(const char *s, float *out)
{
  char  *end;
  double v = strtod(s, &end);
  if (end == s || *end != '\0' || !isfinite(v))
    {
      return -1;
    }

  *out = (float)v;
  return 0;
}

static int do_lqg(int argc, char **argv)
{
  if (!g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: not running — 'linetrace start' first\n");
      return 1;
    }

  if (argc < 1)
    {
      fprintf(stderr,
              "usage: linetrace lqg <speed_mmps> [target] [--hz N] "
              "[--c C] [--L L] [--edge left|right]\n"
              "       [--q1r Q] [--q2 Q] [--kf-qpos Q] [--kf-qhead Q] "
              "[--kf-rmeas R]\n"
              "       [--sat-band MM] [--seek-dps D] [--lost-time MS] "
              "[--lost-timeout MS]\n"
              "       [--black N] [--white N] [--band-margin N] "
              "[--lost-ticks N]\n"
              "  (target defaults to the measured edge midpoint %d; the\n"
              "   observer needs the symmetric midpoint, not the PID 512)\n",
              LQG_DEF_TARGET);
      return 1;
    }

  /* Inherit unspecified params from the current working copy g_lqg (which
   * do_start seeded from compiled defaults).  speed/target are positional;
   * the rest are flags.
   */

  struct lt_lqg_params_s np = g_lqg;
  char *end;
  long  speed = strtol(argv[0], &end, 10);
  if (end == argv[0] || *end != '\0' || speed < 0 || speed > INT_MAX)
    {
      fprintf(stderr, "linetrace: speed must be a non-negative integer\n");
      return 1;
    }

  np.speed_mmps = (int)speed;

  /* Default the LQG target to the measured edge midpoint LQG_DEF_TARGET=634
   * (NOT the PID-shared 512, and NOT the inherited prior target): the
   * observer needs the symmetric midpoint every engage so the linear band
   * is not truncated on the black side (Issue #169 HW-verify fix).  An
   * explicit positional target still overrides; if you want a sticky custom
   * target, pass it explicitly each engage.
   */

  int   target  = LQG_DEF_TARGET;
  int   i       = 1;

  if (i < argc && argv[i][0] != '-')
    {
      target = atoi(argv[i]);
      i++;
    }

  /* Track whether the user explicitly set seek_dps; if not, resolve it to
   * the base speed (mirrors the PID max_turn ~= v law).
   */

  bool seek_set = false;

  while (i < argc)
    {
      if (strcmp(argv[i], "--hz") == 0 && i + 1 < argc)
        {
          np.hz = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--c") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.c) < 0)
            {
              fprintf(stderr, "linetrace: --c must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--L") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.L) < 0)
            {
              fprintf(stderr, "linetrace: --L must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--edge") == 0 && i + 1 < argc)
        {
          if (strcmp(argv[i + 1], "left") == 0)
            {
              np.edge = LQG_EDGE_LEFT;
            }
          else if (strcmp(argv[i + 1], "right") == 0)
            {
              np.edge = LQG_EDGE_RIGHT;
            }
          else
            {
              fprintf(stderr, "linetrace: --edge must be left|right\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--q1r") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.q1r) < 0)
            {
              fprintf(stderr, "linetrace: --q1r must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--q2") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.q2) < 0)
            {
              fprintf(stderr, "linetrace: --q2 must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--kf-qpos") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.kf_qpos) < 0)
            {
              fprintf(stderr,
                      "linetrace: --kf-qpos must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--kf-qhead") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.kf_qhead) < 0)
            {
              fprintf(stderr,
                      "linetrace: --kf-qhead must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--kf-rmeas") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.kf_rmeas) < 0)
            {
              fprintf(stderr,
                      "linetrace: --kf-rmeas must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--sat-band") == 0 && i + 1 < argc)
        {
          if (parse_float_arg(argv[i + 1], &np.sat_band) < 0)
            {
              fprintf(stderr,
                      "linetrace: --sat-band must be a finite number\n");
              return 1;
            }
          i += 2;
        }
      else if (strcmp(argv[i], "--lost-ticks") == 0 && i + 1 < argc)
        {
          np.lost_ticks = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--seek-dps") == 0 && i + 1 < argc)
        {
          np.seek_dps = atoi(argv[i + 1]);
          seek_set    = true;
          i += 2;
        }
      else if (strcmp(argv[i], "--lost-timeout") == 0 && i + 1 < argc)
        {
          np.lost_timeout_ms = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--black") == 0 && i + 1 < argc)
        {
          np.black = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--white") == 0 && i + 1 < argc)
        {
          np.white = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--band-margin") == 0 && i + 1 < argc)
        {
          np.band_margin = atoi(argv[i + 1]);
          i += 2;
        }
      else if (strcmp(argv[i], "--lost-time") == 0 && i + 1 < argc)
        {
          np.lost_time_ms = atoi(argv[i + 1]);
          i += 2;
        }
      else
        {
          fprintf(stderr, "linetrace: unknown option '%s'\n", argv[i]);
          return 1;
        }
    }

  if (target < 0 || target > 1024)
    {
      fprintf(stderr, "linetrace: target must be in [0, 1024]\n");
      return 1;
    }

  /* seek_dps default = base speed (mirrors PID max_turn ~= v). */

  if (!seek_set)
    {
      np.seek_dps = np.speed_mmps;
    }

  np.target = target;

  /* Validate (plan §3): physical sanity + finite checks. */

  if (!(isfinite(np.c) && np.c > 0.0f) ||
      !(isfinite(np.L) && np.L >= 0.0f) ||
      !(isfinite(np.q1r) && np.q1r > 0.0f) ||
      !(isfinite(np.q2) && np.q2 >= 0.0f) ||
      !(isfinite(np.kf_qpos) && np.kf_qpos > 0.0f) ||
      !(isfinite(np.kf_qhead) && np.kf_qhead >= 0.0f) ||
      !(isfinite(np.kf_rmeas) && np.kf_rmeas > 0.0f) ||
      !(isfinite(np.sat_band) && np.sat_band > 0.0f))
    {
      fprintf(stderr,
              "linetrace: lqg params must be finite "
              "(c>0 L>=0 q1r>0 q2>=0 kf_qpos>0 kf_qhead>=0 "
              "kf_rmeas>0 sat_band>0)\n");
      return 1;
    }

  if (np.hz < 10 || np.hz > 200)
    {
      fprintf(stderr, "linetrace: --hz must be in [10, 200]\n");
      return 1;
    }

  if (np.speed_mmps < 0)
    {
      fprintf(stderr, "linetrace: speed must be >= 0\n");
      return 1;
    }

  if (np.lost_ticks < 1)
    {
      fprintf(stderr, "linetrace: --lost-ticks must be >= 1\n");
      return 1;
    }

  if (np.lost_timeout_ms < 0)
    {
      fprintf(stderr, "linetrace: --lost-timeout must be >= 0\n");
      return 1;
    }

  if (np.seek_dps < 0)
    {
      fprintf(stderr, "linetrace: --seek-dps must be >= 0\n");
      return 1;
    }

  if (np.lost_time_ms < 0)
    {
      fprintf(stderr, "linetrace: --lost-time must be >= 0\n");
      return 1;
    }

  /* Sensor band sanity: black<white and the [black+margin, white-margin]
   * window must be non-empty, else every measurement reads "saturated"
   * and the observer can never run a correct (Issue #169 HW-verify fix).
   */

  if (np.black < 0 || np.white > 1024 || np.band_margin < 0 ||
      np.black + np.band_margin >= np.white - np.band_margin)
    {
      fprintf(stderr,
              "linetrace: need 0<=black, white<=1024, band_margin>=0, and "
              "black+margin < white-margin (non-empty linear band)\n");
      return 1;
    }

  /* Matched-slot publish (plan §3 / §3.1).  Fill the INACTIVE slot, build
   * its Kf table; on build failure reject with NO gen bump / NO state
   * change.  The release-store orders all slot writes before the new
   * generation is visible; the daemon's acquire-load pairs with it and the
   * generation-coupled re-seed fires on the first tick that sees the bump.
   */

  uint32_t cur  = atomic_load_explicit(&g_lqg_gen, memory_order_relaxed);
  uint32_t next = (cur + 1) & 1;

  g_lqg_slot[next].p = np;
  if (lqg_build_kf_table(&g_lqg_slot[next].kf, &g_lqg_slot[next].p) < 0)
    {
      fprintf(stderr,
              "linetrace: lqg Kf table build failed "
              "(Riccati non-convergence) — not engaged\n");
      return 1;
    }

  /* Commit the working copy only after a successful build so a rejected
   * engage leaves g_lqg unchanged for the next inherit.
   */

  g_lqg     = np;
  g_loop_hz = np.hz;             /* active loop rate */
  g_params.target = target;      /* seed the live shared setpoint */

  atomic_store_explicit(&g_lqg_gen, cur + 1, memory_order_release);
  g_controller = CTRL_LQG;       /* flip controller (no-op if already LQG) */
  g_engaged    = true;

  printf("linetrace: lqg speed=%d mm/s target=%d hz=%d c=%.2f L=%.2f "
         "edge=%s\n"
         "           q1r=%.4f q2=%.4f kf_qpos=%.4g kf_qhead=%.4g "
         "kf_rmeas=%.4g\n"
         "           sat_band=%.2f seek_dps=%d lost_time=%d ms "
         "lost_timeout=%d ms\n"
         "           black=%d white=%d band_margin=%d (lost_ticks=%d "
         "unused)\n",
         np.speed_mmps, target, np.hz, np.c, np.L,
         (np.edge == LQG_EDGE_LEFT) ? "left" : "right",
         np.q1r, np.q2, np.kf_qpos, np.kf_qhead, np.kf_rmeas,
         np.sat_band, np.seek_dps, np.lost_time_ms, np.lost_timeout_ms,
         np.black, np.white, np.band_margin, np.lost_ticks);
  return 0;
}

/****************************************************************************
 * Subcommand: lqgstat (Issue #169 / P1a)
 *
 * Stream the LQG observer internals for tuning.  Mirrors pidstat's
 * absolute-deadline ladder but simpler (snapshot fields only — no
 * interval aggregation).  Reads ONLY g_lqg_stats; never touches the PID
 * interval-aggregate fields in g_stats.  NOTE: iter keeps advancing and
 * rows keep emitting even at speed=0 / turn=0 (idle-zero), so a
 * zero-motion stream is not a hang.
 ****************************************************************************/

static int do_lqgstat(int argc, char **argv)
{
  long duration_ms = 0;
  long interval_ms = 1000;

  if (argc >= 1)
    {
      char *end;
      long v = strtol(argv[0], &end, 10);
      if (*end != '\0' || v < 0)
        {
          fprintf(stderr,
                  "linetrace: lqgstat duration_ms must be >= 0\n");
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
                  "linetrace: lqgstat interval_ms must be > 0\n");
          return 1;
        }
      interval_ms = v;
    }
  if (argc > 2)
    {
      fprintf(stderr,
              "usage: linetrace lqgstat [duration_ms [interval_ms]]\n");
      return 1;
    }

  if (!g_daemon_running)
    {
      fprintf(stderr, "linetrace: daemon not running\n");
      return 1;
    }

  /* Reject sub-period intervals (memory feedback_observability_jitter).
   * Guard against the ACTIVE loop rate g_loop_hz, not g_params.hz.
   */

  int min_interval_ms = (g_loop_hz > 0) ? (1000 / g_loop_hz) : 10;
  if (min_interval_ms < 1)
    {
      min_interval_ms = 1;
    }
  if (duration_ms > 0 && interval_ms < min_interval_ms)
    {
      fprintf(stderr,
              "linetrace: interval_ms %ld too small "
              "(min %d ms for hz=%d)\n",
              interval_ms, min_interval_ms, g_loop_hz);
      return 1;
    }

  printf("%8s %9s %6s %9s %12s %12s %9s %9s %7s %4s %5s\n",
         "time_ms", "iter", "intens", "ey_hat", "eth_hat_deg",
         "innov", "K1", "K2", "turn", "sat", "lost");

  static const char *sat_name[] = { "NORM", "SAT", "LOST" };

  struct timespec t0;
  struct timespec next;
  clock_gettime(CLOCK_MONOTONIC, &t0);
  next = t0;

  long elapsed_ms = 0;

  for (;;)
    {
      pthread_mutex_lock(&g_stats_lock);
      bool     running   = g_daemon_running;
      uint64_t iter64    = g_lqg_stats.iter;
      int      intensity = g_lqg_stats.intensity;
      float    ey_hat    = g_lqg_stats.ey_hat;
      float    eth_deg   = g_lqg_stats.eth_hat_deg;
      float    innov     = g_lqg_stats.innov;
      float    K1        = g_lqg_stats.K1;
      float    K2        = g_lqg_stats.K2;
      int      turn      = g_lqg_stats.turn_dps;
      int      sat       = g_lqg_stats.sat_state;
      uint32_t lost      = g_lqg_stats.lost_cnt;
      pthread_mutex_unlock(&g_stats_lock);

      struct timespec tn;
      clock_gettime(CLOCK_MONOTONIC, &tn);
      elapsed_ms = (tn.tv_sec - t0.tv_sec) * 1000L +
                   (tn.tv_nsec - t0.tv_nsec) / 1000000L;

      const char *sname = (sat >= 0 && sat <= 2) ? sat_name[sat] : "?";

      printf("%8ld %9lu %6d %9.3f %12.3f %12.3f %9.3f %9.3f %7d %4s "
             "%5lu\n",
             elapsed_ms,
             (unsigned long)(iter64 & 0xFFFFFFFFu),
             intensity, ey_hat, eth_deg, innov, K1, K2, turn, sname,
             (unsigned long)lost);

      if (!running)
        {
          printf("# lqgstat: daemon stopped\n");
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
 *
 * Issue #169 / P1a: target is the one LQG param that stays live-mutable
 * without a slot republish.  The daemon's LQG arm reads g_params.target
 * (this shared field) for err_meas, so `linetrace target N` retunes the
 * setpoint for both controllers identically.
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
 * Subcommand: cap (Issue #166)
 *
 * `linetrace cap arm <n>` / `stop` / `export` / `abort` / `status`.
 * Records a per-tick lap trace into a heap buffer and exports it through
 * /dev/btcap.  All verbs take g_cap_lock and reap a dead exporter before
 * inspecting state, so a SIGKILL'd export is reclaimed on the next verb.
 ****************************************************************************/

#ifdef CONFIG_APP_CAPTURE

static int do_cap_arm(int argc, char **argv)
{
  if (!g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: not running — 'linetrace start' first\n");
      return 1;
    }

  if (argc != 1)
    {
      fprintf(stderr, "usage: linetrace cap arm <n_records>\n");
      return 1;
    }

  char *end = NULL;
  long  n   = strtol(argv[0], &end, 10);
  if (end == argv[0] || *end != '\0')
    {
      fprintf(stderr, "linetrace: cap arm <n_records> must be an integer\n");
      return 1;
    }

  if (n < 1 || n > (long)CAP_MAX_RECORDS)
    {
      fprintf(stderr,
              "linetrace: cap arm n_records must be in [1, %lu]\n",
              (unsigned long)CAP_MAX_RECORDS);
      return 1;
    }

  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();

  if (g_cap.state != LT_CAP_IDLE)
    {
      enum linetrace_cap_state_e st = g_cap.state;
      pthread_mutex_unlock(&g_cap_lock);
      fprintf(stderr,
              "linetrace: cap already %s — `cap stop`/`cap abort` first\n",
              cap_state_str(st));
      return 1;
    }

  uint8_t *buf = (uint8_t *)malloc((size_t)n * CAP_RECORD_SIZE);
  if (buf == NULL)
    {
      g_cap.last_error = -ENOMEM;
      pthread_mutex_unlock(&g_cap_lock);
      fprintf(stderr, "linetrace: cap arm malloc(%lu) failed\n",
              (unsigned long)((size_t)n * CAP_RECORD_SIZE));
      return 1;
    }

  struct timespec ts_now;
  clock_gettime(CLOCK_BOOTTIME, &ts_now);

  g_cap.buf        = buf;
  g_cap.capacity   = (uint32_t)n;
  g_cap.count      = 0;
  g_cap.t0_us      = (uint64_t)ts_now.tv_sec * 1000000ull
                   + (uint64_t)ts_now.tv_nsec / 1000ull;
  g_cap.last_error = 0;
  g_cap.overflow   = 0;
  g_cap.export_pid = -1;
  g_cap.state      = LT_CAP_ARMED;

  /* Publish the lock-free hint LAST, after every g_cap field the append
   * path reads is committed, so the daemon never sees armed=true with a
   * half-initialized buffer.
   */

  g_cap_armed_hint = true;

  /* The append cadence follows the ACTIVE control loop (g_loop_hz), not
   * g_params.hz: under LQG the loop runs at the published slot hz, so use
   * g_loop_hz here so the printed duration estimate matches the real fill
   * rate (Issue #169 / P1a integration).  Display-only.
   */

  int hz = g_loop_hz;
  pthread_mutex_unlock(&g_cap_lock);

  printf("linetrace: cap armed, capacity=%ld records (~%ld/%d s)\n",
         n, n, hz);
  return 0;
}

static int do_cap_stop(void)
{
  if (!g_daemon_running)
    {
      fprintf(stderr,
              "linetrace: not running — 'linetrace start' first\n");
      return 1;
    }

  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();

  if (g_cap.state == LT_CAP_ARMED)
    {
      g_cap.state      = LT_CAP_DONE;
      g_cap_armed_hint = false;
    }

  if (g_cap.state == LT_CAP_DONE)
    {
      uint32_t count = g_cap.count;
      pthread_mutex_unlock(&g_cap_lock);
      printf("linetrace: cap done, %lu records captured\n",
             (unsigned long)count);
      return 0;
    }

  enum linetrace_cap_state_e st = g_cap.state;
  pthread_mutex_unlock(&g_cap_lock);
  fprintf(stderr, "linetrace: nothing to stop (cap %s)\n",
          cap_state_str(st));
  return 1;
}

static int do_cap_export(void)
{
  /* Transition DONE -> EXPORTING atomically; snapshot buf/count. */

  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();

  if (g_cap.state == LT_CAP_ARMED)
    {
      pthread_mutex_unlock(&g_cap_lock);
      fprintf(stderr,
              "linetrace: still armed — `cap stop` first\n");
      return 1;
    }

  if (g_cap.state != LT_CAP_DONE)
    {
      enum linetrace_cap_state_e st = g_cap.state;
      pthread_mutex_unlock(&g_cap_lock);
      fprintf(stderr, "linetrace: nothing to export (cap %s)\n",
              cap_state_str(st));
      return 1;
    }

  if (g_cap.count == 0)
    {
      pthread_mutex_unlock(&g_cap_lock);
      fprintf(stderr, "linetrace: nothing to export (0 records)\n");
      return 1;
    }

  uint8_t *buf   = g_cap.buf;
  uint32_t count = g_cap.count;
  g_cap.export_pid = getpid();
  g_cap.state      = LT_CAP_EXPORTING;
  pthread_mutex_unlock(&g_cap_lock);

  /* Install flag-only SIGINT/SIGTERM handlers so a handled `kill <pid>`
   * while blocked on the reader winds down cleanly (mirrors apps/sensor
   * do_capture).  SIGKILL skips this and is reclaimed lazily by
   * cap_reap_dead_exporter_locked on the next cap verb.
   */

  g_cap_export_aborting = 0;
  struct sigaction old_int_sa;
  struct sigaction old_term_sa;
  bool int_installed  = false;
  bool term_installed = false;
  struct sigaction new_sa = {0};
  new_sa.sa_handler = cap_export_sig_handler;
  sigemptyset(&new_sa.sa_mask);
  if (sigaction(SIGINT, &new_sa, &old_int_sa) == 0)
    {
      int_installed = true;
    }
  if (sigaction(SIGTERM, &new_sa, &old_term_sa) == 0)
    {
      term_installed = true;
    }

  printf("linetrace: cap export %lu records (schema=%s)\n",
         (unsigned long)count, g_capture_schema_linetrace_lap_run.name);
  printf("linetrace: waiting for `btsensor mode capture` (or BT MODE "
         "CAPTURE) to drain /dev/btcap...\n");

  capture_handle_t h;
  bool init_done = false;
  int  rc = 0;

  /* A signal can land between handler-install and the first blocking
   * capture call; check g_cap_export_aborting before each stage so the
   * export still winds down via capture_abort instead of pushing data.
   */

  if (g_cap_export_aborting)
    {
      rc = -ECANCELED;
      goto out;
    }

  rc = capture_init(&h, &g_capture_schema_linetrace_lap_run, count);
  if (rc < 0)
    {
      fprintf(stderr, "linetrace: capture_init: %d\n", rc);
      goto out;
    }

  init_done = true;

  if (g_cap_export_aborting)
    {
      rc = -ECANCELED;
      capture_abort(&h);
      init_done = false;
      goto out;
    }

  rc = capture_write(&h, buf, (size_t)count * CAP_RECORD_SIZE);
  if (rc < 0)
    {
      fprintf(stderr, "linetrace: capture_write: %d\n", rc);
      capture_abort(&h);
      init_done = false;
      goto out;
    }

  if (g_cap_export_aborting)
    {
      rc = -ECANCELED;
      capture_abort(&h);
      init_done = false;
      goto out;
    }

  rc = capture_deinit(&h);
  init_done = false;
  if (rc < 0)
    {
      fprintf(stderr, "linetrace: capture_deinit: %d\n", rc);
      goto out;
    }

  printf("linetrace: cap export done\n");

out:
  if (init_done)
    {
      capture_abort(&h);
    }

  if (g_cap_export_aborting)
    {
      fprintf(stderr, "linetrace: cap export aborted by signal\n");
    }

  if (int_installed)
    {
      sigaction(SIGINT, &old_int_sa, NULL);
    }

  if (term_installed)
    {
      sigaction(SIGTERM, &old_term_sa, NULL);
    }

  /* Release ownership: free the buffer and return the FSM to IDLE.  The
   * daemon never frees a live EXPORTING buffer, so this is the sole
   * owner on the handled path (a SIGKILL would skip this and the lazy
   * reaper covers it).  Guard on export_pid==getpid() so a do_start /
   * daemon-exit that ran concurrently (and could only have done so after
   * a reap) is never stomped.
   */

  pthread_mutex_lock(&g_cap_lock);
  if (g_cap.state == LT_CAP_EXPORTING && g_cap.export_pid == getpid())
    {
      cap_reset_idle_locked((rc < 0) ? rc : 0);
    }
  pthread_mutex_unlock(&g_cap_lock);

  return (rc < 0) ? 1 : 0;
}

static int do_cap_abort(void)
{
  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();

  if (g_cap.state == LT_CAP_ARMED || g_cap.state == LT_CAP_DONE)
    {
      cap_reset_idle_locked(0);
      pthread_mutex_unlock(&g_cap_lock);
      printf("linetrace: cap discarded\n");
      return 0;
    }

  if (g_cap.state == LT_CAP_EXPORTING)
    {
      pid_t pid = g_cap.export_pid;
      pthread_mutex_unlock(&g_cap_lock);
      fprintf(stderr,
              "linetrace: cap export in flight (pid=%d) — `kill %d` to "
              "cancel it\n", (int)pid, (int)pid);
      return 1;
    }

  pthread_mutex_unlock(&g_cap_lock);
  printf("linetrace: nothing to abort\n");
  return 0;
}

static int do_cap_status(void)
{
  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();
  enum linetrace_cap_state_e st = g_cap.state;
  uint32_t count    = g_cap.count;
  uint32_t capacity = g_cap.capacity;
  uint8_t  overflow = g_cap.overflow;
  int      last_err = g_cap.last_error;
  pid_t    pid      = g_cap.export_pid;
  pthread_mutex_unlock(&g_cap_lock);

  printf("cap_state:     %s\n", cap_state_str(st));
  printf("cap_count:     %lu\n", (unsigned long)count);
  printf("cap_capacity:  %lu\n", (unsigned long)capacity);
  printf("cap_overflow:  %u\n", (unsigned)overflow);
  printf("cap_last_error: %d\n", last_err);
  if (st == LT_CAP_EXPORTING)
    {
      printf("cap_export_pid: %d\n", (int)pid);
    }

  return 0;
}

static int do_cap(int argc, char **argv)
{
  if (argc < 1)
    {
      fprintf(stderr,
              "usage: linetrace cap arm <n> | stop | export | abort "
              "| status\n");
      return 1;
    }

  if (strcmp(argv[0], "arm") == 0)
    {
      return do_cap_arm(argc - 1, argv + 1);
    }

  if (strcmp(argv[0], "stop") == 0)
    {
      return do_cap_stop();
    }

  if (strcmp(argv[0], "export") == 0)
    {
      return do_cap_export();
    }

  if (strcmp(argv[0], "abort") == 0)
    {
      return do_cap_abort();
    }

  if (strcmp(argv[0], "status") == 0)
    {
      return do_cap_status();
    }

  fprintf(stderr, "linetrace: unknown cap subcommand '%s'\n", argv[0]);
  return 1;
}

#else /* !CONFIG_APP_CAPTURE */

static int do_cap(int argc, char **argv)
{
  (void)argc;
  (void)argv;
  fprintf(stderr, "linetrace: capture not built (CONFIG_APP_CAPTURE)\n");
  return 1;
}

#endif /* CONFIG_APP_CAPTURE */

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

#ifdef CONFIG_APP_CAPTURE
  pthread_mutex_lock(&g_cap_lock);
  cap_reap_dead_exporter_locked();
  enum linetrace_cap_state_e cap_st = g_cap.state;
  uint32_t cap_count    = g_cap.count;
  uint32_t cap_capacity = g_cap.capacity;
  pthread_mutex_unlock(&g_cap_lock);

  printf("cap_state:     %s\n", cap_state_str(cap_st));
  printf("cap_count:     %lu\n", (unsigned long)cap_count);
  printf("cap_capacity:  %lu\n", (unsigned long)cap_capacity);
#else
  printf("cap_state:     not_built\n");
#endif

  /* LQG controller status (Issue #169 / P1a).  The existing PID lines
   * above are unchanged (tests assert on them).  LQG params come from the
   * CLI-side working copy g_lqg; observer/SM fields from g_lqg_stats.
   */

  uint8_t controller = g_controller;
  printf("controller:    %s\n", (controller == CTRL_LQG) ? "lqg" : "pid");

  pthread_mutex_lock(&g_stats_lock);
  int      lqg_sat  = g_lqg_stats.sat_state;
  uint32_t lqg_lost = g_lqg_stats.lost_cnt;
  pthread_mutex_unlock(&g_stats_lock);

  static const char *sat_name[] = { "NORM", "SAT", "LOST" };
  printf("lqg_c:         %.2f\n", g_lqg.c);
  printf("lqg_L:         %.2f\n", g_lqg.L);
  printf("lqg_edge:      %s\n",
         (g_lqg.edge == LQG_EDGE_LEFT) ? "left" : "right");
  printf("lqg_q1r:       %.4f\n", g_lqg.q1r);
  printf("lqg_q2:        %.4f\n", g_lqg.q2);
  printf("lqg_kf_rmeas:  %.4g\n", g_lqg.kf_rmeas);
  printf("lqg_sat_band:  %.2f\n", g_lqg.sat_band);
  printf("lqg_lost_ticks: %d\n", g_lqg.lost_ticks);
  printf("lqg_seek_dps:  %d\n", g_lqg.seek_dps);
  printf("lqg_lost_timeout: %d\n", g_lqg.lost_timeout_ms);
  printf("lqg_sat_state: %s\n",
         (lqg_sat >= 0 && lqg_sat <= 2) ? sat_name[lqg_sat] : "?");
  printf("lqg_lost_cnt:  %lu\n", (unsigned long)lqg_lost);
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
    "       linetrace lqg <speed_mmps> [target] [--hz N] [--c C] [--L L]\n"
    "                     [--edge left|right] [--q1r Q] [--q2 Q]\n"
    "                     [--kf-qpos Q] [--kf-qhead Q] [--kf-rmeas R]\n"
    "                     [--sat-band MM] [--seek-dps D] [--lost-time MS]\n"
    "                     [--lost-timeout MS] [--black N] [--white N]\n"
    "                     [--band-margin N] [--lost-ticks N]\n"
    "       linetrace target <N>\n"
    "       linetrace brake\n"
    "       linetrace stop\n"
    "       linetrace pidstat [duration_ms [interval_ms]]\n"
    "       linetrace lqgstat [duration_ms [interval_ms]]\n"
    "       linetrace status\n"
    "       linetrace cap arm <n> | stop | export | abort | status\n"
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
    "  lqg:     engage the observer-based LQG follower (coexists w/ PID;\n"
    "           default stays PID).  2-state Kalman observer + closed-form\n"
    "           speed-scheduled state feedback.  c/L are sensor calibration\n"
    "           (counts/mm, look-ahead mm); --edge picks the steer polarity.\n"
    "           lqg 0          idle at zero (FOREVER(0,0), stays engaged)\n"
    "           lqg 150        follow at 150 mm/s with compiled defaults\n"
    "           lqg 150 512 --c 150 --L 52 --edge right\n"
    "           unspecified params inherit the prior lqg / compiled default;\n"
    "           seek-dps defaults to base speed.  `run` flips back to PID.\n"
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
    "  lqgstat: stream LQG observer internals (default 1000ms).  Columns:\n"
    "           time_ms iter intens ey_hat eth_hat_deg innov K1 K2 turn\n"
    "           sat(NORM|SAT|LOST) lost.  All snapshots.  iter keeps\n"
    "           advancing at speed=0 (idle-zero is not a hang).\n"
    "  status:  print daemon state and last err/intensity/i_acc snapshot\n"
    "  cap:     record a per-tick lap trace and export it as a .cap\n"
    "           arm <n>   malloc an n-record buffer; daemon appends each\n"
    "                     tick (idle or engaged) until full or `cap stop`\n"
    "           stop      freeze the capture (keep data), ready to export\n"
    "           export    drain the trace through /dev/btcap (blocks for\n"
    "                     `btsensor mode capture`; run it with `&`)\n"
    "           abort     discard an armed/done capture (free buffer)\n"
    "           status    print cap_state/count/capacity/overflow\n"
    "           brake freezes an in-flight capture; stop drops an\n"
    "           un-exported one — `cap export` before `linetrace stop`\n",
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

  if (strcmp(argv[1], "lqg") == 0)
    {
      return do_lqg(argc - 2, argv + 2);
    }

  if (strcmp(argv[1], "pidstat") == 0)
    {
      return do_pidstat(argc - 2, argv + 2);
    }

  if (strcmp(argv[1], "lqgstat") == 0)
    {
      return do_lqgstat(argc - 2, argv + 2);
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

  if (strcmp(argv[1], "cap") == 0)
    {
      return do_cap(argc - 2, argv + 2);
    }

  usage();
  return 1;
}
