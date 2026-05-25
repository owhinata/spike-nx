/****************************************************************************
 * apps/drivebase/drivebase_sysid.c
 *
 * System identification verbs (Issue #152 Phase 6 Step 6.4).  See
 * drivebase_sysid.h for the architecture + contract.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/power/battery_ioctl.h>
#include <arch/board/board_drivebase.h>

#include "drivebase_angle.h"
#include "drivebase_internal.h"
#include "drivebase_motor.h"
#include "drivebase_observer.h"
#include "drivebase_settings.h"
#include "drivebase_sysid.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SYSID_DEV_BAT           "/dev/bat0"
#define SYSID_DEV_DB            "/dev/drivebase"

/* Observer slope window — match the per-side servo default (Issue
 * #126) so the v_est reading SysId sees is what the closed loop sees.
 */

#define SYSID_OBS_WINDOW_MS     30

/* Threshold above which an observed |v| counts as "moving" (Plan D8).
 * 30 deg/s = 30000 mdeg/s, matching db_stall_settings::stall_speed.
 */

#define SYSID_V_MOVE_MDEGPS     30000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* CRITICAL: db_observer_s contains a 128-slot ring of 16-byte samples,
 * making sysid_obs_s ~4160 bytes — far above the CLI task's 4 KB
 * stack cap (see apps/drivebase/Makefile).  Allocating on the stack
 * caused an immediate HardFault + reset during Phase 6 Step 6.4 bench
 * before the first hold_duty completed.  Callers MUST allocate this
 * struct via calloc(); each verb pairs the calloc with a single free
 * on every exit path.  The `_drive` verb follows the same pattern
 * with db_drivebase_s (drivebase_main.c:1066).
 */

struct sysid_obs_s
{
  struct db_observer_s obs[DB_SIDE_NUM];
  bool                 seeded[DB_SIDE_NUM];
};

/****************************************************************************
 * SIGINT handling
 *
 * Plan D8 contract: every verb coasts on exit, including Ctrl-C
 * interruption.  The hold loops poll g_sysid_abort each tick and break
 * out so the normal sysid_close_motors() path still runs (which coasts
 * both wheels).  We don't call drivebase_motor_coast() from the signal
 * handler itself — the LUMP ioctl path is not async-signal-safe.
 ****************************************************************************/

static volatile sig_atomic_t g_sysid_abort = 0;
static struct sigaction      g_sysid_prev_sigint;
static bool                  g_sysid_sigint_saved = false;

static void sysid_sigint_handler(int sig)
{
  (void)sig;
  g_sysid_abort = 1;
}

static void sysid_install_sigint(void)
{
  g_sysid_abort = 0;
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = sysid_sigint_handler;
  if (sigaction(SIGINT, &sa, &g_sysid_prev_sigint) == 0)
    {
      g_sysid_sigint_saved = true;
    }
}

static void sysid_restore_sigint(void)
{
  /* Restore the caller's previous SIGINT disposition (often the NSH-
   * default Ctrl-C-terminates-task handler).  Falling back to SIG_DFL
   * is fine — the CLI task is about to exit anyway — but preserves
   * whatever the parent had set up.
   */
  if (g_sysid_sigint_saved)
    {
      sigaction(SIGINT, &g_sysid_prev_sigint, NULL);
      g_sysid_sigint_saved = false;
    }
  else
    {
      struct sigaction sa;
      memset(&sa, 0, sizeof(sa));
      sa.sa_handler = SIG_DFL;
      sigaction(SIGINT, &sa, NULL);
    }
}

/****************************************************************************
 * Time + battery helpers
 ****************************************************************************/

static uint64_t sysid_now_us(void)
{
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

static void sysid_sleep_us(uint32_t us)
{
  /* usleep accepts up to 1e6; chunk longer waits.  Caller passes ms-
   * scale numbers (hold windows, segment durations), so this never
   * matters in practice but is safer than a single large usleep.
   */
  while (us > 900000U)
    {
      usleep(900000U);
      us -= 900000U;
    }
  if (us > 0) usleep(us);
}

/* Independent BATIOC_VOLTAGE read — drivebase_battery.c's atomic is
 * daemon-only and unused while SysId is running (daemon must be off).
 * Returns mV on success, -1 on failure.
 */

static int32_t sysid_read_vbat_mv(void)
{
  int fd = open(SYSID_DEV_BAT, O_RDONLY);
  if (fd < 0) return -1;
  int val = 0;
  int rc = ioctl(fd, BATIOC_VOLTAGE, (unsigned long)&val);
  close(fd);
  if (rc < 0 || val <= 0) return -1;
  return (int32_t)val;
}

/* Plan D8: kS_nominal = kS_measured * vbat_mv / battery_nominal_mv.
 * (Low-vbat measurements overestimate gain — scale DOWN to reach
 * nominal.  Round to nearest to avoid systematic truncation bias.)
 */

static int32_t sysid_normalise_to_nominal(int32_t measured, int32_t vbat_mv,
                                          int32_t nominal_mv)
{
  if (nominal_mv <= 0 || vbat_mv <= 0) return measured;
  int64_t num = (int64_t)measured * vbat_mv;
  /* Symmetric rounding regardless of sign. */
  if (num >= 0) return (int32_t)((num + nominal_mv / 2) / nominal_mv);
  return (int32_t)((num - nominal_mv / 2) / nominal_mv);
}

/* Extract a trailing/standalone `pivot` keyword from a verb's argv.
 * Returns true and removes the token (shifting the remainder down +
 * decrementing *argc) so the positional parsing below is unchanged
 * whether or not pivot was given.  `pivot` is a distinct keyword that
 * never collides with the numeric positional args, so a full scan is
 * safe regardless of where the operator put it.
 *
 * Pivot mode drives L = -duty, R = +duty (a symmetric in-place
 * rotation) to identify the HEADING axis instead of the distance axis.
 * In a symmetric pivot the heading state velocity (sR - sL)/2 equals
 * the per-wheel speed magnitude, so the per-wheel kV/kA the fit
 * produces maps directly onto ff_head_kV/ff_head_kA with no
 * axle/wheel conversion (Phase 7 / Issue #158).
 */

static bool sysid_extract_pivot(int *argc, char *argv[])
{
  for (int i = 0; i < *argc; i++)
    {
      if (strcmp(argv[i], "pivot") == 0)
        {
          for (int j = i; j < *argc - 1; j++) argv[j] = argv[j + 1];
          (*argc)--;
          return true;
        }
    }
  return false;
}

/****************************************************************************
 * Daemon-attached gate
 ****************************************************************************/

/* Returns 0 if it is safe to drive motors directly, -EBUSY if the
 * daemon is attached (RT thread would race us), -ENOENT if the chardev
 * itself is missing (encoder-only build — no daemon possible).
 *
 * Fail-closed (Codex Round 1 BLOCKING): if the chardev exists but
 * GET_STATUS fails for any other reason, we cannot prove the daemon is
 * stopped, so refuse with -EBUSY rather than assume safe.  An open
 * failure other than ENOENT (EACCES, EBADF, EBUSY against the device
 * itself) also propagates so the caller knows why the gate refused.
 */

static int sysid_check_daemon_off(void)
{
  int fd = open(SYSID_DEV_DB, O_RDONLY);
  if (fd < 0)
    {
      int err = errno;
      return (err == ENOENT) ? -ENOENT : -err;
    }
  struct drivebase_status_s st;
  memset(&st, 0, sizeof(st));
  int rc = ioctl(fd, DRIVEBASE_GET_STATUS, (unsigned long)&st);
  close(fd);
  if (rc < 0)
    {
      /* Status unknown — assume daemon may be attached.  Caller's
       * message tells the operator to run `drivebase stop` so the
       * normal recovery path is the same as the attached case.
       */
      return -EBUSY;
    }
  return st.daemon_attached ? -EBUSY : 0;
}

/****************************************************************************
 * Motor + observer lifecycle (PID + aggregate bypass)
 ****************************************************************************/

static void sysid_obs_init(struct sysid_obs_s *o)
{
  const struct db_stall_settings_s *st = db_settings_stall();
  for (int i = 0; i < DB_SIDE_NUM; i++)
    {
      db_observer_init(&o->obs[i],
                       (uint32_t)st->stall_speed_mdegps,
                       (uint32_t)st->stall_duty_min,
                       st->stall_window_ms,
                       SYSID_OBS_WINDOW_MS);
      o->seeded[i] = false;
    }
}

/* Drain encoder samples for both sides, feed the observer.  `duty_abs`
 * is the magnitude of the duty currently applied (same for both sides
 * — SysId always commands symmetric drive).
 */

static void sysid_obs_drain(struct sysid_obs_s *o, uint32_t duty_abs)
{
  for (int i = 0; i < DB_SIDE_NUM; i++)
    {
      struct db_motor_sample_s sm;
      int dr = drivebase_motor_drain((enum db_side_e)i, &sm);
      if (dr == 0)
        {
          int64_t x = db_angle_deg_to_mdeg(sm.raw_value);
          if (!o->seeded[i])
            {
              db_observer_reset(&o->obs[i], x, sm.timestamp_us);
              o->seeded[i] = true;
            }
          else
            {
              db_observer_update_sample(&o->obs[i], x, sm.timestamp_us,
                                        duty_abs);
            }
        }
    }
}

static void sysid_hold_duty(struct sysid_obs_s *o,
                            int32_t duty, uint32_t ms, bool pivot)
{
  /* Abort already raised — return without commanding a new duty.
   * Without this guard the inner for-loop of ramp-kv (and any other
   * caller) would still walk through its remaining steps, sending a
   * fresh set_duty per call, between the abort signal and the next
   * outer-loop check.  (Codex Round 2 BLOCKING.)
   */
  if (g_sysid_abort) return;

  if (duty > 10000)  duty =  10000;
  if (duty < -10000) duty = -10000;

  uint32_t duty_abs = (uint32_t)(duty < 0 ? -duty : duty);
  uint32_t elapsed  = 0;
  /* Per-tick re-apply at 5 ms cadence (matches the daemon's RT tick
   * default — bench confirmed the production daemon drives motors
   * cleanly at that rate, and tightening to 2 ms did NOT change SysId
   * behaviour).  The earlier theory that one-shot set_duty fades from
   * a LUMP override loop was wrong — `_motor duty l 6000` keeps the
   * motor spinning for 5+ s with no re-apply, proving mode 2 is not
   * driving an override.  We still re-apply per tick because the
   * daemon does the same, and there is no downside.
   *
   * Ctrl-C breaks out so the caller's normal cleanup path coasts
   * both motors via sysid_close_motors().  We don't coast here — the
   * verb may want to print partial results before the cleanup runs.
   */
  /* Pivot drives the left wheel backward (-duty) so the chassis rotates
   * in place; straight drives both the same.  The caller's uL_buf must
   * match this (uL = pivot ? -d : d), or the kV fit's sign-aware kS
   * subtraction is wrong.  motor invert is applied below us in
   * drivebase_motor_set_duty, so use raw opposite signs here.
   */

  int16_t duty_left  = (int16_t)(pivot ? -duty : duty);
  int16_t duty_right = (int16_t)duty;
  while (elapsed < ms && !g_sysid_abort)
    {
      drivebase_motor_set_duty(DB_SIDE_LEFT,  duty_left);
      drivebase_motor_set_duty(DB_SIDE_RIGHT, duty_right);
      sysid_sleep_us(5000);
      sysid_obs_drain(o, duty_abs);
      elapsed += 5;
    }
}

/* Returns 0 on success, -errno (negative) on any failure.  Caller is
 * expected to propagate the magnitude as the CLI exit code (e.g.
 * `return -rc;`) so NSH `$?` carries the actual errno rather than the
 * old uniform `1`.  Plan D8 BLOCKING (Codex Round 1) was that EBUSY
 * was silently collapsed to 1 — script authors couldn't tell daemon-
 * attached apart from a usage typo.
 */

static int sysid_open_motors(void)
{
  int rc = sysid_check_daemon_off();
  /* ENOENT means the kernel build doesn't have the drivebase chardev
   * at all, so no daemon could possibly be running — proceed as safe.
   * (Codex Round 2 BLOCKING: previously the ENOENT path aborted with
   * `return -ENOENT`, which made SysId unusable on encoder-only kernel
   * configs where this verb is exactly the thing you want to run.)
   */
  if (rc == -ENOENT)
    {
      rc = 0;
    }
  if (rc < 0)
    {
      if (rc == -EBUSY)
        {
          fprintf(stderr,
                  "_sysid: drivebase daemon attached or status unreadable "
                  "— run `drivebase stop` first\n");
        }
      else
        {
          fprintf(stderr, "_sysid: daemon-check failed: %s\n",
                  strerror(-rc));
        }
      return rc;
    }
  rc = drivebase_motor_init();
  if (rc < 0)
    {
      fprintf(stderr, "_sysid: drivebase_motor_init: %s\n", strerror(-rc));
      return rc;
    }
  int rc_m_l = drivebase_motor_select_mode(DB_SIDE_LEFT,  2);
  int rc_m_r = drivebase_motor_select_mode(DB_SIDE_RIGHT, 2);
  if (rc_m_l < 0 || rc_m_r < 0)
    {
      fprintf(stderr,
              "_sysid: select_mode(2) failed L=%d R=%d (may not see encoder)\n",
              rc_m_l, rc_m_r);
      /* Continue anyway — the daemon also doesn't bail on this. */
    }
  usleep(30000);  /* match daemon's mode-2 settle */
  sysid_install_sigint();
  return 0;
}

static void sysid_close_motors(void)
{
  drivebase_motor_coast(DB_SIDE_LEFT);
  drivebase_motor_coast(DB_SIDE_RIGHT);
  /* Best-effort revert to mode 0 so the per-port LUMP kthreads stop
   * pumping 1 kHz encoder updates after the verb exits.  Same rationale
   * as drivebase_daemon.c's teardown.
   */
  drivebase_motor_select_mode(DB_SIDE_LEFT,  0);
  drivebase_motor_select_mode(DB_SIDE_RIGHT, 0);
  drivebase_motor_deinit();
  sysid_restore_sigint();
}

/****************************************************************************
 * Verb: ramp-ks
 ****************************************************************************/

static int sysid_verb_ramp_ks(int argc, char *argv[])
{
  bool pivot = sysid_extract_pivot(&argc, argv);

  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _sysid ramp-ks <step_pct01> "
              "[max_pct01=6000] [hold_ms=200] [pivot]\n"
              "  step_pct01  : duty increment per step (e.g. 50 = 0.5%%)\n"
              "  max_pct01   : abort at this duty if neither wheel moves\n"
              "                (default 60%% — SPIKE Medium Motor's real\n"
              "                breakaway is ~20-30%% under load; the\n"
              "                production daemon routinely commands\n"
              "                80-100%% during cruise.  Below 30%% the\n"
              "                ramp often terminates without finding\n"
              "                breakaway on one or both wheels.)\n"
              "  hold_ms     : settle time per step\n");
      return 1;
    }

  int32_t  step      = (int32_t)atoi(argv[0]);
  int32_t  max_pct01 = (argc >= 2) ? (int32_t)atoi(argv[1]) : 6000;
  uint32_t hold_ms   = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 200;

  if (step <= 0 || step > 1000 || max_pct01 <= 0 || max_pct01 > 10000 ||
      hold_ms < 50 || hold_ms > 5000)
    {
      fprintf(stderr, "_sysid ramp-ks: argument out of range\n");
      return 1;
    }

  int rc_open = sysid_open_motors();
  if (rc_open < 0) return -rc_open;

  int32_t vbat_mv  = sysid_read_vbat_mv();
  const struct db_battery_settings_s *bat = db_settings_battery();
  int32_t nominal_mv = bat ? bat->nominal_mv : 7200;

  printf("# ramp-ks: step=%ld max=%ld hold_ms=%lu vbat=%ld mV nominal=%ld mV "
         "mode=%s\n",
         (long)step, (long)max_pct01, (unsigned long)hold_ms,
         (long)vbat_mv, (long)nominal_mv, pivot ? "PIVOT" : "straight");
  if (pivot)
    {
      printf("# PIVOT MODE: drivebase rotates in place — ensure clearance to "
             "spin, NOT 2 m straight\n");
    }
  printf("# duty  vL_mdegps  vR_mdegps\n");

  /* Heap-allocate the observer pair — see the struct comment for why
   * (~4 KB struct can't live on the 4 KB CLI stack).
   */

  struct sysid_obs_s *o = calloc(1, sizeof(*o));
  if (o == NULL)
    {
      fprintf(stderr, "_sysid ramp-ks: out of memory\n");
      sysid_close_motors();
      return ENOMEM;
    }
  sysid_obs_init(o);

  /* Warm the observer with 0 duty for a few ticks so the first ramp
   * step sees a primed slope window.
   */

  sysid_hold_duty(o, 0, 50, pivot);

  int32_t kS_L = -1;
  int32_t kS_R = -1;
  for (int32_t d = step; d <= max_pct01; d += step)
    {
      sysid_hold_duty(o, d, hold_ms, pivot);
      int32_t vL = db_observer_v(&o->obs[DB_SIDE_LEFT]);
      int32_t vR = db_observer_v(&o->obs[DB_SIDE_RIGHT]);
      printf("%5ld  %9ld  %9ld\n", (long)d, (long)vL, (long)vR);
      /* Breakaway = wheel started moving regardless of direction.  In
       * pivot the left wheel runs backward (vL < 0), so gate on the
       * magnitude, not the signed value.
       */
      int32_t vL_ab = vL < 0 ? -vL : vL;
      int32_t vR_ab = vR < 0 ? -vR : vR;
      if (kS_L < 0 && vL_ab >  SYSID_V_MOVE_MDEGPS) kS_L = d;
      if (kS_R < 0 && vR_ab >  SYSID_V_MOVE_MDEGPS) kS_R = d;
      if (kS_L > 0 && kS_R > 0) break;
      if (g_sysid_abort) break;
    }

  sysid_close_motors();
  free(o);

  if (g_sysid_abort)
    {
      fprintf(stderr, "_sysid ramp-ks: aborted (Ctrl-C), motors coasted\n");
      return EINTR;
    }

  if (kS_L < 0 || kS_R < 0)
    {
      fprintf(stderr,
              "_sysid ramp-ks: did not reach v > %d mdeg/s by max=%ld "
              "(L found=%d R found=%d)\n",
              SYSID_V_MOVE_MDEGPS, (long)max_pct01,
              kS_L > 0, kS_R > 0);
      return 1;
    }

  int32_t kS_max     = kS_L > kS_R ? kS_L : kS_R;
  int32_t kS_nominal = sysid_normalise_to_nominal(kS_max, vbat_mv, nominal_mv);
  printf("kS_measured: L=%ld R=%ld (.01%% duty) max=%ld\n",
         (long)kS_L, (long)kS_R, (long)kS_max);
  printf("kS_nominal : %ld .01%% duty (normalised vbat=%ld -> %ld mV)\n",
         (long)kS_nominal, (long)vbat_mv, (long)nominal_mv);
  if (pivot)
    {
      /* Pivot floor friction differs from straight rolling friction, so
       * the heading kV fit must subtract the PIVOT breakaway, not the
       * straight one.  This is a kS_subtract input for `ramp-kv pivot`,
       * not the production ff_motor_kS (which is the common per-motor
       * friction assist).
       */
      printf("# Pivot breakaway: feed as kS_subtract to "
             "`drivebase _sysid ramp-kv ... pivot` (NOT ff_motor_kS)\n");
    }
  else
    {
      printf("# To apply: set ff_motor_kS = %ld in /mnt/flash/drivebase.cfg\n",
             (long)kS_nominal);
    }
  return 0;
}

/****************************************************************************
 * Verb: ramp-kv  (forward + reverse 11-segment sweep)
 ****************************************************************************/

/* Fit kV by least squares.  On success returns true and writes the
 * fitted gain (.01% per deg/s) to *kV_out + the sample count actually
 * used to *used_out.  Returns false if no samples pass the breakaway
 * gate (`|v_mdegps| < 30000`) — caller is expected to fail loudly
 * rather than report kV=0 (Codex Round 1 CONCERN).
 *
 * Uses int64 accumulators to absorb the worst-case sum of (duty_pct01
 * * v_dps) products without overflow even at full duty + peak v.
 * kS_subtract is the per-side friction term to remove from the duty
 * before fitting.
 */

static bool sysid_fit_kv_pair(int32_t *kV_out, int *used_out,
                              const int32_t *u, const int32_t *v_mdegps,
                              int n, int32_t kS_subtract)
{
  int64_t sum_uv = 0;
  int64_t sum_vv = 0;
  int     used   = 0;
  for (int i = 0; i < n; i++)
    {
      int32_t v_md = v_mdegps[i];
      int32_t v_ab = v_md < 0 ? -v_md : v_md;
      if (v_ab < SYSID_V_MOVE_MDEGPS) continue;        /* breakaway gate */
      int32_t sgn  = v_md > 0 ? 1 : -1;
      int32_t u_eff = u[i] - sgn * kS_subtract;
      int32_t v_dps = v_md / 1000;                     /* mdegps -> degps */
      sum_uv += (int64_t)u_eff * v_dps;
      sum_vv += (int64_t)v_dps * v_dps;
      used++;
    }
  if (used_out) *used_out = used;
  if (used == 0 || sum_vv == 0) return false;
  /* Round to nearest, preserving sign. */
  int64_t num = sum_uv;
  int32_t r;
  if (num >= 0) r = (int32_t)((num + sum_vv / 2) / sum_vv);
  else          r = (int32_t)((num - sum_vv / 2) / sum_vv);
  if (kV_out) *kV_out = r;
  return true;
}

static int sysid_verb_ramp_kv(int argc, char *argv[])
{
  bool pivot = sysid_extract_pivot(&argc, argv);

  if (argc < 3)
    {
      fprintf(stderr,
              "usage: drivebase _sysid ramp-kv <max_pct01> <duration_ms> "
              "<kS_subtract_pct01> [pivot]\n"
              "  max_pct01         : peak signed duty (forward + reverse)\n"
              "  duration_ms       : total time per direction\n"
              "  kS_subtract_pct01 : kS to subtract (run ramp-ks first;\n"
              "                      with pivot, use the pivot ramp-ks)\n"
              "  pivot             : identify HEADING axis (L=-duty,\n"
              "                      R=+duty in-place rotation) -> ff_head_kV\n");
      return 1;
    }

  int32_t  max_pct01    = (int32_t)atoi(argv[0]);
  uint32_t duration_ms  = (uint32_t)atoi(argv[1]);
  int32_t  kS_subtract  = (int32_t)atoi(argv[2]);

  if (max_pct01 <= 0 || max_pct01 > 10000 ||
      duration_ms < 1100 || duration_ms > 20000 ||
      kS_subtract < 0 || kS_subtract > 5000)
    {
      fprintf(stderr, "_sysid ramp-kv: argument out of range\n");
      return 1;
    }

  int rc_open = sysid_open_motors();
  if (rc_open < 0) return -rc_open;

  const int      N_SEG    = 11;
  uint32_t       seg_ms   = duration_ms / N_SEG;
  int32_t        vbat_mv  = sysid_read_vbat_mv();
  const struct db_battery_settings_s *bat = db_settings_battery();
  int32_t nominal_mv = bat ? bat->nominal_mv : 7200;

  printf("# ramp-kv: max=%ld duration_ms=%lu seg_ms=%lu N_SEG=%d "
         "kS_subtract=%ld vbat=%ld mV nominal=%ld mV mode=%s\n",
         (long)max_pct01, (unsigned long)duration_ms,
         (unsigned long)seg_ms, N_SEG,
         (long)kS_subtract, (long)vbat_mv, (long)nominal_mv,
         pivot ? "PIVOT" : "straight");
  if (pivot)
    {
      printf("# PIVOT MODE: drivebase rotates in place — ensure clearance to "
             "spin; FWD=+heading, REV=-heading\n");
    }
  printf("# direction  duty  vL_mdegps  vR_mdegps\n");

  struct sysid_obs_s *o = calloc(1, sizeof(*o));
  if (o == NULL)
    {
      fprintf(stderr, "_sysid ramp-kv: out of memory\n");
      sysid_close_motors();
      return ENOMEM;
    }
  sysid_obs_init(o);
  sysid_hold_duty(o, 0, 100, pivot);

  /* Capture both directions × 11 steps × 2 sides into a small static
   * buffer.  22 segments * 2 sides * 4 bytes = 176 bytes; file-scope
   * static avoids both stack pressure and a second heap allocation.
   *
   * Store the ACTUAL per-side applied duty (uL_buf/uR_buf), not the
   * command magnitude.  In straight mode both sides get +d so the two
   * buffers are identical; in pivot the left wheel is driven with -d,
   * so its fit must see -d (otherwise sysid_fit_kv_pair's sign-aware
   * kS subtraction is wrong and the fitted kV flips sign).
   */

  static int32_t uL_buf[2 * 11];          /* signed applied LEFT  duty */
  static int32_t uR_buf[2 * 11];          /* signed applied RIGHT duty */
  static int32_t vL_buf[2 * 11];
  static int32_t vR_buf[2 * 11];
  int             n = 0;

  for (int dir = 0; dir < 2; dir++)
    {
      int sign = dir == 0 ? +1 : -1;
      const char *dname = dir == 0 ? "FWD" : "REV";
      for (int k = 1; k <= N_SEG; k++)
        {
          int32_t d = sign * (max_pct01 * k) / N_SEG;
          sysid_hold_duty(o, d, seg_ms, pivot);
          if (g_sysid_abort) break;
          int32_t vL = db_observer_v(&o->obs[DB_SIDE_LEFT]);
          int32_t vR = db_observer_v(&o->obs[DB_SIDE_RIGHT]);
          uL_buf[n] = pivot ? -d : d;
          uR_buf[n] = d;
          vL_buf[n] = vL;
          vR_buf[n] = vR;
          n++;
          printf("%s  %5ld  %9ld  %9ld\n",
                 dname, (long)d, (long)vL, (long)vR);
        }
      /* Coast briefly between directions so the slope window is clean
       * before we reverse.
       */
      sysid_hold_duty(o, 0, 200, pivot);
      if (g_sysid_abort) break;
    }

  sysid_close_motors();
  free(o);

  if (g_sysid_abort)
    {
      fprintf(stderr, "_sysid ramp-kv: aborted (Ctrl-C), motors coasted\n");
      return EINTR;
    }

  /* Fit kV per side using all 22 samples.  Forward + reverse are
   * combined into a single least-squares fit (Plan: average L+R
   * pair fits at the end, then nominal-normalise).
   */

  int32_t kV_L = 0, kV_R = 0;
  int     used_L = 0, used_R = 0;
  bool ok_L = sysid_fit_kv_pair(&kV_L, &used_L, uL_buf, vL_buf, n, kS_subtract);
  bool ok_R = sysid_fit_kv_pair(&kV_R, &used_R, uR_buf, vR_buf, n, kS_subtract);

  printf("kV_fit usage: L=%d/%d samples R=%d/%d samples\n",
         used_L, n, used_R, n);

  if (!ok_L || !ok_R)
    {
      fprintf(stderr,
              "_sysid ramp-kv: insufficient samples passed breakaway gate "
              "(L used=%d R used=%d).  Try larger max_pct01 or longer "
              "duration_ms so cruise speed exceeds %d mdeg/s.\n",
              used_L, used_R, SYSID_V_MOVE_MDEGPS);
      return 1;
    }

  int32_t kV_avg = (kV_L + kV_R) / 2;
  int32_t kV_nominal = sysid_normalise_to_nominal(kV_avg, vbat_mv, nominal_mv);

  printf("kV_measured: L=%ld R=%ld avg=%ld (.01%% per deg/s)\n",
         (long)kV_L, (long)kV_R, (long)kV_avg);
  printf("kV_nominal : %ld .01%% per deg/s (normalised vbat=%ld -> %ld mV)\n",
         (long)kV_nominal, (long)vbat_mv, (long)nominal_mv);
  /* In a symmetric pivot the heading state velocity (sR-sL)/2 equals
   * the per-wheel speed, so the per-wheel kV fit IS the heading-axis kV
   * with no axle/wheel conversion (Issue #158).
   */
  printf("# To apply: set %s = %ld in /mnt/flash/drivebase.cfg\n",
         pivot ? "ff_head_kV" : "ff_dist_kV", (long)kV_nominal);
  return 0;
}

/****************************************************************************
 * Verb: ramp-ka  (step response, 63% rise time)
 ****************************************************************************/

static int sysid_verb_ramp_ka(int argc, char *argv[])
{
  bool pivot = sysid_extract_pivot(&argc, argv);

  if (argc < 3)
    {
      fprintf(stderr,
              "usage: drivebase _sysid ramp-ka <duty_pct01> <duration_ms> "
              "<kV_pct01_per_dps> [pivot]\n"
              "  duty_pct01        : step duty applied open-loop\n"
              "  duration_ms       : observation window\n"
              "  kV_pct01_per_dps  : kV from ramp-kv (drives kA formula;\n"
              "                      with pivot, use the pivot ramp-kv)\n"
              "  pivot             : identify HEADING axis (L=-duty,\n"
              "                      R=+duty in-place rotation) -> ff_head_kA\n");
      return 1;
    }

  int32_t  duty         = (int32_t)atoi(argv[0]);
  uint32_t duration_ms  = (uint32_t)atoi(argv[1]);
  int32_t  kV_value     = (int32_t)atoi(argv[2]);

  if (duty <= 0 || duty > 10000 ||
      duration_ms < 300 || duration_ms > 5000 ||
      kV_value <= 0 || kV_value > 1000)
    {
      fprintf(stderr, "_sysid ramp-ka: argument out of range\n");
      return 1;
    }

  int rc_open = sysid_open_motors();
  if (rc_open < 0) return -rc_open;

  /* Sample v_avg every 25 ms across the duration and capture into a
   * static buffer (200 samples max at 25 ms).  Step duty is applied
   * for the whole window; observer slope estimate gives us v(t).
   */

  enum { SAMP_DT_MS = 25, MAX_SAMP = 200 };
  static int32_t  t_ms_buf[MAX_SAMP];
  static int32_t  v_avg_buf[MAX_SAMP];
  uint32_t        n_samp = duration_ms / SAMP_DT_MS;
  if (n_samp > MAX_SAMP) n_samp = MAX_SAMP;

  int32_t vbat_mv = sysid_read_vbat_mv();
  printf("# ramp-ka: duty=%ld duration_ms=%lu samp_ms=%d kV=%ld vbat=%ld mV "
         "mode=%s\n",
         (long)duty, (unsigned long)duration_ms, SAMP_DT_MS,
         (long)kV_value, (long)vbat_mv, pivot ? "PIVOT" : "straight");
  if (pivot)
    {
      printf("# PIVOT MODE: drivebase rotates in place — ensure clearance to "
             "spin; v_avg is magnitude (|vL|+|vR|)/2\n");
    }
  printf("# t_ms  v_avg_mdegps\n");

  struct sysid_obs_s *o = calloc(1, sizeof(*o));
  if (o == NULL)
    {
      fprintf(stderr, "_sysid ramp-ka: out of memory\n");
      sysid_close_motors();
      return ENOMEM;
    }
  sysid_obs_init(o);

  /* Apply the step.  Pivot drives the left wheel backward (-duty) so the
   * chassis rotates in place; the right wheel stays +duty.  Forgetting
   * this direct-set path (it bypasses sysid_hold_duty) would silently
   * run a straight step under a `pivot` request.
   */

  drivebase_motor_set_duty(DB_SIDE_LEFT,  (int16_t)(pivot ? -duty : duty));
  drivebase_motor_set_duty(DB_SIDE_RIGHT, (int16_t)duty);

  uint64_t t0       = sysid_now_us();
  uint32_t collected = 0;
  for (uint32_t i = 0; i < n_samp; i++)
    {
      sysid_sleep_us(SAMP_DT_MS * 1000U);
      sysid_obs_drain(o, (uint32_t)duty);
      int32_t vL = db_observer_v(&o->obs[DB_SIDE_LEFT]);
      int32_t vR = db_observer_v(&o->obs[DB_SIDE_RIGHT]);
      /* Straight: both wheels share a sign, plain average is the body
       * speed.  Pivot: vL and vR are opposite-signed, so the signed
       * average is ~0 — average the magnitudes, which in a symmetric
       * pivot equals the heading state speed (sR-sL)/2.  v_avg stays
       * positive either way, so the 63.2 % rise detection below works
       * unchanged.
       */
      int32_t va;
      if (pivot)
        {
          int32_t vL_ab = vL < 0 ? -vL : vL;
          int32_t vR_ab = vR < 0 ? -vR : vR;
          va = (vL_ab + vR_ab) / 2;
        }
      else
        {
          va = (vL + vR) / 2;
        }
      t_ms_buf[i]  = (int32_t)((sysid_now_us() - t0) / 1000);
      v_avg_buf[i] = va;
      printf("%5ld  %9ld\n", (long)t_ms_buf[i], (long)va);
      collected++;
      if (g_sysid_abort) break;
    }

  sysid_close_motors();
  free(o);

  if (g_sysid_abort)
    {
      fprintf(stderr, "_sysid ramp-ka: aborted (Ctrl-C), motors coasted\n");
      return EINTR;
    }
  /* Use the actual sample count for the steady-state average — if the
   * window was truncated for any reason we still get a coherent τ.
   */
  n_samp = collected;

  /* Steady-state estimate: average of the last 25 % of samples.  τ =
   * first time v_avg reaches 63.2 % of v_steady — simple, robust to
   * the observer's own 30 ms smoothing.
   */

  uint32_t n_tail = n_samp / 4;
  if (n_tail < 2) n_tail = 2;
  int64_t  vsum = 0;
  for (uint32_t i = n_samp - n_tail; i < n_samp; i++) vsum += v_avg_buf[i];
  int32_t v_steady = (int32_t)(vsum / n_tail);

  int32_t v_threshold = (int32_t)((int64_t)v_steady * 632 / 1000);
  int32_t tau_ms      = -1;
  for (uint32_t i = 0; i < n_samp; i++)
    {
      if (v_steady > 0 ? v_avg_buf[i] >= v_threshold
                       : v_avg_buf[i] <= v_threshold)
        {
          tau_ms = t_ms_buf[i];
          break;
        }
    }

  if (tau_ms < 0 || v_steady == 0)
    {
      fprintf(stderr,
              "_sysid ramp-ka: failed to find 63%% rise (v_steady=%ld, "
              "threshold=%ld)\n",
              (long)v_steady, (long)v_threshold);
      return 1;
    }

  /* kA = kV * τ_ms / 1000.  Both ints; product max ~1000 * 5000 = 5e6,
   * safe in int32.  Round to nearest.
   */

  int32_t kA = (int32_t)(((int64_t)kV_value * tau_ms + 500) / 1000);

  printf("v_steady_avg = %ld mdeg/s\n",  (long)v_steady);
  printf("tau_ms       = %ld\n",          (long)tau_ms);
  printf("kA_estimate  = %ld .01%% per (deg/s^2)\n", (long)kA);
  printf("# To apply: set %s = %ld in /mnt/flash/drivebase.cfg\n",
         pivot ? "ff_head_kA" : "ff_dist_kA", (long)kA);
  return 0;
}

/****************************************************************************
 * Verb: vbat
 ****************************************************************************/

static int sysid_verb_vbat(int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  int32_t mv = sysid_read_vbat_mv();
  if (mv < 0)
    {
      fprintf(stderr, "_sysid vbat: read failed\n");
      return 1;
    }
  const struct db_battery_settings_s *bat = db_settings_battery();
  int32_t nominal_mv = bat ? bat->nominal_mv : 7200;
  /* Two ratios to avoid the label confusion Codex flagged:
   *   compensation_factor = nominal/vbat  — the multiplier Step 6.3
   *     applies to duty (greater than 1 when vbat is below nominal).
   *   normalisation_factor = vbat/nominal — the multiplier the SysId
   *     verbs use to rebase a raw kS/kV measurement to the nominal
   *     reference voltage.  Inverse of compensation_factor.
   * Both printed in milli-units (× 1000) since they straddle 1.0.
   */
  printf("vbat=%ld mV  nominal=%ld mV  "
         "compensation_factor=%ld/1000  normalisation_factor=%ld/1000\n",
         (long)mv, (long)nominal_mv,
         (long)((int64_t)nominal_mv * 1000 / mv),
         (long)((int64_t)mv * 1000 / nominal_mv));
  return 0;
}

/****************************************************************************
 * Public API
 ****************************************************************************/

int drivebase_sysid_cli(int argc, char *argv[])
{
  if (argc < 1)
    {
      fprintf(stderr,
              "usage: drivebase _sysid {ramp-ks|ramp-kv|ramp-ka|vbat} ... "
              "[pivot]\n"
              "  Ground / safety contract:\n"
              "    1) The drivebase MUST be on level ground.  In straight\n"
              "       mode (default) keep ~2 m of clear straight space; in\n"
              "       `pivot` mode the chassis rotates IN PLACE, so the\n"
              "       clearance you need is a clear circle to spin within,\n"
              "       NOT 2 m straight.  Free-spin (wheels-up) measurements\n"
              "       under-fit the real plant in both modes.\n"
              "    2) Have Ctrl-C ready — every verb coasts both motors\n"
              "       on a clean exit.  If your terminal swallows Ctrl-C\n"
              "       (e.g. picocom): run the verb in the background\n"
              "       with `&` and stop it with `kill -2 <pid>` (SIGINT,\n"
              "       NOT the default signal — `kill <pid>` sends SIGKILL\n"
              "       on most NuttX builds, which bypasses cleanup and\n"
              "       leaves motors at their last duty).\n"
              "    3) Re-run at different battery voltages to confirm\n"
              "       vbat normalisation is converging on a stable value.\n"
              "  Axis: default identifies the DISTANCE axis (both wheels\n"
              "    forward) -> ff_dist_*.  Append `pivot` to identify the\n"
              "    HEADING axis (L=-duty, R=+duty in-place rotation) ->\n"
              "    ff_head_*.  Run the two axes as separate sweeps.\n"
              "  Verb order: ramp-ks  ->  ramp-kv  ->  ramp-ka.\n"
              "    ramp-kv subtracts the kS measured by ramp-ks.\n"
              "    ramp-ka uses the kV measured by ramp-kv.\n"
              "    (In pivot, use the pivot ramp-ks/ramp-kv outputs — the\n"
              "    pivot floor friction differs from straight rolling.)\n");
      return 1;
    }

  const char *sub = argv[0];
  if (strcmp(sub, "ramp-ks") == 0)
    return sysid_verb_ramp_ks(argc - 1, &argv[1]);
  if (strcmp(sub, "ramp-kv") == 0)
    return sysid_verb_ramp_kv(argc - 1, &argv[1]);
  if (strcmp(sub, "ramp-ka") == 0)
    return sysid_verb_ramp_ka(argc - 1, &argv[1]);
  if (strcmp(sub, "vbat") == 0)
    return sysid_verb_vbat(argc - 1, &argv[1]);

  fprintf(stderr, "_sysid: unknown subcommand '%s'\n", sub);
  return 1;
}
