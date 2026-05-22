/****************************************************************************
 * apps/drivebase/drivebase_daemon.c
 *
 * Daemon lifecycle FSM.  See drivebase_daemon.h for the API contract.
 *
 * Teardown order (commits #9 + #11 retrospective):
 *   1. set running = false                  → RT thread will exit
 *      after its next sleep
 *   2. db_rt_stop                           → joins the RT thread
 *   3. db_chardev_handler_detach            → ioctl DAEMON_DETACH and
 *                                              close fd; the kernel
 *                                              chardev's close-cleanup
 *                                              also coasts both motors
 *                                              as a safety net
 *   4. drivebase_motor_deinit               → close LEGOSENSOR fds;
 *                                              legoport close-cleanup
 *                                              auto-coasts any port
 *                                              left in PWM state
 *
 * syslog() calls are silenced from the running daemon — they would
 * interleave with NSH stdout on the same USB CDC channel and corrupt
 * subsequent shell input.  Diagnostics live in DRIVEBASE_GET_STATUS
 * counters instead.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <nuttx/sensors/ioctl.h>

#include <arch/board/board_drivebase.h>

#include "drivebase_daemon.h"
#include "drivebase_drivebase.h"
#include "drivebase_motor.h"
#include "drivebase_chardev_handler.h"
#include "drivebase_imu.h"
#include "drivebase_rt.h"
#include "drivebase_config.h"
#include "drivebase_settings.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Task name kept distinct from the `drivebase` builtin so NSH's
 * builtin lookup is not shadowed by the running daemon task.
 */

#define DAEMON_TASK_NAME        "dbase_daemon"
#define DAEMON_TASK_PRIORITY    100
#define DAEMON_TASK_STACK       CONFIG_APP_DRIVEBASE_DAEMON_STACKSIZE

#define DAEMON_STALL_LIMIT      5    /* tick deadline misses to trigger    */
                                     /* stall watchdog                     */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct daemon_global_s
{
  /* Lifecycle */

  atomic_int                state;          /* enum db_daemon_state_e   */
  atomic_bool               running;
  pid_t                     pid;
  uint32_t                  wheel_d_um;
  uint32_t                  axle_t_um;
  pthread_mutex_t           lock;
  sem_t                     teardown_done;

  /* Subsystems — kept in BSS so the daemon task stack only needs to
   * cover ioctl / syslog frames, not the multi-KB drivebase + 2 servo
   * structs.
   */

  struct db_drivebase_s     db;
  struct db_chardev_handler_s handler;
  struct db_rt_s            rt;
  struct db_imu_s           imu;
  uint8_t                   use_gyro;
  bool                      imu_open;

  /* Stall watchdog */

  uint32_t                  consecutive_misses;
  uint32_t                  last_seen_misses;
};

/****************************************************************************
 * Private Data
 *
 * `g_daemon` is calloc'd from the user heap on first start and never
 * freed — keeps the 3+ KB struct out of the tight 64 KB usram .bss
 * region (Issue #95 / revert of #93).  The pointer is checked by every
 * accessor; pre-first-start state is reported as STOPPED with no
 * daemon attached.
 ****************************************************************************/

static struct daemon_global_s *g_daemon;

static int daemon_global_alloc(void)
{
  if (g_daemon != NULL)
    {
      return 0;
    }

  struct daemon_global_s *d = calloc(1, sizeof(*d));
  if (d == NULL)
    {
      return -ENOMEM;
    }

  atomic_init(&d->state,   DB_DAEMON_STOPPED);
  atomic_init(&d->running, false);
  d->pid = -1;
  pthread_mutex_init(&d->lock, NULL);
  sem_init(&d->teardown_done, 0, 0);
  g_daemon = d;
  return 0;
}

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int rt_tick_cb(uint64_t now_us_arg, void *arg)
{
  struct daemon_global_s *d = (struct daemon_global_s *)arg;

  /* Drain envelopes + dispatch + drivebase update. */

  db_chardev_handler_tick(&d->handler, now_us_arg);
  if (d->handler.configured)
    {
      db_drivebase_update(&d->db, now_us_arg);
    }
  if (d->imu_open)
    {
      db_imu_drain_and_update(&d->imu, now_us_arg);
    }

  /* Re-publish state after the drivebase update so the user sees
   * fresh distance / heading values immediately.  IMU heading is
   * folded in when use_gyro is non-NONE.
   */

  if (d->handler.configured)
    {
      struct drivebase_state_s st;
      db_drivebase_get_state(&d->db, &st);
      if (d->use_gyro != DRIVEBASE_USE_GYRO_NONE && d->imu_open)
        {
          int64_t h = db_imu_get_heading_mdeg(&d->imu);
          if (h >  INT32_MAX) h = INT32_MAX;
          if (h <  INT32_MIN) h = INT32_MIN;
          st.angle_mdeg = (int32_t)h;
        }
      st.tick_seq = d->rt.tick_count;
      db_chardev_handler_publish_state(&d->handler, &st);
    }

  /* Stall watchdog: emergency coast + initiate teardown after
   * DAEMON_STALL_LIMIT consecutive misses.
   */

  uint32_t misses_now = d->rt.deadline_miss_count;
  if (misses_now > d->last_seen_misses)
    {
      d->consecutive_misses += (misses_now - d->last_seen_misses);
      if (d->consecutive_misses >= DAEMON_STALL_LIMIT)
        {
          drivebase_motor_coast(DB_SIDE_LEFT);
          drivebase_motor_coast(DB_SIDE_RIGHT);
          atomic_store(&d->running, false);
        }
    }
  else
    {
      d->consecutive_misses = 0;
    }
  d->last_seen_misses = misses_now;

  return atomic_load(&d->running) ? 0 : -1;
}

static int daemon_task_main(int argc, char *argv[])
{
  struct daemon_global_s *d = g_daemon;

  /* CLI arguments (argv) may carry a 0 sentinel meaning "the CLI didn't
   * supply this positional arg, fall back to config or the compiled-in
   * default" (Issue #143).  Reset the live settings to their compiled
   * defaults first so each `drivebase start` begins from a clean slate
   * (without this step, keys present in the previous load but absent
   * from the current cfg would stick at the previously-loaded value).
   * Then load the cfg so any keys it specifies override the defaults.
   * Finally freeze() below locks the result in before the RT thread
   * starts, so the per-tick reader never races a writer.
   */

  db_settings_thaw();
  db_settings_reset_to_defaults();
  db_config_load(DB_CONFIG_DEFAULT_PATH);
  const struct db_config_start_defaults_s *cfg = db_config_get_start_defaults();

  uint32_t cli_wheel = (argc >= 2) ? (uint32_t)strtoul(argv[1], NULL, 10) : 0;
  uint32_t cli_axle  = (argc >= 3) ? (uint32_t)strtoul(argv[2], NULL, 10) : 0;
  uint32_t cli_tick  = (argc >= 4) ? (uint32_t)strtoul(argv[3], NULL, 10) : 0;

  uint32_t wheel_d_um =
      cli_wheel != 0 ? cli_wheel :
      cfg->wheel_d_um != 0 ? cfg->wheel_d_um : 56000;
  uint32_t axle_t_um  =
      cli_axle  != 0 ? cli_axle :
      cfg->axle_t_um  != 0 ? cfg->axle_t_um  : 112000;
  uint32_t tick_us    =
      cli_tick  != 0 ? cli_tick :
      cfg->tick_us    != 0 ? cfg->tick_us    : DB_RT_TICK_US_DEFAULT;
  uint32_t tick_ms    = tick_us / 1000;
  if (tick_ms == 0) tick_ms = 1;

  /* Surface the resolved geometry/tick so dmesg makes the cli/cfg/default
   * merge inspectable from outside (`drivebase _alg settings` only shows
   * its own CLI defaults, not the daemon's live geometry).
   */

  syslog(LOG_INFO,
         "drivebase: start wheel_d=%lu um axle_t=%lu um tick=%lu us\n",
         (unsigned long)wheel_d_um,
         (unsigned long)axle_t_um,
         (unsigned long)tick_us);

  d->wheel_d_um = wheel_d_um;
  d->axle_t_um  = axle_t_um;
  d->consecutive_misses = 0;
  d->last_seen_misses   = 0;
  d->use_gyro           = DRIVEBASE_USE_GYRO_NONE;
  d->imu_open           = false;

  atomic_store(&d->state, DB_DAEMON_INITIALISING);

  int rc = drivebase_motor_init();
  if (rc < 0) goto fail;

  drivebase_motor_select_mode(DB_SIDE_LEFT,  2);
  drivebase_motor_select_mode(DB_SIDE_RIGHT, 2);
  usleep(30000);

  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  uint64_t t0 = (uint64_t)ts.tv_sec * 1000000ULL +
                (uint64_t)ts.tv_nsec / 1000ULL;

  rc = db_drivebase_init(&d->db, d->wheel_d_um, d->axle_t_um, tick_ms);
  if (rc < 0) goto fail_motor;
  rc = db_drivebase_reset(&d->db, t0);
  if (rc < 0) goto fail_motor;

  int port_l = drivebase_motor_port_idx(DB_SIDE_LEFT);
  int port_r = drivebase_motor_port_idx(DB_SIDE_RIGHT);

  /* Lock settings before the RT thread starts.  After this point, the
   * RT tick (which dereferences the pointer returned by
   * db_settings_pid_gains() on every iteration) is the only consumer,
   * and db_settings_set_* will reject writes with -EBUSY.  This
   * structurally prevents writer/reader races without a runtime lock.
   */

  db_settings_freeze();

  rc = db_chardev_handler_attach(&d->handler, &d->db, port_l, port_r,
                                 DRIVEBASE_ON_COMPLETION_COAST);
  if (rc < 0) goto fail_motor;
  d->handler.configured = true;
  d->handler.wheel_d_um = d->wheel_d_um;
  d->handler.axle_t_um  = d->axle_t_um;
  d->handler.tick_ms    = tick_ms;

  /* Phase 2.5 (#145) — layer 2 of the ODR rollback 3-layer defense:
   * unconditionally force the LSM6DSL ODR back to 833 Hz before we
   * subscribe.  btsensor IMU_CAP mode switches the driver to 104 Hz
   * during a Tedaldi capture session; if that process crashed or was
   * killed before its layer-1 cleanup ran, the driver state would
   * persist into a fresh `drivebase start` and silently corrupt
   * integration.  Issuing SET unconditionally here means a cold-start
   * after a btsensor crash always lands at the expected 833 Hz, and
   * a `drivebase start` racing an active IMU_CAP session wins (any
   * already-captured frame data is rejected by the host via the
   * per-sample fsr_*_idx change).
   *
   * Transient O_WRONLY fd so we don't subscribe — sensor upper-half
   * auto-activates on O_RDOK, and we want the topic to come up only
   * via db_imu_open() below.
   */

  int odr_fd = open("/dev/uorb/sensor_imu0", O_WRONLY);
  if (odr_fd >= 0)
    {
      if (ioctl(odr_fd, SNIOC_SETSAMPLERATE, 833) < 0)
        {
          syslog(LOG_WARNING,
                 "drivebase: ODR force 833 Hz failed errno %d\n", errno);
        }

      close(odr_fd);
    }
  else
    {
      syslog(LOG_WARNING,
             "drivebase: ODR force open errno %d (continuing)\n", errno);
    }

  /* IMU is best-effort — the daemon runs encoder-only if open fails. */

  if (db_imu_open(&d->imu) == 0)
    {
      d->imu_open = true;
    }

  db_rt_init(&d->rt, tick_us);
  rc = db_rt_start(&d->rt, CONFIG_APP_DRIVEBASE_RT_PRIORITY,
                   rt_tick_cb, d);
  if (rc < 0) goto fail_chardev;

  atomic_store(&d->running, true);
  atomic_store(&d->state, DB_DAEMON_RUNNING);

  /* Idle loop — the RT thread drives all real work; we wake every
   * 50 ms to publish status counters and check for stop request.
   */

  while (atomic_load(&d->running))
    {
      usleep(50000);
      db_chardev_handler_publish_status(&d->handler);

      /* If user requested DRIVEBASE_JITTER_RESET, claim and apply it
       * before the snapshot so the published cache reflects the post-
       * reset state in the same idle wake.
       */

      int reset_req = 0;
      if (ioctl(d->handler.fd, DRIVEBASE_DAEMON_CLAIM_JITTER_RESET,
                (unsigned long)&reset_req) == 0 && reset_req)
        {
          db_rt_reset_jitter(&d->rt);
        }

      /* Snapshot RT loop counters and push to the kernel cache so
       * `drivebase jitter` reads live data instead of zeros.  20 Hz
       * is plenty for diagnostics; the snapshot itself is a
       * mutex-protected memcpy of an 8-bucket histogram + 3 counters.
       */

      struct drivebase_jitter_dump_s jdump;
      db_rt_get_jitter(&d->rt, &jdump);
      db_chardev_handler_publish_jitter(&d->handler, &jdump);
    }

  atomic_store(&d->state, DB_DAEMON_TEARDOWN);

  /* Teardown order matters — see header comment. */

  db_rt_stop(&d->rt, 100);
  drivebase_motor_coast(DB_SIDE_LEFT);
  drivebase_motor_coast(DB_SIDE_RIGHT);

  /* Return motors to mode 0 so the per-port LUMP kthreads stop
   * processing 1 kHz mode-2 encoder events.  Without this the
   * lump-A / lump-B kthreads stay pinned at ~20 % CPU after every
   * daemon stop and the scheduling pressure accumulates across
   * multiple start/stop cycles, eventually destabilising the system.
   */

  drivebase_motor_select_mode(DB_SIDE_LEFT,  0);
  drivebase_motor_select_mode(DB_SIDE_RIGHT, 0);
  if (d->imu_open) db_imu_close(&d->imu);
  db_chardev_handler_detach(&d->handler);
  drivebase_motor_deinit();

  atomic_store(&d->state, DB_DAEMON_STOPPED);
  d->pid = -1;
  sem_post(&d->teardown_done);
  return 0;

fail_chardev:
  if (d->imu_open) db_imu_close(&d->imu);
  db_chardev_handler_detach(&d->handler);
fail_motor:
  drivebase_motor_deinit();
fail:
  atomic_store(&d->state, DB_DAEMON_STOPPED);
  d->pid = -1;
  sem_post(&d->teardown_done);
  return -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int drivebase_daemon_start(uint32_t wheel_d_um, uint32_t axle_t_um,
                           uint32_t tick_us)
{
  int rc = daemon_global_alloc();
  if (rc < 0)
    {
      return rc;
    }

  if (atomic_load(&g_daemon->state) != DB_DAEMON_STOPPED)
    {
      return -EALREADY;
    }

  pthread_mutex_lock(&g_daemon->lock);

  /* Drain any stale teardown_done from a previous start/stop cycle. */

  while (sem_trywait(&g_daemon->teardown_done) == 0) { }

  /* Pass wheel/axle/tick_us to the daemon task via argv strings. */

  if (tick_us == 0) tick_us = DB_RT_TICK_US_DEFAULT;
  static char wheel_str[12], axle_str[12], tick_str[12];
  snprintf(wheel_str, sizeof(wheel_str), "%lu",
           (unsigned long)wheel_d_um);
  snprintf(axle_str,  sizeof(axle_str),  "%lu",
           (unsigned long)axle_t_um);
  snprintf(tick_str,  sizeof(tick_str),  "%lu",
           (unsigned long)tick_us);
  static char *argv[] = { wheel_str, axle_str, tick_str, NULL };

  int pid = task_create(DAEMON_TASK_NAME, DAEMON_TASK_PRIORITY,
                        DAEMON_TASK_STACK, daemon_task_main, argv);
  if (pid < 0)
    {
      pthread_mutex_unlock(&g_daemon->lock);
      return -errno;
    }

  g_daemon->pid = (pid_t)pid;
  pthread_mutex_unlock(&g_daemon->lock);
  return pid;
}

int drivebase_daemon_stop(uint32_t timeout_ms)
{
  if (g_daemon == NULL ||
      atomic_load(&g_daemon->state) == DB_DAEMON_STOPPED)
    {
      return -EAGAIN;
    }

  atomic_store(&g_daemon->running, false);

  if (timeout_ms == 0) timeout_ms = 2000;

  /* Wait for daemon to post teardown_done. */

  struct timespec deadline;
  clock_gettime(CLOCK_REALTIME, &deadline);
  deadline.tv_sec  += timeout_ms / 1000;
  deadline.tv_nsec += (long)(timeout_ms % 1000) * 1000000L;
  if (deadline.tv_nsec >= 1000000000L)
    {
      deadline.tv_nsec -= 1000000000L;
      deadline.tv_sec  += 1;
    }
  if (sem_timedwait(&g_daemon->teardown_done, &deadline) < 0)
    {
      return -ETIMEDOUT;
    }
  return 0;
}

enum db_daemon_state_e drivebase_daemon_state(void)
{
  return g_daemon == NULL ? DB_DAEMON_STOPPED
                          : (enum db_daemon_state_e)
                              atomic_load(&g_daemon->state);
}

int drivebase_daemon_get_pid(void)
{
  return g_daemon == NULL ? -1 : g_daemon->pid;
}
