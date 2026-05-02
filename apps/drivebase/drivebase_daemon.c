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
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#include <arch/board/board_drivebase.h>

#include "drivebase_daemon.h"
#include "drivebase_drivebase.h"
#include "drivebase_motor.h"
#include "drivebase_chardev_handler.h"
#include "drivebase_imu.h"
#include "drivebase_rt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Task name kept distinct from the `drivebase` builtin so NSH's
 * builtin lookup is not shadowed by the running daemon task.
 */

#define DAEMON_TASK_NAME        "dbase_daemon"
#define DAEMON_TASK_PRIORITY    100
#define DAEMON_TASK_STACK       8192    /* TLS_MAXSTACK = 1 << 13 = 8 KB */

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
  uint32_t                  wheel_d_mm;
  uint32_t                  axle_t_mm;
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
 ****************************************************************************/

static struct daemon_global_s g_daemon =
{
  .state    = ATOMIC_VAR_INIT(DB_DAEMON_STOPPED),
  .running  = ATOMIC_VAR_INIT(false),
  .pid      = -1,
};

static bool g_daemon_inited;

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
      ioctl(d->handler.fd, DRIVEBASE_DAEMON_PUBLISH_STATE,
            (unsigned long)&st);
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
  struct daemon_global_s *d = &g_daemon;

  uint32_t wheel_d_mm = (argc >= 2) ? (uint32_t)atoi(argv[1]) : 56;
  uint32_t axle_t_mm  = (argc >= 3) ? (uint32_t)atoi(argv[2]) : 112;

  d->wheel_d_mm = wheel_d_mm;
  d->axle_t_mm  = axle_t_mm;
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

  rc = db_drivebase_init(&d->db, d->wheel_d_mm, d->axle_t_mm);
  if (rc < 0) goto fail_motor;
  rc = db_drivebase_reset(&d->db, t0);
  if (rc < 0) goto fail_motor;

  int port_l = drivebase_motor_port_idx(DB_SIDE_LEFT);
  int port_r = drivebase_motor_port_idx(DB_SIDE_RIGHT);

  rc = db_chardev_handler_attach(&d->handler, &d->db, port_l, port_r,
                                 DRIVEBASE_ON_COMPLETION_COAST);
  if (rc < 0) goto fail_motor;
  d->handler.configured = true;
  d->handler.wheel_d_mm = d->wheel_d_mm;
  d->handler.axle_t_mm  = d->axle_t_mm;

  /* IMU is best-effort — the daemon runs encoder-only if open fails. */

  if (db_imu_open(&d->imu) == 0)
    {
      d->imu_open = true;
    }

  db_rt_init(&d->rt);
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
    }

  atomic_store(&d->state, DB_DAEMON_TEARDOWN);

  /* Teardown order matters — see header comment. */

  db_rt_stop(&d->rt, 100);
  drivebase_motor_coast(DB_SIDE_LEFT);
  drivebase_motor_coast(DB_SIDE_RIGHT);
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

int drivebase_daemon_start(uint32_t wheel_d_mm, uint32_t axle_t_mm)
{
  if (!g_daemon_inited)
    {
      pthread_mutex_init(&g_daemon.lock, NULL);
      sem_init(&g_daemon.teardown_done, 0, 0);
      g_daemon_inited = true;
    }

  if (atomic_load(&g_daemon.state) != DB_DAEMON_STOPPED)
    {
      return -EALREADY;
    }

  pthread_mutex_lock(&g_daemon.lock);

  /* Drain any stale teardown_done from a previous start/stop cycle. */

  while (sem_trywait(&g_daemon.teardown_done) == 0) { }

  /* Pass wheel/axle to the daemon task via argv strings. */

  static char wheel_str[12], axle_str[12];
  snprintf(wheel_str, sizeof(wheel_str), "%lu", (unsigned long)wheel_d_mm);
  snprintf(axle_str,  sizeof(axle_str),  "%lu", (unsigned long)axle_t_mm);
  static char *argv[] = { wheel_str, axle_str, NULL };

  int pid = task_create(DAEMON_TASK_NAME, DAEMON_TASK_PRIORITY,
                        DAEMON_TASK_STACK, daemon_task_main, argv);
  if (pid < 0)
    {
      pthread_mutex_unlock(&g_daemon.lock);
      return -errno;
    }

  g_daemon.pid = (pid_t)pid;
  pthread_mutex_unlock(&g_daemon.lock);
  return pid;
}

int drivebase_daemon_stop(uint32_t timeout_ms)
{
  if (!g_daemon_inited ||
      atomic_load(&g_daemon.state) == DB_DAEMON_STOPPED)
    {
      return -EAGAIN;
    }

  atomic_store(&g_daemon.running, false);

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
  if (sem_timedwait(&g_daemon.teardown_done, &deadline) < 0)
    {
      return -ETIMEDOUT;
    }
  return 0;
}

enum db_daemon_state_e drivebase_daemon_state(void)
{
  return (enum db_daemon_state_e)atomic_load(&g_daemon.state);
}

int drivebase_daemon_get_pid(void)
{
  return g_daemon.pid;
}
