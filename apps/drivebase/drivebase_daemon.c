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
#include "drivebase_imu.h"          /* DB_IMU_DEFAULT_STALE_THRESHOLD_US */
#include "drivebase_rt.h"
#include "drivebase_config.h"
#include "drivebase_settings.h"
#include "drivebase_battery.h"      /* Phase 6 Step 6.3 sag correction   */

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
  bool                      imu_open;

  /* Stall watchdog */

  uint32_t                  consecutive_misses;
  uint32_t                  last_seen_misses;

  /* #154 motor-port self-recovery — throttle flags so a stuck-unplugged
   * motor logs once per failure transition, not every 50 ms idle wake.
   */

  bool                      reclaim_ack_warned;
  bool                      reclaim_fail_warned[DB_SIDE_NUM];
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

  /* #154 freeze gate.  While the non-RT idle loop reclaims a lost motor
   * port it raises io_frozen and waits for this acknowledgement before it
   * closes/reopens any fd or resets db state.  When frozen, touch NOTHING
   * that reads/writes `db` or `g_motor[]` — that means skipping command
   * dispatch, the IMU drain, the drivebase update, the NORMAL db-derived
   * state publish, AND the stall-watchdog coast below — so the reclaim has
   * exclusive access.  We DO still publish a minimal keep-alive snapshot
   * (built without reading db) every tick so the kernel stale-daemon
   * watchdog (50 ms) cannot detach us during a long reclaim/resync.  Still
   * honour a stop request so teardown is never blocked.
   */

  if (drivebase_motor_io_frozen())
    {
      drivebase_motor_set_rt_idle(true);

      /* Keep the kernel stale-daemon watchdog (DB_STALE_THRESHOLD_MS = 50 ms)
       * fed while the non-RT reclaim holds the freeze.  A port resync +
       * reclaim can take >50 ms (observed ~240 ms), and we must not run the
       * normal publish here because it reads `db`, which the daemon is
       * resetting under the freeze.  Publish a minimal keep-alive snapshot
       * built WITHOUT touching db / g_motor[] — same chardev fd, no shared
       * mutable state with the reclaim — so the daemon stays attached for
       * any freeze duration.  actuation_fault=1 marks recovery in progress.
       */

      struct drivebase_state_s ka;
      memset(&ka, 0, sizeof(ka));
      ka.actuation_fault = 1;
      ka.tick_seq        = d->rt.tick_count;
      db_chardev_handler_publish_state(&d->handler, &ka);

      return atomic_load(&d->running) ? 0 : -1;
    }

  /* Drain envelopes + dispatch + IMU drain + drivebase update.  Phase
   * 3b moves the IMU drain ahead of db_drivebase_update so the heading
   * PID injection (which reads db_imu_get_heading_mdeg) sees a sample
   * from the same tick, not the previous one.  See Phase 3b plan Step 0.
   */

  db_chardev_handler_tick(&d->handler, now_us_arg);
  if (d->imu_open)
    {
      db_imu_drain_and_update(&d->imu, now_us_arg);
    }
  if (d->handler.configured)
    {
      db_drivebase_update(&d->db, now_us_arg);
    }

  /* Re-publish state after the drivebase update so the user sees
   * fresh distance / heading values immediately.
   *
   * Phase 3b (#148) publish overwrite — fold the IMU heading into
   * `angle_mdeg` when EITHER of the two scenarios holds:
   *   1. A motion is in flight with latched 3D mode (the PID is
   *      already running on the gyro state input — publish must agree
   *      so reset/SMART/get-state stay coherent with the PID).
   *   2. The drivebase is idle but the user already armed 3D mode and
   *      we hold a valid origin.  This is the manual-spin case
   *      ("hand-rotate the robot and watch angle_mdeg track").
   * In both cases the IMU must be online and not stale; otherwise the
   * encoder-derived publish from db_drivebase_get_state passes through
   * unchanged (transient bus-stall fallback).
   *
   * The same gyro_origin_mdeg baseline used by the PID injection feeds
   * the publish overwrite, so PID state and user-visible heading share
   * a single source of truth (SSOT).  Saturating to int32 is safe: at
   * ±5.96 M deg the saturation only fires after months of continuous
   * spin, far past anything the user would interpret as a fresh heading.
   */

  if (d->handler.configured)
    {
      struct drivebase_state_s st;
      db_drivebase_get_state(&d->db, &st);

      bool publish_gyro =
          (d->db.use_gyro_latched   == DRIVEBASE_USE_GYRO_3D) ||
          (d->db.use_gyro_requested == DRIVEBASE_USE_GYRO_3D &&
           d->db.gyro_origin_valid);

      if (publish_gyro && d->imu_open && d->db.gyro_origin_valid &&
          !db_imu_is_stale(&d->imu, now_us_arg,
                           DB_IMU_DEFAULT_STALE_THRESHOLD_US))
        {
          int64_t raw = db_imu_get_heading_mdeg(&d->imu);
          int64_t rel = raw - d->db.gyro_origin_mdeg;
          if (rel >  INT32_MAX) rel = INT32_MAX;
          if (rel <  INT32_MIN) rel = INT32_MIN;
          st.angle_mdeg = (int32_t)rel;
        }
      st.tick_seq        = d->rt.tick_count;
      st.actuation_fault = drivebase_motor_reclaim_pending() != 0;  /* #154 */
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

/* #154 motor-port self-recovery, run from the NON-RT idle loop.  When a
 * set_duty / drain on the RT path detects a lost LUMP port (stale
 * LEGOSENSOR CLAIM after a disconnect+resync), it arms a per-side reclaim
 * request.  Here we freeze the RT tick (fail-closed on its ack), reclaim
 * the affected side(s) with a fresh CLAIM, and once both are healthy reset
 * the drivebase epoch — restoring motion WITHOUT a stop/start.
 */

static void daemon_try_reclaim(struct daemon_global_s *d)
{
  if (drivebase_motor_reclaim_pending() == 0)
    {
      return;
    }

  /* Freeze the RT tick and wait (fail-closed) for it to acknowledge it is
   * idle at the freeze gate before touching any motor fd.  If the ack does
   * not arrive (RT wedged or mid-stop) we do NOT close/reopen/reset —
   * release the freeze and retry on the next idle wake.
   */

  drivebase_motor_set_rt_idle(false);
  drivebase_motor_set_io_frozen(true);

  struct timespec fz0;
  clock_gettime(CLOCK_MONOTONIC, &fz0);

  bool acked = false;
  for (int i = 0; i < 12 && atomic_load(&d->running); i++)
    {
      if (drivebase_motor_rt_idle())
        {
          acked = true;
          break;
        }
      usleep(2000);
    }

  if (!acked)
    {
      drivebase_motor_set_io_frozen(false);
      if (!d->reclaim_ack_warned)
        {
          syslog(LOG_WARNING, "drivebase: #154 reclaim ack timeout\n");
          d->reclaim_ack_warned = true;
        }
      return;
    }
  d->reclaim_ack_warned = false;

  /* RT is provably idle (early-returns at the freeze gate, where it now
   * publishes a keep-alive snapshot every tick to keep the kernel stale
   * watchdog fed).  Safe to reclaim/reset db + g_motor[] exclusively here.
   */

  unsigned req = drivebase_motor_reclaim_pending();
  for (int side = 0; side < DB_SIDE_NUM && atomic_load(&d->running); side++)
    {
      if ((req & (1u << (unsigned)side)) == 0)
        {
          continue;
        }

      int rc = drivebase_motor_reclaim((enum db_side_e)side);
      if (rc == 0)
        {
          drivebase_motor_clear_reclaim((enum db_side_e)side);
          d->reclaim_fail_warned[side] = false;
          syslog(LOG_INFO, "drivebase: #154 reclaimed motor side=%d\n", side);
        }
      else if (!d->reclaim_fail_warned[side])
        {
          /* Still physically unplugged (-ENODEV) or another error — leave
           * the bit set and retry next cycle; log once per transition.
           */

          syslog(LOG_WARNING,
                 "drivebase: #154 reclaim side=%d rc=%d (retry)\n", side, rc);
          d->reclaim_fail_warned[side] = true;
        }
    }

  /* Re-baseline only once BOTH sides are healthy — a differential
   * drivebase with one dead motor cannot be meaningfully reset.  The reset
   * re-seeds the encoder baseline + observer (kills the phantom-velocity
   * epoch) and aborts the in-flight command (done=true), like stop/start
   * but without tearing the daemon down.
   */

  if (drivebase_motor_reclaim_pending() == 0)
    {
      struct timespec ts;
      clock_gettime(CLOCK_MONOTONIC, &ts);
      uint64_t now_us = (uint64_t)ts.tv_sec * 1000000ull +
                        (uint64_t)ts.tv_nsec / 1000ull;
      db_drivebase_reset(&d->db, now_us);
      uint32_t froze_ms = (uint32_t)((ts.tv_sec - fz0.tv_sec) * 1000L +
                                     (ts.tv_nsec - fz0.tv_nsec) / 1000000L);
      syslog(LOG_INFO,
             "drivebase: #154 recovery complete (epoch reset, froze %lu ms)\n",
             (unsigned long)froze_ms);
    }

  drivebase_motor_set_io_frozen(false);
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
  d->imu_open           = false;

  /* Mark running BEFORE db_rt_start (below) launches the RT thread.
   * db_rt_start creates the SCHED_FIFO prio-220 RT thread, whose first
   * rt_tick_cb returns -1 (and so exits the thread) if it observes
   * `running == false`.  Setting running here — rather than after
   * db_rt_start — closes a TOCTOU where, if this prio-100 task is
   * preempted (e.g. a LUMP IRQ burst) past the RT thread's first tick
   * before the store, the RT thread exits at tick 1 and the daemon idles
   * with a dead publisher: the kernel /dev/drivebase watchdog then detaches
   * it, leaving a permanent zombie (status all-zero, set-gyro -ENOTCONN,
   * start -EALREADY).  See Issue #153.  `running` is only read by
   * rt_tick_cb and the idle loop, both reached only after db_rt_start, so
   * setting it this early is safe; a stop during INITIALISING clears it and
   * is honoured by the pre-db_rt_start check below.
   */

  atomic_store(&d->running, true);
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

  /* Phase 6 Step 6.3 (#152) — seed the battery atomic with the live
   * nominal_mv (the cfg may have overridden the compiled default just
   * above) so the first RT tick reads ×1 correction.  Opening the
   * battery fd is best-effort: failure leaves the snapshot at the seed
   * value, which the RT side handles as ×1 (no correction).  Must run
   * AFTER db_settings_freeze (settings now stable) and BEFORE
   * db_rt_start (no concurrent reader yet).
   */

  const struct db_battery_settings_s *bat_cfg = db_settings_battery();
  db_battery_init(bat_cfg->nominal_mv);
  db_battery_open();

  rc = db_chardev_handler_attach(&d->handler, &d->db, port_l, port_r,
                                 DRIVEBASE_ON_COMPLETION_COAST);
  if (rc < 0) goto fail_battery;
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
      /* Phase 3b (#148) — attach the live IMU so heading PID injection
       * + publish overwrite can pull world-vertical mdeg.  Safe to call
       * before the RT thread starts; later set_use_gyro / command-start
       * latches will negotiate when to actually use the data.
       */

      db_drivebase_attach_imu(&d->db, &d->imu);
    }

  /* Phase 3b (#148) — apply persisted boot-time gyro mode.  Madgwick
   * has not converged yet (calibrated false), so set_use_gyro will
   * record `requested` and defer the latched / origin capture to the
   * first command-start.  Skipping the call entirely when use_gyro_plus1
   * is unset preserves the prior default of DRIVEBASE_USE_GYRO_NONE.
   */

  if (cfg->use_gyro_plus1 != 0)
    {
      uint8_t boot_mode = (uint8_t)(cfg->use_gyro_plus1 - 1);
      int set_rc = db_drivebase_set_use_gyro(&d->db, boot_mode, t0);
      if (set_rc < 0)
        {
          syslog(LOG_WARNING,
                 "drivebase: boot use_gyro=%u rejected rc=%d\n",
                 (unsigned)boot_mode, set_rc);
        }
    }

  db_rt_init(&d->rt, tick_us);

  /* poll_tick is declared before the `goto teardown` below so the jump
   * does not skip an initialised auto variable.
   */

  uint32_t poll_tick = 0;

  /* If a stop arrived during INITIALISING, `running` is already false.
   * Honour it without ever starting the RT thread (avoids creating a
   * thread that would immediately exit) — go straight to teardown.
   */

  if (!atomic_load(&d->running))
    {
      atomic_store(&d->state, DB_DAEMON_TEARDOWN);
      goto teardown;
    }

  rc = db_rt_start(&d->rt, CONFIG_APP_DRIVEBASE_RT_PRIORITY,
                   rt_tick_cb, d);
  if (rc < 0) goto fail_chardev;

  atomic_store(&d->state, DB_DAEMON_RUNNING);

  /* Idle loop — the RT thread drives all real work; we wake every
   * 50 ms to publish status counters and check for stop request.
   */

  while (atomic_load(&d->running))
    {
      usleep(50000);

      /* #154: recover a lost motor port (close+reopen+re-CLAIM under a
       * freeze handshake) before the periodic status publish, so a fault
       * is cleared within one idle cycle (~50 ms).
       */

      daemon_try_reclaim(d);

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

      /* Phase 6 Step 6.3 (#152) — battery sag poll at 200 ms cadence
       * (every 4th idle wake = 5 Hz).  The ioctl runs on the daemon
       * thread so the RT path never sees mutex-protected upper-half
       * latency; the EMA (τ ≈ 1.6 s) absorbs single-sample noise; the
       * RT side reads a single _Atomic int32 with no locks.
       */

      if ((++poll_tick & 0x3) == 0)
        {
          db_battery_poll();   /* failures already log + auto-suppress */
        }
    }

  atomic_store(&d->state, DB_DAEMON_TEARDOWN);

teardown:

  /* Teardown order matters — see header comment.  db_rt_stop is a no-op
   * when the RT thread was never started (the stop-during-init path above
   * jumps here before db_rt_start).
   */

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
  db_battery_close();
  db_chardev_handler_detach(&d->handler);
  drivebase_motor_deinit();

  atomic_store(&d->state, DB_DAEMON_STOPPED);
  d->pid = -1;
  sem_post(&d->teardown_done);
  return 0;

fail_chardev:
  if (d->imu_open) db_imu_close(&d->imu);
  db_chardev_handler_detach(&d->handler);
fail_battery:
  db_battery_close();
fail_motor:
  drivebase_motor_deinit();
fail:
  atomic_store(&d->running, false);
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
