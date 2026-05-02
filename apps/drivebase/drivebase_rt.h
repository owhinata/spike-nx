/****************************************************************************
 * apps/drivebase/drivebase_rt.h
 *
 * 5 ms RT control task for the drivebase daemon (Issue #77 commit #8).
 * Spawns a pthread at SCHED_FIFO priority that wakes every 5 ms via
 * clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ...) and invokes a
 * caller-supplied tick callback.  Jitter (wake-time vs intended
 * deadline) is sampled into an 8-bucket histogram + max_lag for the
 * DRIVEBASE_JITTER_DUMP ioctl path.
 *
 * The task is intentionally generic — both the daemon (commit #11) and
 * the standalone `_rt` test verb call it the same way.  Only the
 * callback differs.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_RT_H
#define __APPS_DRIVEBASE_DRIVEBASE_RT_H

#include <pthread.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board_drivebase.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_RT_TICK_US      5000        /* 5 ms control loop period       */
#define DB_RT_DEADLINE_US  1000        /* lag threshold above which we   */
                                       /* count a deadline miss          */

/****************************************************************************
 * Types
 ****************************************************************************/

/* Tick callback.  Returns 0 to continue, < 0 to request immediate stop
 * (the task records the return value as `last_cb_error` and exits its
 * loop).  Called from the RT thread context, so observe the same
 * "no syslog/printf/malloc/long-mutex" rules the daemon documentation
 * imposes.
 */

typedef int (*db_rt_tick_cb_t)(uint64_t now_us, void *arg);

struct db_rt_s
{
  pthread_t          thread;
  bool               started;
  atomic_bool        running;
  int                priority;
  int                last_cb_error;

  db_rt_tick_cb_t    tick_cb;
  void              *tick_cb_arg;

  /* Counters / jitter */

  uint32_t           tick_count;
  uint32_t           deadline_miss_count;
  uint32_t           jitter_hist[DRIVEBASE_JITTER_BUCKETS];
                     /* <50 / 50-100 / 100-200 / 200-500 /
                      * 500-1k / 1k-2k / 2k-5k / 5k+                      */
  uint32_t           max_lag_us;
  pthread_mutex_t    stats_lock;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

void db_rt_init(struct db_rt_s *rt);

/* Spawn the RT thread.  `priority` is the pthread SCHED_FIFO value
 * (CONFIG_APP_DRIVEBASE_RT_PRIORITY recommended, default 220).
 * Returns 0 on success or a negated errno.
 */

int  db_rt_start(struct db_rt_s *rt, int priority,
                 db_rt_tick_cb_t cb, void *arg);

/* Stop and join the RT thread.  Safe to call from any context.  Sets
 * running = false, waits up to `timeout_ms` for the thread to finish.
 */

void db_rt_stop(struct db_rt_s *rt, uint32_t timeout_ms);

/* Snapshot the jitter histogram + counters into an ABI-format struct. */

void db_rt_get_jitter(struct db_rt_s *rt,
                      struct drivebase_jitter_dump_s *out);

/* Zero the histogram + counters.  Useful between bench scenarios. */

void db_rt_reset_jitter(struct db_rt_s *rt);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_RT_H */
