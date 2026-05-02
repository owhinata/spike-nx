/****************************************************************************
 * apps/drivebase/drivebase_rt.c
 *
 * 5 ms RT control task.  See drivebase_rt.h for the API contract.
 *
 * Sched / timing rules (Codex review of Issue #77 plan):
 *   - SCHED_FIFO so the kernel does not preempt us round-robin against
 *     other RT/100-priority tasks.
 *   - clock_nanosleep with TIMER_ABSTIME ⇒ drift-free deadlines (each
 *     deadline is computed from t0, not from the previous sleep
 *     return).
 *   - Jitter is sampled from the post-wake clock_gettime read, so a
 *     missed deadline is captured as a positive `lag` regardless of
 *     why the wake came late (preemption, IRQ stall, etc).
 *   - The histogram + counters are protected by a stats_lock that the
 *     reader path takes briefly; the RT side takes it for the few µs
 *     of `bucket++ / max-update`.  Codex-approved tradeoff: a brief
 *     pthread mutex inside the RT path is acceptable as long as no
 *     blocking I/O sits behind it.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <pthread.h>
#include <sched.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>

#include "drivebase_rt.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define US_PER_S       (1000000ULL)
#define NS_PER_US      (1000ULL)
#define NS_PER_S       (US_PER_S * NS_PER_US)

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static uint64_t ts_to_us(const struct timespec *ts)
{
  return (uint64_t)ts->tv_sec * US_PER_S +
         (uint64_t)ts->tv_nsec / NS_PER_US;
}

static void ts_add_ns(struct timespec *ts, uint64_t ns)
{
  ts->tv_nsec += (long)(ns % NS_PER_S);
  ts->tv_sec  += (time_t)(ns / NS_PER_S);
  if (ts->tv_nsec >= (long)NS_PER_S)
    {
      ts->tv_nsec -= (long)NS_PER_S;
      ts->tv_sec  += 1;
    }
}

static void record_jitter(struct db_rt_s *rt, int64_t lag_us)
{
  if (lag_us < 0) lag_us = 0;
  uint32_t lag = (uint32_t)lag_us;

  uint32_t bucket;
  if      (lag <    50) bucket = 0;
  else if (lag <   100) bucket = 1;
  else if (lag <   200) bucket = 2;
  else if (lag <   500) bucket = 3;
  else if (lag <  1000) bucket = 4;
  else if (lag <  2000) bucket = 5;
  else if (lag <  5000) bucket = 6;
  else                  bucket = 7;

  pthread_mutex_lock(&rt->stats_lock);
  rt->tick_count++;
  rt->jitter_hist[bucket]++;
  if (lag > rt->max_lag_us) rt->max_lag_us = lag;
  if (lag >= DB_RT_DEADLINE_US) rt->deadline_miss_count++;
  pthread_mutex_unlock(&rt->stats_lock);
}

/****************************************************************************
 * Private Functions — RT thread body
 ****************************************************************************/

static void *db_rt_thread(void *arg)
{
  struct db_rt_s *rt = (struct db_rt_s *)arg;

  /* Set our own SCHED_FIFO priority — pthread_create's attr-based
   * priority isn't reliably honoured under BUILD_PROTECTED so we
   * raise here instead.  pthread_setschedparam returns 0 on success
   * or a positive errno (not negated).
   */

  /* SCHED_FIFO promotion now happens via pthread_attr_setschedparam
   * in db_rt_start (which set both POSIX_SPAWN_SETSCHEDULER and
   * POSIX_SPAWN_SETSCHEDPARAM).  Calling pthread_setschedparam from
   * the thread itself accumulates kernel scheduling state across
   * create/join cycles and crashes the device on the 3rd cycle
   * (Issue #96).
   */

  struct timespec next;
  if (clock_gettime(CLOCK_MONOTONIC, &next) != 0)
    {
      rt->last_cb_error = -errno;
      return NULL;
    }

  while (atomic_load(&rt->running))
    {
      ts_add_ns(&next, DB_RT_TICK_US * NS_PER_US);

      /* Sleep until the next deadline; EINTR is benign — clamp and  */
      /* continue.  Other failures abort the loop.                    */

      int sr;
      do
        {
          sr = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next,
                               NULL);
        }
      while (sr == EINTR);
      if (sr != 0)
        {
          rt->last_cb_error = -sr;
          break;
        }

      struct timespec now;
      clock_gettime(CLOCK_MONOTONIC, &now);
      uint64_t now_us  = ts_to_us(&now);
      uint64_t want_us = ts_to_us(&next);
      int64_t  lag_us  = (int64_t)now_us - (int64_t)want_us;

      record_jitter(rt, lag_us);

      if (rt->tick_cb != NULL)
        {
          int cr = rt->tick_cb(now_us, rt->tick_cb_arg);
          if (cr < 0)
            {
              rt->last_cb_error = cr;
              break;
            }
        }
    }

  atomic_store(&rt->running, false);
  return NULL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_rt_init(struct db_rt_s *rt)
{
  memset(rt, 0, sizeof(*rt));
  atomic_store(&rt->running, false);
  pthread_mutex_init(&rt->stats_lock, NULL);
}

int db_rt_start(struct db_rt_s *rt, int priority,
                db_rt_tick_cb_t cb, void *arg)
{
  if (rt->started)
    {
      return -EALREADY;
    }

  rt->priority      = priority;
  rt->tick_cb       = cb;
  rt->tick_cb_arg   = arg;
  rt->last_cb_error = 0;
  atomic_store(&rt->running, true);

  /* Explicit 4 KB stack — CONFIG_PTHREAD_STACK_DEFAULT is 2 KB on
   * this build, which is not enough for the RT loop's dispatch path
   * (encoder drain + observer + PID + motor ioctl + state publish
   * have nested call frames easily reaching 2-3 KB).
   */

  pthread_attr_t attr;
  pthread_attr_init(&attr);
  pthread_attr_setstacksize(&attr, 4096);

  /* Promote to SCHED_FIFO at the requested priority via attr — calling
   * pthread_setschedparam from the thread itself accumulates internal
   * scheduling state and crashes the device after a few create/join
   * cycles (Issue #96 bisect).
   */

  struct sched_param sp = { .sched_priority = rt->priority };
  pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  pthread_attr_setschedparam(&attr, &sp);
  pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

  int rc = pthread_create(&rt->thread, &attr, db_rt_thread, rt);
  pthread_attr_destroy(&attr);
  if (rc != 0)
    {
      atomic_store(&rt->running, false);
      return -rc;
    }
  rt->started = true;
  return 0;
}

void db_rt_stop(struct db_rt_s *rt, uint32_t timeout_ms)
{
  if (!rt->started)
    {
      return;
    }

  atomic_store(&rt->running, false);

  /* Cooperative join — the thread checks `running` between sleeps so
   * worst-case wait is one tick (5 ms).  NuttX does not implement
   * pthread_timedjoin_np, but the bound is small enough that we can
   * trust pthread_join.
   */

  (void)timeout_ms;
  pthread_join(rt->thread, NULL);
  rt->started = false;
}

void db_rt_get_jitter(struct db_rt_s *rt,
                      struct drivebase_jitter_dump_s *out)
{
  pthread_mutex_lock(&rt->stats_lock);
  out->total_ticks         = rt->tick_count;
  out->max_lag_us          = rt->max_lag_us;
  out->deadline_miss_count = rt->deadline_miss_count;
  for (uint32_t i = 0; i < DRIVEBASE_JITTER_BUCKETS; i++)
    {
      out->hist_us[i] = rt->jitter_hist[i];
    }
  pthread_mutex_unlock(&rt->stats_lock);
}

void db_rt_reset_jitter(struct db_rt_s *rt)
{
  pthread_mutex_lock(&rt->stats_lock);
  rt->tick_count          = 0;
  rt->max_lag_us          = 0;
  rt->deadline_miss_count = 0;
  for (uint32_t i = 0; i < DRIVEBASE_JITTER_BUCKETS; i++)
    {
      rt->jitter_hist[i] = 0;
    }
  pthread_mutex_unlock(&rt->stats_lock);
}
