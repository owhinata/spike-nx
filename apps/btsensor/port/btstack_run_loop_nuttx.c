/****************************************************************************
 * apps/btsensor/port/btstack_run_loop_nuttx.c
 *
 * NuttX-native btstack run loop (Issue #52 Step C).  Single-threaded, using
 * poll(2) for fd-based data sources and clock_gettime(CLOCK_MONOTONIC) for
 * timers.  No pipe/eventfd is required because the btsensor app runs every
 * btstack entry point from a single task; cross-thread notifications simply
 * set a volatile flag which is inspected at the top of each iteration.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <time.h>
#include <unistd.h>

#include "btstack_run_loop.h"
#include "btstack_run_loop_base.h"
#include "btstack_linked_list.h"
#include "btstack_util.h"

#include "btstack_run_loop_nuttx.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* poll(2) is our per-iteration wait primitive.  Hard upper bound on the
 * number of fds the run loop will watch.  Sized for the post-Commit-C
 * btsensor fd set:
 *   1. /dev/ttyBT  (HCI transport via btstack_uart_nuttx)
 *   2. /dev/uorb/sensor_imu0  (IMU sampler data source)
 *   3. /dev/buttons  (BT button)
 *   4. self-pipe wake (cross-thread execute_on_main_thread)
 * with two slots of headroom for follow-up work (Commit D RFCOMM rx
 * notifications etc.).
 */

#define NUTTX_RUN_LOOP_MAX_FDS     8

/* Maximum time poll() is allowed to sleep without any timer or fd event.
 * With the /dev/ttyBT chardev deferring poll_notify() to HPWORK (mirroring
 * NuttX's own drivers/wireless/bluetooth/bt_uart.c pattern), POLLIN reliably
 * wakes us from poll() within a few microseconds of the RX IDLE IRQ, so
 * this clamp is only an idle-time safety net (e.g. a future background
 * thread posting execute_on_main_thread with no other event pending).
 */

#define NUTTX_RUN_LOOP_MAX_WAIT_MS 1000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool            g_exit_requested;
static volatile bool   g_trigger_event;
static struct timespec g_start_ts;

/* Cross-thread callbacks land on btstack_run_loop_base_callbacks via
 * execute_on_main_thread().  The btstack-provided list itself is not
 * thread-safe; serialise enqueue/dequeue with this mutex so an NSH
 * context posting a teardown_kick cannot tear the list while the run
 * loop is iterating it.  Held only across the linked-list operation —
 * the user callback runs unlocked so it may itself post further
 * cross-thread work without deadlocking.
 */

static pthread_mutex_t g_callback_lock = PTHREAD_MUTEX_INITIALIZER;

/* Self-pipe wake (Issue #56 Commit C).  execute_on_main_thread() writes
 * a single byte to g_wake_wfd, which the run loop's poll() picks up as
 * POLLIN on g_wake_rfd and drains.  This drops the cross-thread wake
 * latency from up to NUTTX_RUN_LOOP_MAX_WAIT_MS (1 s) to a few hundred
 * microseconds — essential for responsive BT button handling and short
 * teardown latency.  g_wake_lock guards open/close vs. write() to
 * prevent NSH context from writing to a stale fd during teardown.
 */

static pthread_mutex_t       g_wake_lock = PTHREAD_MUTEX_INITIALIZER;
static int                   g_wake_rfd  = -1;
static int                   g_wake_wfd  = -1;
static btstack_data_source_t g_wake_ds;

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static uint32_t timespec_ms_since(const struct timespec *start,
                                  const struct timespec *now)
{
  uint64_t sec = (uint64_t)(now->tv_sec - start->tv_sec);
  int64_t  nsec_diff = (int64_t)now->tv_nsec - (int64_t)start->tv_nsec;

  if (nsec_diff < 0)
    {
      sec     -= 1;
      nsec_diff += 1000000000;
    }

  return (uint32_t)(sec * 1000ULL + (uint64_t)nsec_diff / 1000000ULL);
}

static uint32_t run_loop_nuttx_get_time_ms(void)
{
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return timespec_ms_since(&g_start_ts, &now);
}

/****************************************************************************
 * btstack_run_loop_t callbacks
 ****************************************************************************/

static void wake_process(btstack_data_source_t *ds,
                         btstack_data_source_callback_type_t type)
{
  (void)ds;
  (void)type;
  char drain[16];
  while (read(g_wake_rfd, drain, sizeof(drain)) > 0)
    {
      /* Just drain — the wake is the signal; the actual work was queued
       * on btstack_run_loop_base_callbacks before the byte was written.
       */
    }
}

static void run_loop_nuttx_init(void)
{
  btstack_run_loop_base_init();
  clock_gettime(CLOCK_MONOTONIC, &g_start_ts);
  g_exit_requested = false;
  g_trigger_event  = false;

  /* Recreate the self-pipe on every init() so a stop/start cycle
   * starts with fresh fds.  Old fds are closed defensively in case a
   * previous run left them open.
   */

  pthread_mutex_lock(&g_wake_lock);
  if (g_wake_rfd >= 0)
    {
      close(g_wake_rfd);
      g_wake_rfd = -1;
    }

  if (g_wake_wfd >= 0)
    {
      close(g_wake_wfd);
      g_wake_wfd = -1;
    }

  int p[2];
  if (pipe(p) == 0)
    {
      fcntl(p[0], F_SETFL, O_NONBLOCK);
      fcntl(p[1], F_SETFL, O_NONBLOCK);
      g_wake_rfd = p[0];
      g_wake_wfd = p[1];
    }
  else
    {
      syslog(LOG_ERR, "btstack_run_loop_nuttx: pipe() failed errno=%d\n",
             errno);
    }

  pthread_mutex_unlock(&g_wake_lock);

  if (g_wake_rfd >= 0)
    {
      btstack_run_loop_set_data_source_fd(&g_wake_ds, g_wake_rfd);
      btstack_run_loop_set_data_source_handler(&g_wake_ds, wake_process);
      btstack_run_loop_add_data_source(&g_wake_ds);
      btstack_run_loop_enable_data_source_callbacks(
          &g_wake_ds, DATA_SOURCE_CALLBACK_READ);
    }
}

static void run_loop_nuttx_set_timer(btstack_timer_source_t *ts,
                                     uint32_t timeout_in_ms)
{
  ts->timeout = run_loop_nuttx_get_time_ms() + timeout_in_ms + 1;
}

static void run_loop_nuttx_execute(void)
{
  struct pollfd pfds[NUTTX_RUN_LOOP_MAX_FDS];
  btstack_data_source_t *ds_map[NUTTX_RUN_LOOP_MAX_FDS];

  while (!g_exit_requested)
    {
      /* Collect active data sources into pollfd array. */

      int nfds = 0;
      btstack_linked_list_iterator_t it;
      btstack_linked_list_iterator_init(&it,
                                        &btstack_run_loop_base_data_sources);
      while (btstack_linked_list_iterator_has_next(&it) &&
             nfds < NUTTX_RUN_LOOP_MAX_FDS)
        {
          btstack_data_source_t *ds =
              (btstack_data_source_t *)btstack_linked_list_iterator_next(&it);
          if (ds->source.fd < 0)
            {
              continue;
            }

          short events = 0;
          if (ds->flags & DATA_SOURCE_CALLBACK_READ)
            {
              events |= POLLIN;
            }

          if (ds->flags & DATA_SOURCE_CALLBACK_WRITE)
            {
              events |= POLLOUT;
            }

          if (events == 0)
            {
              continue;
            }

          pfds[nfds].fd      = ds->source.fd;
          pfds[nfds].events  = events;
          pfds[nfds].revents = 0;
          ds_map[nfds]       = ds;
          nfds++;
        }

      /* Next timer deadline, clamped so external triggers can be observed. */

      uint32_t now_ms = run_loop_nuttx_get_time_ms();
      int32_t delta_ms = btstack_run_loop_base_get_time_until_timeout(now_ms);
      int timeout_ms;
      if (delta_ms < 0)
        {
          timeout_ms = NUTTX_RUN_LOOP_MAX_WAIT_MS;
        }
      else if (delta_ms > NUTTX_RUN_LOOP_MAX_WAIT_MS)
        {
          timeout_ms = NUTTX_RUN_LOOP_MAX_WAIT_MS;
        }
      else
        {
          timeout_ms = (int)delta_ms;
        }

      if (g_trigger_event)
        {
          timeout_ms = 0;
        }

      int ret = poll(pfds, nfds, timeout_ms);
      if (ret < 0 && errno != EINTR)
        {
          break;
        }

      if (ret > 0)
        {
          for (int i = 0; i < nfds; i++)
            {
              btstack_data_source_t *ds = ds_map[i];
              if ((pfds[i].revents & POLLIN) != 0 &&
                  (ds->flags & DATA_SOURCE_CALLBACK_READ) != 0)
                {
                  ds->process(ds, DATA_SOURCE_CALLBACK_READ);
                }

              if ((pfds[i].revents & POLLOUT) != 0 &&
                  (ds->flags & DATA_SOURCE_CALLBACK_WRITE) != 0)
                {
                  ds->process(ds, DATA_SOURCE_CALLBACK_WRITE);
                }
            }
        }

      /* Serve trigger requests from other threads (there are none yet in
       * Step C but the hook is required by btstack).
       */

      if (g_trigger_event)
        {
          g_trigger_event = false;
          btstack_run_loop_base_poll_data_sources();
        }

      /* Expired timers. */

      btstack_run_loop_base_process_timers(run_loop_nuttx_get_time_ms());

      /* Cross-thread callbacks queued via execute_on_main_thread.
       * Skip the mutex acquire entirely on the common (empty) path —
       * a racy NULL check is safe here because a concurrent enqueue
       * on another thread also writes to the wake fd (see
       * run_loop_nuttx_execute_on_main_thread), so the next poll()
       * iteration is guaranteed to see the new entry.
       */

      while (btstack_run_loop_base_callbacks != NULL)
        {
          pthread_mutex_lock(&g_callback_lock);
          btstack_context_callback_registration_t *cbr =
              (btstack_context_callback_registration_t *)
                  btstack_linked_list_pop(&btstack_run_loop_base_callbacks);
          pthread_mutex_unlock(&g_callback_lock);

          if (cbr == NULL)
            {
              break;
            }

          (*cbr->callback)(cbr->context);
        }
    }

  /* Run loop exited — release the self-pipe.  Held under g_wake_lock
   * to keep a concurrent execute_on_main_thread() from writing to a
   * stale fd; that caller will see g_wake_wfd < 0 and skip the write.
   */

  btstack_run_loop_disable_data_source_callbacks(
      &g_wake_ds, DATA_SOURCE_CALLBACK_READ);
  btstack_run_loop_remove_data_source(&g_wake_ds);

  pthread_mutex_lock(&g_wake_lock);
  if (g_wake_rfd >= 0)
    {
      close(g_wake_rfd);
      g_wake_rfd = -1;
    }

  if (g_wake_wfd >= 0)
    {
      close(g_wake_wfd);
      g_wake_wfd = -1;
    }

  pthread_mutex_unlock(&g_wake_lock);
}

static void run_loop_nuttx_poll_data_sources_from_irq(void)
{
  g_trigger_event = true;
}

static void run_loop_nuttx_execute_on_main_thread(
    btstack_context_callback_registration_t *callback_registration)
{
  pthread_mutex_lock(&g_callback_lock);
  btstack_run_loop_base_add_callback(callback_registration);
  pthread_mutex_unlock(&g_callback_lock);

  /* Wake the run loop via self-pipe.  EAGAIN means the pipe already
   * has at least one queued byte, which is plenty — the run loop only
   * needs to know there's work pending.
   */

  pthread_mutex_lock(&g_wake_lock);
  if (g_wake_wfd >= 0)
    {
      char x = 'x';
      (void)write(g_wake_wfd, &x, 1);
    }

  pthread_mutex_unlock(&g_wake_lock);
}

static void run_loop_nuttx_trigger_exit(void)
{
  g_exit_requested = true;
}

/****************************************************************************
 * Public API
 ****************************************************************************/

const btstack_run_loop_t *btstack_run_loop_nuttx_get_instance(void)
{
  static const btstack_run_loop_t instance =
  {
    &run_loop_nuttx_init,
    &btstack_run_loop_base_add_data_source,
    &btstack_run_loop_base_remove_data_source,
    &btstack_run_loop_base_enable_data_source_callbacks,
    &btstack_run_loop_base_disable_data_source_callbacks,
    &run_loop_nuttx_set_timer,
    &btstack_run_loop_base_add_timer,
    &btstack_run_loop_base_remove_timer,
    &run_loop_nuttx_execute,
    &btstack_run_loop_base_dump_timer,
    &run_loop_nuttx_get_time_ms,
    &run_loop_nuttx_poll_data_sources_from_irq,
    &run_loop_nuttx_execute_on_main_thread,
    &run_loop_nuttx_trigger_exit,
  };

  return &instance;
}
