/****************************************************************************
 * boards/spike-prime-hub/src/stm32_btcap_chardev.c
 *
 * Kernel-side chardev for /dev/btcap (Issue #122).  Pipe-style synchronous
 * channel that links a single userspace writer (the apps/capture library
 * called from apps/sensor) to a single userspace reader (the btsensor
 * MODE CAPTURE handler).  The channel never buffers a whole capture
 * session: a small ring (BTCAP_RING_BYTES, default 1024 B = 4 BT chunks)
 * sits between the two sides so the writer blocks naturally whenever the
 * reader is slower than the producer or absent altogether.
 *
 * State machine (externally visible: IDLE / READY / ABORTED):
 *
 *   IDLE  --REGISTER--> READY (writer_done=false)
 *   READY --FINALIZE--> READY (writer_done=true)
 *   READY --drain done after FINALIZE--> IDLE
 *   READY --writer fd close before FINALIZE--> ABORTED
 *   READY --ABORT_SESSION--> ABORTED
 *   ABORTED --all fds closed--> IDLE
 *
 * Cleanup contract:
 *   - All cancelable wait paths (write/read/QUERY_STATE-poll) honour an
 *     internal `shutdown` flag posted by the release fop; waiters wake
 *     and observe the new state, returning -ECANCELED.
 *   - SIGKILL is handled implicitly: the NuttX signal/cancel machinery
 *     pulls the dying TCB off the sem wait list (`nxsem_recover`), then
 *     `fdlist_free()` -> `file_close()` invokes our release fop, which
 *     transitions the session to ABORTED in the same mutex-protected
 *     critical section that wakes the peer.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/board/board_btcap.h>

#include "board_usercheck.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_BOARD_BTCAP_CHARDEV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_BOARD_BTCAP_RING_BYTES
#  define CONFIG_BOARD_BTCAP_RING_BYTES 1024
#endif

#define BTCAP_RING_BYTES         CONFIG_BOARD_BTCAP_RING_BYTES
#define BTCAP_MAX_POLLWAITERS    2

/* Internal-only state values.  External callers see only IDLE / READY /
 * ABORTED through QUERY_STATE; READY covers both pre- and post-FINALIZE
 * and is differentiated via `writer_done` for the FINALIZE precondition.
 */

#define BTCAP_S_IDLE             0
#define BTCAP_S_READY            1
#define BTCAP_S_ABORTED          2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct btcap_chardev_s
{
  /* Lock covers state, ring indices, writer_done, shutdown, and all
   * `poll_notify` calls.  Sem waits drop the lock before sleeping and
   * re-acquire it after waking (cf. btcap_wait_writable_locked).
   */

  mutex_t        lock;

  /* Ring buffer.  head writes (producer), tail reads (consumer).
   * Empty: head == tail, full: ((head + 1) % size) == tail.
   */

  uint8_t        ring[BTCAP_RING_BYTES];
  size_t         head;
  size_t         tail;

  /* Wakeup primitives.  writers wait for ring space, readers wait for
   * data or for state changes.  Both are posted by every state change
   * so a session-end / abort wakes the peer regardless of which side is
   * blocked.
   */

  sem_t          writable_sem;
  sem_t          readable_sem;

  /* Session state.  `state` is the externally visible value; transitions
   * always happen under `lock` so external observers see consistent
   * snapshots.  `writer_done` flips inside READY when FINALIZE_SESSION
   * is accepted.  `session_generation` bumps on every IDLE->READY
   * transition so lingering fds from a torn-down session can detect
   * staleness.
   */

  int            state;
  bool           writer_done;
  bool           writer_open;       /* true while a writer fd is held    */
  bool           reader_open;       /* true while a reader fd is held    */
  bool           shutdown;          /* set in release fop to drain waits */
  uint32_t       session_generation;

  /* Session metadata recorded at REGISTER_SESSION time and surfaced to
   * the reader through GET_SESSION_META.
   */

  struct btcap_session_meta_s meta;

  /* Poll waiters.  Index 0 is the writer slot, 1 is the reader slot.
   * The chardev is single-instance + single open per direction so two
   * slots suffice.
   */

  FAR struct pollfd *fds[BTCAP_MAX_POLLWAITERS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct btcap_chardev_s g_btcap;

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

static int     btcap_open(FAR struct file *filep);
static int     btcap_close(FAR struct file *filep);
static ssize_t btcap_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen);
static ssize_t btcap_write(FAR struct file *filep,
                           FAR const char *buffer, size_t buflen);
static int     btcap_ioctl(FAR struct file *filep, int cmd,
                           unsigned long arg);
static int     btcap_poll(FAR struct file *filep,
                          FAR struct pollfd *fds, bool setup);

static const struct file_operations g_btcap_fops =
{
  btcap_open,
  btcap_close,
  btcap_read,
  btcap_write,
  NULL,                 /* seek */
  btcap_ioctl,
  NULL,                 /* mmap */
  NULL,                 /* truncate */
  btcap_poll,
};

/****************************************************************************
 * Private Functions: state-machine helpers (must be called under lock)
 ****************************************************************************/

static size_t btcap_ring_used_locked(FAR struct btcap_chardev_s *priv)
{
  if (priv->head >= priv->tail)
    {
      return priv->head - priv->tail;
    }

  return BTCAP_RING_BYTES - priv->tail + priv->head;
}

static size_t btcap_ring_free_locked(FAR struct btcap_chardev_s *priv)
{
  /* One byte is reserved to distinguish full from empty. */

  return BTCAP_RING_BYTES - 1u - btcap_ring_used_locked(priv);
}

/* Wake every waiter and notify pollers.  Called whenever state, ring
 * occupancy, or the shutdown flag changes; over-posting is harmless
 * since waiters always re-check the predicate after wakeup.
 */

static void btcap_notify_all_locked(FAR struct btcap_chardev_s *priv,
                                    pollevent_t pollset)
{
  int sval;
  if (nxsem_get_value(&priv->writable_sem, &sval) == 0 && sval <= 0)
    {
      nxsem_post(&priv->writable_sem);
    }

  if (nxsem_get_value(&priv->readable_sem, &sval) == 0 && sval <= 0)
    {
      nxsem_post(&priv->readable_sem);
    }

  if (pollset != 0)
    {
      poll_notify(priv->fds, BTCAP_MAX_POLLWAITERS, pollset);
    }
}

static void btcap_reset_locked(FAR struct btcap_chardev_s *priv)
{
  priv->state             = BTCAP_S_IDLE;
  priv->writer_done       = false;
  priv->head              = 0;
  priv->tail              = 0;
  memset(&priv->meta, 0, sizeof(priv->meta));
}

/****************************************************************************
 * Private Functions: file_operations
 ****************************************************************************/

static int btcap_open(FAR struct file *filep)
{
  FAR struct btcap_chardev_s *priv = filep->f_inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Direction is implied by O_RDONLY / O_WRONLY: writers (apps/capture)
   * open O_WRONLY, readers (btsensor MODE CAPTURE) open O_RDONLY.  Read
   * + write on the same fd is rejected: there is no use case and it
   * would muddle the ABORTED semantics.
   */

  bool want_write = (filep->f_oflags & O_WROK) != 0;
  bool want_read  = (filep->f_oflags & O_RDOK) != 0;

  if (want_write && want_read)
    {
      nxmutex_unlock(&priv->lock);
      return -EINVAL;
    }

  if (want_write)
    {
      if (priv->writer_open)
        {
          nxmutex_unlock(&priv->lock);
          return -EBUSY;
        }

      priv->writer_open = true;
    }
  else
    {
      if (priv->reader_open)
        {
          nxmutex_unlock(&priv->lock);
          return -EBUSY;
        }

      priv->reader_open = true;
    }

  nxmutex_unlock(&priv->lock);
  return OK;
}

static int btcap_close(FAR struct file *filep)
{
  FAR struct btcap_chardev_s *priv = filep->f_inode->i_private;
  bool want_write = (filep->f_oflags & O_WROK) != 0;
  pollevent_t pollset = 0;

  nxmutex_lock(&priv->lock);

  if (want_write)
    {
      priv->writer_open = false;

      /* Writer release behaviour:
       *   - state READY + writer_done==false  -> writer abandoned the
       *     session mid-stream.  Tear it down: ABORTED + wake the
       *     reader so it can post BTAB.
       *   - state READY + writer_done==true   -> normal exit.  If the
       *     reader is still attached and draining, leave the session
       *     alone (the reader will transition us to IDLE when the ring
       *     empties).  If no reader is attached, reclaim immediately:
       *     the user will never come for the data, so park the chardev
       *     back at IDLE.
       *   - state ABORTED                     -> already torn down by
       *     the peer; just stay there until the reader fd also closes,
       *     at which point we land in IDLE.
       */

      if (priv->state == BTCAP_S_READY && !priv->writer_done)
        {
          priv->state = BTCAP_S_ABORTED;
          pollset |= POLLERR | POLLHUP;
        }
      else if (priv->state == BTCAP_S_READY && priv->writer_done &&
               !priv->reader_open)
        {
          /* No reader will ever come.  Reset directly to IDLE; nothing
           * to flush since the reader is absent.
           */

          btcap_reset_locked(priv);
        }
    }
  else
    {
      priv->reader_open = false;

      /* Reader release.  Three sub-cases:
       *   1. writer_done && (any) → reader has finished draining and is
       *      handing the session back; transition to IDLE so the
       *      writer's QUERY_STATE polling can observe completion and
       *      close its end of the chardev.
       *   2. !writer_done && !writer_open → writer disappeared mid-
       *      stream and reader is gone too; reclaim immediately.
       *   3. !writer_done && writer_open → reader bailed early on a
       *      live writer; force ABORTED so the writer's next write()
       *      returns -ECANCELED instead of hanging on backpressure.
       */

      if (priv->state == BTCAP_S_ABORTED && !priv->writer_open)
        {
          btcap_reset_locked(priv);
        }
      else if (priv->state == BTCAP_S_READY && priv->writer_done)
        {
          btcap_reset_locked(priv);
        }
      else if (priv->state == BTCAP_S_READY && !priv->writer_open)
        {
          btcap_reset_locked(priv);
        }
      else if (priv->state == BTCAP_S_READY)
        {
          priv->state = BTCAP_S_ABORTED;
          pollset |= POLLERR | POLLHUP;
        }
    }

  /* If both sides are now closed, force the chardev back to IDLE so a
   * subsequent REGISTER_SESSION sees a clean slate even after an abort.
   */

  if (!priv->writer_open && !priv->reader_open &&
      priv->state == BTCAP_S_ABORTED)
    {
      btcap_reset_locked(priv);
    }

  /* Wake every waiter (they will re-check predicates and exit). */

  priv->shutdown = false;   /* shutdown flag is per-wait, not per-session */
  btcap_notify_all_locked(priv, pollset);

  nxmutex_unlock(&priv->lock);
  return OK;
}

/* Wait for the ring to have space, the session to abort, or the chardev
 * to be shut down.  Drops `priv->lock` before sleeping.  On return the
 * lock is held again.  Returns 0 if there is space and the session is
 * still READY+!writer_done, -ECANCELED on abort/shutdown, or -EINTR if
 * the wait was interrupted by a signal.
 */

static int btcap_wait_writable_locked(FAR struct btcap_chardev_s *priv,
                                      uint32_t expect_generation)
{
  while (priv->state == BTCAP_S_READY && !priv->writer_done &&
         priv->session_generation == expect_generation &&
         btcap_ring_free_locked(priv) == 0)
    {
      nxmutex_unlock(&priv->lock);
      int ret = nxsem_wait(&priv->writable_sem);
      nxmutex_lock(&priv->lock);

      if (ret == -EINTR)
        {
          return -EINTR;
        }

      if (priv->session_generation != expect_generation ||
          priv->state == BTCAP_S_ABORTED)
        {
          return -ECANCELED;
        }
    }

  if (priv->state != BTCAP_S_READY || priv->writer_done ||
      priv->session_generation != expect_generation)
    {
      return -ECANCELED;
    }

  return OK;
}

static int btcap_wait_readable_locked(FAR struct btcap_chardev_s *priv,
                                      uint32_t expect_generation)
{
  while (priv->state == BTCAP_S_READY &&
         priv->session_generation == expect_generation &&
         btcap_ring_used_locked(priv) == 0 && !priv->writer_done)
    {
      nxmutex_unlock(&priv->lock);
      int ret = nxsem_wait(&priv->readable_sem);
      nxmutex_lock(&priv->lock);

      if (ret == -EINTR)
        {
          return -EINTR;
        }

      if (priv->session_generation != expect_generation ||
          priv->state == BTCAP_S_ABORTED)
        {
          return -ECANCELED;
        }
    }

  if (priv->session_generation != expect_generation)
    {
      return -ECANCELED;
    }

  return OK;
}

static ssize_t btcap_write(FAR struct file *filep,
                           FAR const char *buffer, size_t buflen)
{
  FAR struct btcap_chardev_s *priv = filep->f_inode->i_private;

  if (buflen == 0)
    {
      return 0;
    }

  if (!board_user_in_ok(buffer, buflen))
    {
      return -EFAULT;
    }

  nxmutex_lock(&priv->lock);

  if (priv->state != BTCAP_S_READY || priv->writer_done)
    {
      nxmutex_unlock(&priv->lock);
      return priv->state == BTCAP_S_ABORTED ? -ECANCELED : -EINVAL;
    }

  uint32_t generation = priv->session_generation;
  size_t   written    = 0;

  while (written < buflen)
    {
      int ret;
      if (btcap_ring_free_locked(priv) == 0)
        {
          if ((filep->f_oflags & O_NONBLOCK) != 0)
            {
              break;        /* return whatever we managed; or -EAGAIN
                             * if we have not made any progress yet  */
            }

          ret = btcap_wait_writable_locked(priv, generation);
          if (ret < 0)
            {
              nxmutex_unlock(&priv->lock);
              return written > 0 ? (ssize_t)written : ret;
            }
        }

      size_t free_now = btcap_ring_free_locked(priv);
      size_t want     = buflen - written;
      size_t chunk    = want < free_now ? want : free_now;

      /* Copy in two segments if the ring wraps in the middle. */

      size_t first = BTCAP_RING_BYTES - priv->head;
      if (first > chunk)
        {
          first = chunk;
        }

      memcpy(&priv->ring[priv->head], buffer + written, first);
      if (chunk > first)
        {
          memcpy(&priv->ring[0], buffer + written + first, chunk - first);
        }

      priv->head = (priv->head + chunk) % BTCAP_RING_BYTES;
      written   += chunk;

      btcap_notify_all_locked(priv, POLLIN);
    }

  nxmutex_unlock(&priv->lock);

  if (written == 0)
    {
      /* Reached only when O_NONBLOCK + ring full; the wait helper above
       * never sets `written == 0` after a successful wait.
       */

      return -EAGAIN;
    }

  return (ssize_t)written;
}

static ssize_t btcap_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  FAR struct btcap_chardev_s *priv = filep->f_inode->i_private;

  if (buflen == 0)
    {
      return 0;
    }

  if (!board_user_out_ok(buffer, buflen))
    {
      return -EFAULT;
    }

  nxmutex_lock(&priv->lock);

  if (priv->state == BTCAP_S_IDLE || priv->state == BTCAP_S_ABORTED)
    {
      /* IDLE: either no session has been registered yet, or the last
       * session has already been fully drained.  Either way return EOF
       * (0) so a `cat /dev/btcap` style reader exits cleanly instead of
       * spinning on -ENODATA.  ABORTED also returns 0 (EOF) — the
       * caller already saw whatever truncated payload landed in the
       * ring before the abort.
       */

      nxmutex_unlock(&priv->lock);
      return 0;
    }

  uint32_t generation = priv->session_generation;
  size_t   nread      = 0;

  /* If the writer has finalised and the ring is empty, return EOF (0).
   * Do NOT reset to IDLE here — the reader is still attached and may
   * read a few more zero-byte returns from this fd before it closes
   * (cf. cat / `read until EOF` semantics).  The IDLE transition fires
   * in the release fop when the reader drops the fd; that is what the
   * writer-side QUERY_STATE polling waits on.
   */

  if (priv->writer_done && btcap_ring_used_locked(priv) == 0)
    {
      nxmutex_unlock(&priv->lock);
      return 0;
    }

  while (nread < buflen)
    {
      int ret;
      if (btcap_ring_used_locked(priv) == 0)
        {
          if (priv->writer_done)
            {
              break;        /* EOF: leave session-finalize cleanup for
                             * the next call so we can return the bytes
                             * we already drained.                     */
            }

          if ((filep->f_oflags & O_NONBLOCK) != 0)
            {
              break;
            }

          ret = btcap_wait_readable_locked(priv, generation);
          if (ret < 0)
            {
              nxmutex_unlock(&priv->lock);
              return nread > 0 ? (ssize_t)nread : ret;
            }
        }

      size_t used_now = btcap_ring_used_locked(priv);
      size_t want     = buflen - nread;
      size_t chunk    = want < used_now ? want : used_now;

      size_t first = BTCAP_RING_BYTES - priv->tail;
      if (first > chunk)
        {
          first = chunk;
        }

      memcpy(buffer + nread, &priv->ring[priv->tail], first);
      if (chunk > first)
        {
          memcpy(buffer + nread + first, &priv->ring[0], chunk - first);
        }

      priv->tail = (priv->tail + chunk) % BTCAP_RING_BYTES;
      nread     += chunk;

      btcap_notify_all_locked(priv, POLLOUT);
    }

  /* If the writer has finalised and the ring just emptied, leave the
   * session in READY (writer_done==true, used==0) and notify POLLIN so
   * the reader's next poll wakes for an EOF read.  The IDLE
   * transition is reserved for the release fop when the reader drops
   * its fd; doing it here would cause the next poll() to see
   * state==IDLE with no POLLIN attached and the reader would never
   * fire on_read again, which is exactly the bug that masked Test 2's
   * stale-session forwarding.
   */

  if (priv->writer_done && btcap_ring_used_locked(priv) == 0)
    {
      btcap_notify_all_locked(priv, POLLIN | POLLHUP);
    }

  nxmutex_unlock(&priv->lock);

  if (nread == 0 && (filep->f_oflags & O_NONBLOCK) != 0)
    {
      return -EAGAIN;
    }

  return (ssize_t)nread;
}

static int btcap_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct btcap_chardev_s *priv = filep->f_inode->i_private;
  int ret = -ENOTTY;

  nxmutex_lock(&priv->lock);

  switch (cmd)
    {
      case BTCAPIOC_REGISTER_SESSION:
        {
          FAR struct btcap_session_meta_s *meta =
            (FAR struct btcap_session_meta_s *)arg;

          if (!board_user_in_ok(meta, sizeof(*meta)))
            {
              ret = -EFAULT;
              break;
            }

          if (priv->state != BTCAP_S_IDLE)
            {
              ret = -EBUSY;
              break;
            }

          if ((filep->f_oflags & O_WROK) == 0)
            {
              ret = -EACCES;
              break;
            }

          memcpy(&priv->meta, meta, sizeof(priv->meta));
          priv->meta.name[BTCAP_NAME_MAX - 1] = '\0';

          priv->state             = BTCAP_S_READY;
          priv->writer_done       = false;
          priv->head              = 0;
          priv->tail              = 0;
          priv->session_generation++;

          btcap_notify_all_locked(priv, POLLIN);
          ret = OK;
        }
        break;

      case BTCAPIOC_FINALIZE_SESSION:
        {
          if ((filep->f_oflags & O_WROK) == 0)
            {
              ret = -EACCES;
              break;
            }

          if (priv->state != BTCAP_S_READY || priv->writer_done)
            {
              ret = -EINVAL;
              break;
            }

          priv->writer_done = true;

          /* If there is no reader and the ring is empty there is
           * nothing for anyone to consume, so collapse straight to
           * IDLE.  Otherwise let the reader drain on its own schedule.
           */

          if (!priv->reader_open && btcap_ring_used_locked(priv) == 0)
            {
              btcap_reset_locked(priv);
            }

          btcap_notify_all_locked(priv, POLLIN | POLLHUP);
          ret = OK;
        }
        break;

      case BTCAPIOC_ABORT_SESSION:
        {
          if (priv->state != BTCAP_S_READY)
            {
              ret = OK;     /* idempotent: ABORT in IDLE / ABORTED is OK */
              break;
            }

          priv->state = BTCAP_S_ABORTED;
          btcap_notify_all_locked(priv, POLLERR | POLLHUP);
          ret = OK;
        }
        break;

      case BTCAPIOC_GET_SESSION_META:
        {
          FAR struct btcap_session_meta_s *meta =
            (FAR struct btcap_session_meta_s *)arg;

          if (!board_user_out_ok(meta, sizeof(*meta)))
            {
              ret = -EFAULT;
              break;
            }

          if (priv->state != BTCAP_S_READY)
            {
              ret = -ENOENT;
              break;
            }

          memcpy(meta, &priv->meta, sizeof(*meta));
          ret = OK;
        }
        break;

      case BTCAPIOC_QUERY_STATE:
        {
          FAR int32_t *out = (FAR int32_t *)arg;
          if (!board_user_out_ok(out, sizeof(*out)))
            {
              ret = -EFAULT;
              break;
            }

          *out = (int32_t)priv->state;
          ret  = OK;
        }
        break;

      default:
        ret = -ENOTTY;
        break;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

static int btcap_poll(FAR struct file *filep,
                      FAR struct pollfd *fds, bool setup)
{
  FAR struct btcap_chardev_s *priv = filep->f_inode->i_private;
  bool want_write = (filep->f_oflags & O_WROK) != 0;
  int ret = OK;

  nxmutex_lock(&priv->lock);

  if (setup)
    {
      int slot = want_write ? 0 : 1;
      if (priv->fds[slot] != NULL)
        {
          ret = -EBUSY;
          goto done;
        }

      priv->fds[slot] = fds;
      fds->priv       = &priv->fds[slot];

      pollevent_t evt = 0;
      if (want_write)
        {
          if (priv->state == BTCAP_S_READY && !priv->writer_done &&
              btcap_ring_free_locked(priv) > 0)
            {
              evt |= POLLOUT;
            }
        }
      else
        {
          if (priv->state == BTCAP_S_READY &&
              (btcap_ring_used_locked(priv) > 0 || priv->writer_done))
            {
              evt |= POLLIN;
            }
        }

      if (priv->state == BTCAP_S_ABORTED)
        {
          evt |= POLLERR | POLLHUP;
        }

      if (evt != 0)
        {
          poll_notify(&priv->fds[slot], 1, evt);
        }
    }
  else
    {
      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      if (slot != NULL)
        {
          *slot     = NULL;
          fds->priv = NULL;
        }
    }

done:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_btcap_chardev_register(void)
{
  FAR struct btcap_chardev_s *priv = &g_btcap;

  nxmutex_init(&priv->lock);
  nxsem_init(&priv->writable_sem, 0, 0);
  nxsem_init(&priv->readable_sem, 0, 0);

  btcap_reset_locked(priv);

  return register_driver(BTCAP_DEVPATH, &g_btcap_fops, 0666, priv);
}

#endif /* CONFIG_BOARD_BTCAP_CHARDEV */
