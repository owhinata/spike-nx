/****************************************************************************
 * boards/spike-prime-hub/src/stm32_drivebase_chardev.c
 *
 * Kernel-side thin shim for /dev/drivebase.  Registered from board
 * bringup so the device node always exists; the userspace drivebase
 * daemon (apps/drivebase/, Issue #77) lazily DRIVEBASE_DAEMON_ATTACHes
 * itself once it has CLAIM-locked the two motor uORB topics.
 *
 * IPC structure (NuttX side; the Linux/FUSE port replaces this entire
 * shim with libfuse callbacks that call the apps/drivebase/ chardev
 * handler directly):
 *
 *   - cmd_ring : MPSC (multiple user fds push, daemon pops).  The push
 *                path holds producer_lock for ~µs.  STOP envelopes are
 *                coalesced at the tail and bypass the ring-full check
 *                (oldest non-STOP is dropped) so a STOP storm cannot be
 *                shed.  Non-STOP push under a full ring returns -EBUSY
 *                so the caller is never blocked behind the ring (avoids
 *                priority inversion).
 *
 *   - state_db : double buffer.  Daemon writes the inactive slot and
 *                atomically swaps `state_active`; user readers memcpy
 *                from the active slot.  Single writer = no race.
 *
 *   - status   : seqlock.  Daemon writes under odd seq, reader retries
 *                until seq is even and unchanged across the memcpy.
 *
 *   - emergency_stop : DRIVEBASE_STOP latches output_epoch and calls
 *                stm32_legoport_pwm_coast/brake() directly using the
 *                port indices the daemon registered at ATTACH time.  No
 *                user function pointer is ever stored in the kernel —
 *                a daemon segfault cannot leave a dangling callback.
 *
 *   - stale-daemon watchdog : an LPWORK item polls last_publish_ts.
 *                A 50 ms gap (10 ticks of the 5 ms control loop) triggers
 *                an unconditional motor coast and a forced detach.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wqueue.h>

#include <arch/board/board_drivebase.h>

#include "spike_prime_hub.h"

#ifdef CONFIG_BOARD_DRIVEBASE_CHARDEV

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_CMD_RING_DEPTH         8
#define DB_STALE_THRESHOLD_MS     50
#define DB_WATCHDOG_PERIOD_MS     25
#define DB_PORT_IDX_INVALID       0xff

#define DB_CMD_RING_NEXT(i)       (((i) + 1u) % DB_CMD_RING_DEPTH)
#define DB_CMD_RING_PREV(i)       (((i) + DB_CMD_RING_DEPTH - 1u) % \
                                   DB_CMD_RING_DEPTH)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct db_chardev_s
{
  /* Command ring.  cmd_head is producer-only (under producer_lock),
   * cmd_tail is consumer-only (single daemon thread).  Empty ⇔ head == tail.
   */

  struct drivebase_cmd_envelope_s cmd_buf[DB_CMD_RING_DEPTH];
  uint32_t                        cmd_head;
  atomic_uint                     cmd_tail;
  mutex_t                         producer_lock;
  sem_t                           cmd_avail_sem;

  /* State double-buffer (single writer = daemon, multi reader = user) */

  struct drivebase_state_s        state_db[2];
  atomic_uint                     state_active;

  /* Status: seqlock.  Writers (daemon) bump status_seq twice per write. */

  struct drivebase_status_s       status;
  atomic_uint                     status_seq;

  /* Jitter dump: snapshot replaced wholesale on PUBLISH_JITTER, so a
   * single mutex suffices (slow-path only, not on the control loop's
   * critical path).
   */

  struct drivebase_jitter_dump_s  jitter;
  mutex_t                         jitter_lock;

  /* Daemon attach state.  attach_filep is the fd that issued ATTACH;
   * close() compares this pointer to detect daemon termination
   * (segfault / `task_delete` / clean exit) and triggers cleanup.
   */

  bool                            attached;
  FAR struct file                *attach_filep;
  uint8_t                         motor_l_port_idx;
  uint8_t                         motor_r_port_idx;
  uint8_t                         default_on_completion;
  clock_t                         last_publish_ticks;

  /* Atomics shared between fast-path ioctl and the daemon */

  atomic_uint                     output_epoch;
  atomic_uint                     cmd_seq;

  /* Watchdog */

  struct work_s                   watchdog;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct db_chardev_s g_db_cdev;

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

static int db_chardev_open(FAR struct file *filep);
static int db_chardev_close(FAR struct file *filep);
static int db_chardev_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);

static const struct file_operations g_db_cdev_fops =
{
  db_chardev_open,
  db_chardev_close,
  NULL,                 /* read    */
  NULL,                 /* write   */
  NULL,                 /* seek    */
  db_chardev_ioctl,
  NULL,                 /* mmap    */
  NULL,                 /* truncate */
  NULL,                 /* poll    */
};

static void db_watchdog_work(FAR void *arg);

/****************************************************************************
 * Private Functions — emergency stop
 ****************************************************************************/

/* Drive both motors into COAST (or BRAKE) using the port indices the
 * daemon registered at ATTACH time.  Safe to call from any context that
 * may take stm32_legoport_pwm_*'s internal mutex (i.e. not from a hard
 * IRQ).  Returns silently if no daemon is attached.
 */

static void db_emergency_actuate(FAR struct db_chardev_s *dev,
                                 uint8_t on_completion)
{
  if (dev->motor_l_port_idx != DB_PORT_IDX_INVALID)
    {
      if (on_completion == DRIVEBASE_ON_COMPLETION_BRAKE ||
          on_completion == DRIVEBASE_ON_COMPLETION_BRAKE_SMART ||
          on_completion == DRIVEBASE_ON_COMPLETION_HOLD)
        {
          stm32_legoport_pwm_brake(dev->motor_l_port_idx);
        }
      else
        {
          stm32_legoport_pwm_coast(dev->motor_l_port_idx);
        }
    }

  if (dev->motor_r_port_idx != DB_PORT_IDX_INVALID)
    {
      if (on_completion == DRIVEBASE_ON_COMPLETION_BRAKE ||
          on_completion == DRIVEBASE_ON_COMPLETION_BRAKE_SMART ||
          on_completion == DRIVEBASE_ON_COMPLETION_HOLD)
        {
          stm32_legoport_pwm_brake(dev->motor_r_port_idx);
        }
      else
        {
          stm32_legoport_pwm_coast(dev->motor_r_port_idx);
        }
    }
}

/****************************************************************************
 * Private Functions — cmd ring
 ****************************************************************************/

/* Producer side.  Returns 0 on success, -EBUSY when the ring is full
 * for non-STOP cmds (STOP always succeeds — coalesced or by dropping
 * the oldest non-STOP).
 */

static int db_push_envelope(FAR struct db_chardev_s *dev,
                            uint32_t cmd_kind, uint32_t epoch,
                            FAR const void *payload, size_t payload_len)
{
  bool is_stop = (cmd_kind == DRIVEBASE_STOP);
  int  ret;

  if (payload_len > DRIVEBASE_CMD_PAYLOAD_BYTES)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->producer_lock);
  if (ret < 0)
    {
      return ret;
    }

  uint32_t tail  = atomic_load(&dev->cmd_tail);
  uint32_t head  = dev->cmd_head;
  uint32_t next  = DB_CMD_RING_NEXT(head);
  bool     full  = (next == tail);

  if (is_stop)
    {
      /* STOP coalesce: if the most recently pushed envelope is also a
       * STOP, overwrite it in place with the latest payload.
       */

      if (head != tail)
        {
          uint32_t prev = DB_CMD_RING_PREV(head);
          if (dev->cmd_buf[prev].cmd_kind == DRIVEBASE_STOP)
            {
              dev->cmd_buf[prev].epoch    = epoch;
              dev->cmd_buf[prev].cmd_seq  =
                atomic_fetch_add(&dev->cmd_seq, 1) + 1;
              if (payload != NULL && payload_len > 0)
                {
                  memset(dev->cmd_buf[prev].payload, 0,
                         DRIVEBASE_CMD_PAYLOAD_BYTES);
                  memcpy(dev->cmd_buf[prev].payload, payload, payload_len);
                }
              dev->status.last_cmd_seq = dev->cmd_buf[prev].cmd_seq;
              nxmutex_unlock(&dev->producer_lock);
              return 0;
            }
        }

      /* Non-coalesced STOP into a full ring: drop the oldest non-STOP. */

      if (full)
        {
          uint32_t i;
          for (i = tail; i != head; i = DB_CMD_RING_NEXT(i))
            {
              if (dev->cmd_buf[i].cmd_kind != DRIVEBASE_STOP)
                {
                  /* Compact: shift everything after `i` down one slot. */
                  uint32_t j = i;
                  uint32_t k;
                  while ((k = DB_CMD_RING_NEXT(j)) != head)
                    {
                      dev->cmd_buf[j] = dev->cmd_buf[k];
                      j = k;
                    }
                  head = j;
                  next = DB_CMD_RING_NEXT(head);
                  dev->cmd_head = head;
                  dev->status.cmd_drop_count++;
                  full = false;
                  break;
                }
            }
        }
    }
  else if (full)
    {
      dev->status.cmd_drop_count++;
      nxmutex_unlock(&dev->producer_lock);
      return -EBUSY;
    }

  dev->cmd_buf[head].cmd_kind = cmd_kind;
  dev->cmd_buf[head].cmd_seq  = atomic_fetch_add(&dev->cmd_seq, 1) + 1;
  dev->cmd_buf[head].epoch    = epoch;
  dev->cmd_buf[head].reserved = 0;
  memset(dev->cmd_buf[head].payload, 0, DRIVEBASE_CMD_PAYLOAD_BYTES);
  if (payload != NULL && payload_len > 0)
    {
      memcpy(dev->cmd_buf[head].payload, payload, payload_len);
    }

  dev->cmd_head            = next;
  dev->status.last_cmd_seq = dev->cmd_buf[head].cmd_seq;

  /* Update occupancy counter for diagnostics. */

  uint32_t depth = (next + DB_CMD_RING_DEPTH - tail) % DB_CMD_RING_DEPTH;
  dev->status.cmd_ring_depth = depth;

  nxmutex_unlock(&dev->producer_lock);

  /* Wake the daemon if it is blocked in PICKUP_CMD (commit #9). */

  int sval = 0;
  if (nxsem_get_value(&dev->cmd_avail_sem, &sval) == 0 && sval <= 0)
    {
      nxsem_post(&dev->cmd_avail_sem);
    }

  return 0;
}

/* Consumer side (daemon only).  Pops one envelope; returns -EAGAIN when
 * the ring is empty.  Single consumer ⇒ no consumer-side lock needed.
 */

static int db_pop_envelope(FAR struct db_chardev_s *dev,
                           FAR struct drivebase_cmd_envelope_s *out)
{
  uint32_t tail = atomic_load(&dev->cmd_tail);
  uint32_t head;

  /* Re-read head under producer_lock for a consistent snapshot — the
   * producer touches both head and the slot we are about to read.
   */

  int ret = nxmutex_lock(&dev->producer_lock);
  if (ret < 0)
    {
      return ret;
    }
  head = dev->cmd_head;
  if (head == tail)
    {
      nxmutex_unlock(&dev->producer_lock);
      return -EAGAIN;
    }
  *out = dev->cmd_buf[tail];
  uint32_t next = DB_CMD_RING_NEXT(tail);
  atomic_store(&dev->cmd_tail, next);

  uint32_t depth = (head + DB_CMD_RING_DEPTH - next) % DB_CMD_RING_DEPTH;
  dev->status.cmd_ring_depth = depth;
  dev->status.last_pickup_us = (uint32_t)TICK2USEC(clock_systime_ticks());

  nxmutex_unlock(&dev->producer_lock);
  return 0;
}

/****************************************************************************
 * Private Functions — state / status publish
 ****************************************************************************/

static void db_publish_state(FAR struct db_chardev_s *dev,
                             FAR const struct drivebase_state_s *st)
{
  unsigned int active   = atomic_load(&dev->state_active);
  unsigned int inactive = active ^ 1u;
  dev->state_db[inactive] = *st;
  atomic_store(&dev->state_active, inactive);

  dev->last_publish_ticks    = clock_systime_ticks();
  dev->status.last_publish_us =
    (uint32_t)TICK2USEC(dev->last_publish_ticks);
}

static void db_read_state(FAR struct db_chardev_s *dev,
                          FAR struct drivebase_state_s *out)
{
  unsigned int active = atomic_load(&dev->state_active);
  *out = dev->state_db[active];
}

static void db_publish_status(FAR struct db_chardev_s *dev,
                              FAR const struct drivebase_status_s *src)
{
  /* Writer-side seqlock: bump to odd, copy, bump to even.  Status fields
   * the kernel maintains itself (cmd_ring_depth / cmd_drop_count /
   * last_cmd_seq / last_pickup_us / last_publish_us / attach_generation)
   * are preserved across the publish — the daemon never overwrites
   * them.
   */

  unsigned int seq = atomic_fetch_add(&dev->status_seq, 1) + 1;  /* odd  */
  (void)seq;

  dev->status.configured        = src->configured;
  dev->status.motor_l_bound     = src->motor_l_bound;
  dev->status.motor_r_bound     = src->motor_r_bound;
  dev->status.imu_present       = src->imu_present;
  dev->status.use_gyro          = src->use_gyro;
  dev->status.tick_count        = src->tick_count;
  dev->status.tick_overrun_count = src->tick_overrun_count;
  dev->status.tick_max_lag_us    = src->tick_max_lag_us;
  dev->status.encoder_drop_count = src->encoder_drop_count;

  atomic_fetch_add(&dev->status_seq, 1);                          /* even */
}

static void db_read_status(FAR struct db_chardev_s *dev,
                           FAR struct drivebase_status_s *out)
{
  unsigned int s1;
  unsigned int s2 = ~0u;

  do
    {
      s1 = atomic_load(&dev->status_seq);
      if ((s1 & 1u) != 0)
        {
          /* Writer in progress — spin briefly */
          continue;
        }
      *out = dev->status;
      s2   = atomic_load(&dev->status_seq);
    }
  while (s1 != s2);
}

/****************************************************************************
 * Private Functions — attach / detach
 ****************************************************************************/

static void db_disarm_watchdog(FAR struct db_chardev_s *dev)
{
  work_cancel(LPWORK, &dev->watchdog);
}

static void db_arm_watchdog(FAR struct db_chardev_s *dev)
{
  work_queue(LPWORK, &dev->watchdog, db_watchdog_work, dev,
             MSEC2TICK(DB_WATCHDOG_PERIOD_MS));
}

/* Internal cleanup shared between explicit DAEMON_DETACH and the close()
 * fop's "ATTACH fd was just closed" path.  Caller holds producer_lock.
 */

static void db_detach_locked(FAR struct db_chardev_s *dev,
                             bool emergency_stop)
{
  if (!dev->attached)
    {
      return;
    }

  if (emergency_stop)
    {
      db_emergency_actuate(dev, dev->default_on_completion);
    }

  dev->attached            = false;
  dev->attach_filep        = NULL;
  dev->motor_l_port_idx    = DB_PORT_IDX_INVALID;
  dev->motor_r_port_idx    = DB_PORT_IDX_INVALID;

  /* Clear status fields that describe a live daemon under the seqlock
   * so a detached chardev reads as a zeroed snapshot.  Daemon-published
   * fields (configured / motor_l_bound / motor_r_bound / imu_present /
   * use_gyro / tick_*) become meaningless once we detach, and the
   * publish/pickup timestamps stop advancing — clear them too so
   * monitoring tools see a clean transition.  Only attach_generation
   * (monotonic) and the cmd ring counters (queued user commands
   * survive across re-attach) are preserved.
   */

  (void)atomic_fetch_add(&dev->status_seq, 1);             /* odd  */
  dev->status.daemon_attached    = 0;
  dev->status.configured         = 0;
  dev->status.motor_l_bound      = 0;
  dev->status.motor_r_bound      = 0;
  dev->status.imu_present        = 0;
  dev->status.use_gyro           = 0;
  dev->status.tick_count         = 0;
  dev->status.tick_overrun_count = 0;
  dev->status.tick_max_lag_us    = 0;
  dev->status.encoder_drop_count = 0;
  dev->status.last_publish_us    = 0;
  dev->status.last_pickup_us     = 0;
  dev->status.attach_generation++;
  (void)atomic_fetch_add(&dev->status_seq, 1);             /* even */

  db_disarm_watchdog(dev);
}

/****************************************************************************
 * Private Functions — watchdog
 ****************************************************************************/

static void db_watchdog_work(FAR void *arg)
{
  FAR struct db_chardev_s *dev = arg;
  bool stale = false;

  nxmutex_lock(&dev->producer_lock);
  if (!dev->attached)
    {
      nxmutex_unlock(&dev->producer_lock);
      return;
    }

  clock_t now      = clock_systime_ticks();
  clock_t age_tick = now - dev->last_publish_ticks;
  uint32_t age_ms  = (uint32_t)TICK2MSEC(age_tick);

  if (age_ms >= DB_STALE_THRESHOLD_MS)
    {
      stale = true;
      db_detach_locked(dev, true);
    }
  nxmutex_unlock(&dev->producer_lock);

  if (!stale)
    {
      db_arm_watchdog(dev);
    }
}

/****************************************************************************
 * Private Functions — fops
 ****************************************************************************/

static int db_chardev_open(FAR struct file *filep)
{
  /* No exclusion: any number of user fds can poll status / state.  The
   * daemon is identified by ATTACH, not by which fd opened first.
   */

  UNUSED(filep);
  return OK;
}

static int db_chardev_close(FAR struct file *filep)
{
  FAR struct db_chardev_s *dev = filep->f_inode->i_private;

  nxmutex_lock(&dev->producer_lock);
  if (dev->attached && dev->attach_filep == filep)
    {
      /* The fd that ATTACHed us was just closed — daemon termination.
       * Force a motor coast and tear down attach state so the next
       * DAEMON_ATTACH can re-arm cleanly.
       */

      db_detach_locked(dev, true);
    }
  nxmutex_unlock(&dev->producer_lock);

  return OK;
}

static int db_handle_daemon_attach(FAR struct db_chardev_s *dev,
                                   FAR struct file *filep,
                                   FAR const struct drivebase_attach_s *att)
{
  if (att == NULL)
    {
      return -EINVAL;
    }
  if (att->motor_l_port_idx >= 6 || att->motor_r_port_idx >= 6)
    {
      return -EINVAL;
    }

  nxmutex_lock(&dev->producer_lock);
  if (dev->attached)
    {
      nxmutex_unlock(&dev->producer_lock);
      return -EBUSY;
    }

  dev->attached              = true;
  dev->attach_filep          = filep;
  dev->motor_l_port_idx      = att->motor_l_port_idx;
  dev->motor_r_port_idx      = att->motor_r_port_idx;
  dev->default_on_completion = att->default_on_completion;
  dev->last_publish_ticks    = clock_systime_ticks();
  dev->status.daemon_attached = 1;
  dev->status.attach_generation++;
  nxmutex_unlock(&dev->producer_lock);

  db_arm_watchdog(dev);
  return OK;
}

static int db_handle_user_drive(FAR struct db_chardev_s *dev,
                                int cmd, FAR const void *arg)
{
  size_t payload_len;

  if (!dev->attached)
    {
      return -ENOTCONN;
    }
  if (arg == NULL)
    {
      return -EINVAL;
    }

  switch (cmd)
    {
      case DRIVEBASE_CONFIG:
        payload_len = sizeof(struct drivebase_config_s);
        break;
      case DRIVEBASE_RESET:
        payload_len = sizeof(struct drivebase_reset_s);
        break;
      case DRIVEBASE_DRIVE_STRAIGHT:
        payload_len = sizeof(struct drivebase_drive_straight_s);
        break;
      case DRIVEBASE_DRIVE_CURVE:
        payload_len = sizeof(struct drivebase_drive_curve_s);
        break;
      case DRIVEBASE_DRIVE_ARC_ANGLE:
      case DRIVEBASE_DRIVE_ARC_DISTANCE:
        payload_len = sizeof(struct drivebase_drive_arc_s);
        break;
      case DRIVEBASE_DRIVE_FOREVER:
        payload_len = sizeof(struct drivebase_drive_forever_s);
        break;
      case DRIVEBASE_TURN:
        payload_len = sizeof(struct drivebase_turn_s);
        break;
      case DRIVEBASE_SPIKE_DRIVE_FOREVER:
        payload_len = sizeof(struct drivebase_spike_forever_s);
        break;
      case DRIVEBASE_SPIKE_DRIVE_TIME:
        payload_len = sizeof(struct drivebase_spike_time_s);
        break;
      case DRIVEBASE_SPIKE_DRIVE_ANGLE:
        payload_len = sizeof(struct drivebase_spike_angle_s);
        break;
      case DRIVEBASE_SET_DRIVE_SETTINGS:
        payload_len = sizeof(struct drivebase_drive_settings_s);
        break;
      case DRIVEBASE_SET_USE_GYRO:
        payload_len = sizeof(struct drivebase_set_use_gyro_s);
        break;
      default:
        return -ENOTTY;
    }

  uint32_t epoch = atomic_load(&dev->output_epoch);
  return db_push_envelope(dev, (uint32_t)cmd, epoch, arg, payload_len);
}

static int db_handle_stop(FAR struct db_chardev_s *dev,
                          FAR const struct drivebase_stop_s *stp)
{
  if (!dev->attached)
    {
      return -ENOTCONN;
    }
  if (stp == NULL)
    {
      return -EINVAL;
    }

  /* (i) bump output_epoch so any envelope still queued from before this
   *     STOP is recognised as superseded by the daemon, (ii) latch the
   *     new policy in default_on_completion so the LPWORK watchdog
   *     coasts using the right verb, (iii) coast/brake the H-bridge in
   *     this caller's context so the motors stop within ~30 µs without
   *     waiting for the daemon, (iv) push a STOP envelope so the daemon
   *     can sync trajectory state on its next pickup.
   */

  uint32_t new_epoch = atomic_fetch_add(&dev->output_epoch, 1) + 1;

  nxmutex_lock(&dev->producer_lock);
  dev->default_on_completion = stp->on_completion;
  db_emergency_actuate(dev, stp->on_completion);
  nxmutex_unlock(&dev->producer_lock);

  return db_push_envelope(dev, DRIVEBASE_STOP, new_epoch,
                          stp, sizeof(*stp));
}

static int db_chardev_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct db_chardev_s *dev = filep->f_inode->i_private;
  FAR void *argp = (FAR void *)(uintptr_t)arg;

  switch (cmd)
    {
      /* Daemon-internal */

      case DRIVEBASE_DAEMON_ATTACH:
        return db_handle_daemon_attach(dev, filep, argp);

      case DRIVEBASE_DAEMON_DETACH:
        nxmutex_lock(&dev->producer_lock);
        if (dev->attached && dev->attach_filep == filep)
          {
            db_detach_locked(dev, false);
            nxmutex_unlock(&dev->producer_lock);
            return OK;
          }
        nxmutex_unlock(&dev->producer_lock);
        return -ENOTCONN;

      case DRIVEBASE_DAEMON_PICKUP_CMD:
        if (argp == NULL)
          {
            return -EINVAL;
          }
        if (!dev->attached || dev->attach_filep != filep)
          {
            return -ENOTCONN;
          }
        return db_pop_envelope(dev, argp);

      case DRIVEBASE_DAEMON_PUBLISH_STATE:
        if (argp == NULL)
          {
            return -EINVAL;
          }
        if (!dev->attached || dev->attach_filep != filep)
          {
            return -ENOTCONN;
          }
        db_publish_state(dev, argp);
        return OK;

      case DRIVEBASE_DAEMON_PUBLISH_STATUS:
        if (argp == NULL)
          {
            return -EINVAL;
          }
        if (!dev->attached || dev->attach_filep != filep)
          {
            return -ENOTCONN;
          }
        db_publish_status(dev, argp);
        return OK;

      case DRIVEBASE_DAEMON_PUBLISH_JITTER:
        if (argp == NULL)
          {
            return -EINVAL;
          }
        if (!dev->attached || dev->attach_filep != filep)
          {
            return -ENOTCONN;
          }
        nxmutex_lock(&dev->jitter_lock);
        dev->jitter = *(FAR const struct drivebase_jitter_dump_s *)argp;
        nxmutex_unlock(&dev->jitter_lock);
        return OK;

      /* User-facing: STOP has its own fast path */

      case DRIVEBASE_STOP:
        return db_handle_stop(dev, argp);

      /* User-facing: state / status / jitter snapshot reads */

      case DRIVEBASE_GET_STATE:
        if (argp == NULL)
          {
            return -EINVAL;
          }
        if (!dev->attached)
          {
            return -ENOTCONN;
          }
        db_read_state(dev, argp);
        return OK;

      case DRIVEBASE_GET_STATUS:
        if (argp == NULL)
          {
            return -EINVAL;
          }
        db_read_status(dev, argp);   /* always available, even detached */
        return OK;

      case DRIVEBASE_JITTER_DUMP:
        if (argp == NULL)
          {
            return -EINVAL;
          }
        if (!dev->attached)
          {
            return -ENOTCONN;
          }
        nxmutex_lock(&dev->jitter_lock);
        *(FAR struct drivebase_jitter_dump_s *)argp = dev->jitter;
        nxmutex_unlock(&dev->jitter_lock);
        return OK;

      /* User-facing: drive verbs all funnel through cmd_ring */

      case DRIVEBASE_CONFIG:
      case DRIVEBASE_RESET:
      case DRIVEBASE_DRIVE_STRAIGHT:
      case DRIVEBASE_DRIVE_CURVE:
      case DRIVEBASE_DRIVE_ARC_ANGLE:
      case DRIVEBASE_DRIVE_ARC_DISTANCE:
      case DRIVEBASE_DRIVE_FOREVER:
      case DRIVEBASE_TURN:
      case DRIVEBASE_SPIKE_DRIVE_FOREVER:
      case DRIVEBASE_SPIKE_DRIVE_TIME:
      case DRIVEBASE_SPIKE_DRIVE_ANGLE:
      case DRIVEBASE_SET_DRIVE_SETTINGS:
      case DRIVEBASE_SET_USE_GYRO:
        return db_handle_user_drive(dev, cmd, argp);

      case DRIVEBASE_GET_DRIVE_SETTINGS:
      case DRIVEBASE_GET_HEADING:
        /* Read paths backed by the daemon publish.  These return NOTCONN
         * until the daemon has had at least one tick to publish; for
         * commit #2 we shorthand them to NOTCONN until the daemon-side
         * handler ships in commit #9.
         */
        return dev->attached ? -ENOSYS : -ENOTCONN;

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_drivebase_chardev_register(void)
{
  FAR struct db_chardev_s *dev = &g_db_cdev;

  memset(dev, 0, sizeof(*dev));
  nxmutex_init(&dev->producer_lock);
  nxmutex_init(&dev->jitter_lock);
  nxsem_init(&dev->cmd_avail_sem, 0, 0);

  dev->motor_l_port_idx = DB_PORT_IDX_INVALID;
  dev->motor_r_port_idx = DB_PORT_IDX_INVALID;

  return register_driver(DRIVEBASE_DEVPATH, &g_db_cdev_fops, 0666, dev);
}

#endif /* CONFIG_BOARD_DRIVEBASE_CHARDEV */
