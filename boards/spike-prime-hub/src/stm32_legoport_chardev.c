/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport_chardev.c
 *
 * Character device shim for the I/O port DCM (Issue #42).  Registers
 * /dev/legoport0../dev/legoport5 backed by the kernel-side state in
 * stm32_legoport.c.  Single-open per port (mirrors /dev/ttyBT semantics).
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include <arch/board/board_legoport.h>

#include "spike_prime_hub.h"

#ifdef CONFIG_LEGO_LUMP_DIAG
#  include "stm32_legoport_uart_hw.h"
#endif

#ifdef CONFIG_LEGO_LUMP
#  include <arch/board/board_lump.h>
#endif

#ifdef CONFIG_LEGO_PORT

#define LEGOPORT_MAX_POLLWAITERS  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct legoport_chardev_s
{
  uint8_t            port;             /* 0..5 */
  bool               open;
  uint32_t           last_seen_event;  /* per-fd snapshot for WAIT_* */
  mutex_t            lock;             /* serializes open/close vs. poll */
  FAR struct pollfd *fds[LEGOPORT_MAX_POLLWAITERS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct legoport_chardev_s g_legoport_cdev[BOARD_LEGOPORT_COUNT];

/****************************************************************************
 * Forward Declarations from stm32_legoport.c
 ****************************************************************************/

int      stm32_legoport_get_info(int port, FAR struct legoport_info_s *out);
int      stm32_legoport_wait_change(int port, uint32_t snapshot,
                                    uint32_t timeout_ms,
                                    FAR uint32_t *new_counter_out);
uint32_t stm32_legoport_get_max_step_us(void);
uint32_t stm32_legoport_get_max_interval_us(void);
void     stm32_legoport_reset_stats(void);
void     stm32_legoport_get_stats(FAR struct legoport_stats_s *out);

/****************************************************************************
 * File Operations
 ****************************************************************************/

static int     legoport_cdev_open(FAR struct file *filep);
static int     legoport_cdev_close(FAR struct file *filep);
static ssize_t legoport_cdev_read(FAR struct file *filep, FAR char *buffer,
                                  size_t buflen);
static ssize_t legoport_cdev_write(FAR struct file *filep,
                                   FAR const char *buffer, size_t buflen);
static int     legoport_cdev_ioctl(FAR struct file *filep, int cmd,
                                   unsigned long arg);
static int     legoport_cdev_poll(FAR struct file *filep,
                                  FAR struct pollfd *fds, bool setup);

static const struct file_operations g_legoport_cdev_fops =
{
  legoport_cdev_open,
  legoport_cdev_close,
  legoport_cdev_read,
  legoport_cdev_write,
  NULL,                /* seek */
  legoport_cdev_ioctl,
  NULL,                /* mmap */
  NULL,                /* truncate */
  legoport_cdev_poll,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int legoport_cdev_open(FAR struct file *filep)
{
  FAR struct legoport_chardev_s *priv = filep->f_inode->i_private;
  int ret;

  ret = nxmutex_lock(&priv->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (priv->open)
    {
      nxmutex_unlock(&priv->lock);
      return -EBUSY;
    }

  priv->open            = true;
  priv->last_seen_event = 0;

  nxmutex_unlock(&priv->lock);
  return OK;
}

static int legoport_cdev_close(FAR struct file *filep)
{
  FAR struct legoport_chardev_s *priv = filep->f_inode->i_private;

  nxmutex_lock(&priv->lock);
  priv->open = false;
  nxmutex_unlock(&priv->lock);
  return OK;
}

static ssize_t legoport_cdev_read(FAR struct file *filep, FAR char *buffer,
                                  size_t buflen)
{
  UNUSED(filep);
  UNUSED(buffer);
  UNUSED(buflen);

  /* No streaming for #42; sensor / motor data is the job of #44/#45. */
  return -EAGAIN;
}

static ssize_t legoport_cdev_write(FAR struct file *filep,
                                   FAR const char *buffer, size_t buflen)
{
  UNUSED(filep);
  UNUSED(buffer);
  UNUSED(buflen);

  return -ENOTSUP;
}

static int legoport_cdev_ioctl(FAR struct file *filep, int cmd,
                               unsigned long arg)
{
  FAR struct legoport_chardev_s *priv = filep->f_inode->i_private;
  int ret;

  switch (cmd)
    {
      case LEGOPORT_GET_DEVICE_TYPE:
        {
          FAR uint8_t *user = (FAR uint8_t *)arg;
          struct legoport_info_s info;

          if (user == NULL)
            {
              return -EINVAL;
            }

          ret = stm32_legoport_get_info(priv->port, &info);
          if (ret < 0)
            {
              return ret;
            }

          /* memcpy through a kernel-stack copy is sufficient bounds
           * hardening for #42 under BUILD_PROTECTED — Issue #41 covers
           * the broader user-pointer sweep across all ioctl handlers.
           */
          memcpy(user, &info.device_type, sizeof(uint8_t));
          return OK;
        }

      case LEGOPORT_GET_DEVICE_INFO:
        {
          FAR struct legoport_info_s *user =
              (FAR struct legoport_info_s *)arg;
          struct legoport_info_s info;

          if (user == NULL)
            {
              return -EINVAL;
            }

          ret = stm32_legoport_get_info(priv->port, &info);
          if (ret < 0)
            {
              return ret;
            }

          memcpy(user, &info, sizeof(info));

          /* Refresh the per-fd "last seen" so a subsequent WAIT_*
           * blocks until the next edge.
           */
          priv->last_seen_event = info.event_counter;
          return OK;
        }

      case LEGOPORT_WAIT_CONNECT:
      case LEGOPORT_WAIT_DISCONNECT:
        {
          uint32_t timeout_ms = (uint32_t)arg;
          uint32_t snapshot   = priv->last_seen_event;
          struct legoport_info_s info;
          uint8_t target_connected =
              (cmd == LEGOPORT_WAIT_CONNECT) ? 1 : 0;

          /* Re-check before blocking — if the target state already
           * holds since the last snapshot, return immediately.
           */
          ret = stm32_legoport_get_info(priv->port, &info);
          if (ret < 0)
            {
              return ret;
            }

          uint8_t connected =
              ((info.flags & LEGOPORT_FLAG_CONNECTED) != 0) ? 1 : 0;

          if (info.event_counter != snapshot && connected == target_connected)
            {
              priv->last_seen_event = info.event_counter;
              return OK;
            }

          /* Block on edges until the target state matches. */

          for (; ; )
            {
              uint32_t new_counter;
              ret = stm32_legoport_wait_change(priv->port, snapshot,
                                               timeout_ms, &new_counter);
              if (ret < 0)
                {
                  return ret;
                }

              snapshot = new_counter;
              ret = stm32_legoport_get_info(priv->port, &info);
              if (ret < 0)
                {
                  return ret;
                }

              connected =
                  ((info.flags & LEGOPORT_FLAG_CONNECTED) != 0) ? 1 : 0;
              if (connected == target_connected)
                {
                  priv->last_seen_event = info.event_counter;
                  return OK;
                }
            }
        }

      case LEGOPORT_GET_STATS:
        {
          FAR struct legoport_stats_s *user =
              (FAR struct legoport_stats_s *)arg;
          struct legoport_stats_s stats;

          if (user == NULL)
            {
              return -EINVAL;
            }

          stm32_legoport_get_stats(&stats);
          memcpy(user, &stats, sizeof(stats));
          return OK;
        }

      case LEGOPORT_RESET_STATS:
        stm32_legoport_reset_stats();
        return OK;

#ifdef CONFIG_LEGO_LUMP_DIAG
      case LEGOPORT_LUMP_HW_DUMP:
        lump_uart_hw_dump();
        return OK;
#endif

#ifdef CONFIG_LEGO_LUMP
      case LEGOPORT_LUMP_GET_INFO:
        {
          FAR struct lump_device_info_s *user =
              (FAR struct lump_device_info_s *)arg;
          struct lump_device_info_s info;

          if (user == NULL)
            {
              return -EINVAL;
            }

          int rc = lump_get_info(priv->port, &info);
          if (rc < 0)
            {
              return rc;
            }
          memcpy(user, &info, sizeof(info));
          return OK;
        }
#endif

      default:
        return -ENOTTY;
    }
}

static int legoport_cdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                              bool setup)
{
  FAR struct legoport_chardev_s *priv = filep->f_inode->i_private;
  int ret = OK;
  int i;

  nxmutex_lock(&priv->lock);

  if (setup)
    {
      for (i = 0; i < LEGOPORT_MAX_POLLWAITERS; i++)
        {
          if (priv->fds[i] == NULL)
            {
              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= LEGOPORT_MAX_POLLWAITERS)
        {
          ret = -EBUSY;
          goto out;
        }

      /* Report POLLIN if a state change is already pending. */

      if ((fds->events & POLLIN) != 0)
        {
          struct legoport_info_s info;
          if (stm32_legoport_get_info(priv->port, &info) == OK &&
              info.event_counter != priv->last_seen_event)
            {
              poll_notify(&fds, 1, POLLIN);
            }
        }
    }
  else
    {
      if (fds->priv != NULL)
        {
          FAR struct pollfd **slot = fds->priv;
          *slot     = NULL;
          fds->priv = NULL;
        }
    }

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_legoport_chardev_register(void)
{
  char devpath[24];
  int  ret;
  int  p;

  for (p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      FAR struct legoport_chardev_s *priv = &g_legoport_cdev[p];

      memset(priv, 0, sizeof(*priv));
      priv->port = (uint8_t)p;
      nxmutex_init(&priv->lock);

      snprintf(devpath, sizeof(devpath), BOARD_LEGOPORT_DEVPATH_FMT, p);

      ret = register_driver(devpath, &g_legoport_cdev_fops, 0666, priv);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

#endif /* CONFIG_LEGO_PORT */
