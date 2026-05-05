/****************************************************************************
 * boards/spike-prime-hub/src/stm32_btuart_chardev.c
 *
 * Expose the CC2564C HCI UART as a character device (/dev/ttyBT) so a
 * user-mode Bluetooth host stack (btstack, under apps/btsensor/, Issue #52)
 * can drive the link through standard read/write/poll/ioctl.
 *
 * The chardev is a thin shim over the board-local btuart_lowerhalf_s
 * produced by stm32_btuart_instantiate().  Single open/close only: the BT
 * stack is expected to own the device end-to-end.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include <arch/board/board_btuart.h>

#include "board_usercheck.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_USART2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BTUART_MAX_POLLWAITERS  2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct btuart_chardev_s
{
  FAR struct btuart_lowerhalf_s *lower;
  mutex_t                        lock;
  sem_t                          rxsem;       /* Posted when RX data lands */
  bool                           open;
  FAR struct pollfd             *fds[BTUART_MAX_POLLWAITERS];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct btuart_chardev_s g_btuart_cdev;

/****************************************************************************
 * Forward declarations
 ****************************************************************************/

static int     btuart_cdev_open(FAR struct file *filep);
static int     btuart_cdev_close(FAR struct file *filep);
static ssize_t btuart_cdev_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen);
static ssize_t btuart_cdev_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen);
static int     btuart_cdev_ioctl(FAR struct file *filep, int cmd,
                                 unsigned long arg);
static int     btuart_cdev_poll(FAR struct file *filep,
                                FAR struct pollfd *fds, bool setup);

static const struct file_operations g_btuart_cdev_fops =
{
  btuart_cdev_open,
  btuart_cdev_close,
  btuart_cdev_read,
  btuart_cdev_write,
  NULL,                 /* seek */
  btuart_cdev_ioctl,
  NULL,                 /* mmap */
  NULL,                 /* truncate */
  btuart_cdev_poll,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* RX arrival callback: wake blocking readers and any poll waiters.  The
 * lower-half latches rxwork_pending across successive bursts; btuart_read
 * clears the latch once the ring drains empty, which re-arms the IDLE
 * ISR to fire this callback again for the next incoming packet.
 */

static void btuart_cdev_rxnotify(FAR const struct btuart_lowerhalf_s *lower,
                                 FAR void *arg)
{
  FAR struct btuart_chardev_s *priv = (FAR struct btuart_chardev_s *)arg;

  UNUSED(lower);

  int sval;
  if (nxsem_get_value(&priv->rxsem, &sval) == 0 && sval <= 0)
    {
      nxsem_post(&priv->rxsem);
    }

  poll_notify(priv->fds, BTUART_MAX_POLLWAITERS, POLLIN);
}

static int btuart_cdev_open(FAR struct file *filep)
{
  FAR struct btuart_chardev_s *priv = filep->f_inode->i_private;
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

  if (priv->lower == NULL ||
      priv->lower->rxattach == NULL ||
      priv->lower->rxenable == NULL)
    {
      nxmutex_unlock(&priv->lock);
      return -ENODEV;
    }

  priv->lower->rxattach(priv->lower, btuart_cdev_rxnotify, priv);
  priv->lower->rxenable(priv->lower, true);
  priv->open = true;

  nxmutex_unlock(&priv->lock);
  return OK;
}

static int btuart_cdev_close(FAR struct file *filep)
{
  FAR struct btuart_chardev_s *priv = filep->f_inode->i_private;

  nxmutex_lock(&priv->lock);
  if (priv->open && priv->lower != NULL)
    {
      priv->lower->rxenable(priv->lower, false);
      priv->lower->rxattach(priv->lower, NULL, NULL);
    }

  priv->open = false;
  nxmutex_unlock(&priv->lock);
  return OK;
}

static ssize_t btuart_cdev_read(FAR struct file *filep, FAR char *buffer,
                                size_t buflen)
{
  FAR struct btuart_chardev_s *priv = filep->f_inode->i_private;
  ssize_t n;

  if (buflen == 0)
    {
      return 0;
    }

  if (!board_user_out_ok(buffer, buflen))
    {
      return -EFAULT;
    }

  /* One-shot loop: try a read; if empty and blocking, wait for rxsem. */

  for (; ; )
    {
      n = priv->lower->read(priv->lower, (FAR uint8_t *)buffer, buflen);
      if (n != 0)
        {
          return n;
        }

      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          return -EAGAIN;
        }

      int ret = nxsem_wait(&priv->rxsem);
      if (ret < 0)
        {
          return ret;
        }
    }
}

static ssize_t btuart_cdev_write(FAR struct file *filep,
                                 FAR const char *buffer, size_t buflen)
{
  FAR struct btuart_chardev_s *priv = filep->f_inode->i_private;

  if (buflen == 0)
    {
      return 0;
    }

  if (!board_user_in_ok(buffer, buflen))
    {
      return -EFAULT;
    }

  return priv->lower->write(priv->lower, (FAR const uint8_t *)buffer,
                            buflen);
}

static int btuart_cdev_ioctl(FAR struct file *filep, int cmd,
                             unsigned long arg)
{
  FAR struct btuart_chardev_s *priv = filep->f_inode->i_private;

  switch (cmd)
    {
      case BTUART_IOC_SETBAUD:
        if (priv->lower == NULL || priv->lower->setbaud == NULL)
          {
            return -ENOSYS;
          }

        return priv->lower->setbaud(priv->lower, (uint32_t)arg);

      case BTUART_IOC_CHIPRESET:
        return stm32_bluetooth_chip_reset();

      default:
        return -ENOTTY;
    }
}

static int btuart_cdev_poll(FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup)
{
  FAR struct btuart_chardev_s *priv = filep->f_inode->i_private;
  int ret = OK;
  int i;

  nxmutex_lock(&priv->lock);

  if (setup)
    {
      for (i = 0; i < BTUART_MAX_POLLWAITERS; i++)
        {
          if (priv->fds[i] == NULL)
            {
              priv->fds[i] = fds;
              fds->priv    = &priv->fds[i];
              break;
            }
        }

      if (i >= BTUART_MAX_POLLWAITERS)
        {
          ret = -EBUSY;
          goto out;
        }

      /* Report already-satisfied events right away:
       *   POLLIN  - non-destructive peek of the RX ring.
       *   POLLOUT - the lower-half's write() is blocking-DMA based so the
       *             chardev is always writable as far as poll() clients
       *             (e.g. btstack_uart_nuttx) are concerned.
       */

      pollevent_t ready = 0;
      if ((fds->events & POLLIN) != 0 &&
          stm32_btuart_rx_available(priv->lower) > 0)
        {
          ready |= POLLIN;
        }

      if ((fds->events & POLLOUT) != 0)
        {
          ready |= POLLOUT;
        }

      if (ready != 0)
        {
          poll_notify(&fds, 1, ready);
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

int stm32_btuart_chardev_register(FAR struct btuart_lowerhalf_s *lower)
{
  FAR struct btuart_chardev_s *priv = &g_btuart_cdev;

  if (lower == NULL)
    {
      return -EINVAL;
    }

  memset(priv, 0, sizeof(*priv));
  priv->lower = lower;
  nxmutex_init(&priv->lock);
  nxsem_init(&priv->rxsem, 0, 0);

  return register_driver(BOARD_BTUART_DEVPATH, &g_btuart_cdev_fops, 0666,
                         priv);
}

#endif /* CONFIG_STM32_USART2 */
