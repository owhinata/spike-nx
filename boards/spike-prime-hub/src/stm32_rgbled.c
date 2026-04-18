/****************************************************************************
 * boards/spike-prime-hub/src/stm32_rgbled.c
 *
 * Char device /dev/rgbled0 that exposes the TLC5955 48-channel PWM LED
 * driver to user space via ioctl.  Required because CONFIG_BUILD_PROTECTED=y
 * prevents user apps from calling tlc5955_set_duty() directly.
 *
 * The driver is a thin wrapper: ioctl handlers validate arguments and
 * forward to the existing kernel-internal TLC5955 API, which still
 * batches per-channel updates onto HPWORK.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdint.h>

#include <nuttx/fs/fs.h>
#include <nuttx/fs/ioctl.h>

#include <arch/board/board_rgbled.h>

#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_SPI1

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int rgbled_open(FAR struct file *filep)
{
  return OK;
}

static int rgbled_close(FAR struct file *filep)
{
  return OK;
}

static int rgbled_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  switch (cmd)
    {
      case RGBLEDIOC_SETDUTY:
        {
          FAR struct rgbled_duty_s *d =
              (FAR struct rgbled_duty_s *)((uintptr_t)arg);
          if (d == NULL)
            {
              return -EINVAL;
            }

          if (d->channel >= TLC5955_NUM_CHANNELS)
            {
              return -EINVAL;
            }

          tlc5955_set_duty(d->channel, d->value);
          return OK;
        }

      case RGBLEDIOC_SETALL:
        {
          uint16_t value = (uint16_t)arg;
          uint8_t ch;

          for (ch = 0; ch < TLC5955_NUM_CHANNELS; ch++)
            {
              tlc5955_set_duty(ch, value);
            }

          return OK;
        }

      case RGBLEDIOC_UPDATE:
        {
          return tlc5955_update_sync();
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_rgbled_fops =
{
  .open  = rgbled_open,
  .close = rgbled_close,
  .read  = NULL,
  .write = NULL,
  .seek  = NULL,
  .ioctl = rgbled_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_rgbled_register(void)
{
  int ret = register_driver("/dev/rgbled0", &g_rgbled_fops, 0666, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "rgbled: register_driver failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "rgbled: /dev/rgbled0 registered\n");
  return OK;
}

#endif /* CONFIG_STM32_SPI1 */
