/****************************************************************************
 * boards/spike-prime-hub/src/stm32_pcm.c
 *
 * Raw PCM char device /dev/pcm0 for SPIKE Prime Hub.
 *
 * write() accepts a single blob containing a struct pcm_write_hdr_s header
 * immediately followed by `sample_count` uint16_t samples.  Samples are
 * centered on 0x8000 (12-bit left-aligned DAC).  The header format is
 * pybricks-compatible in spirit: a single call carries data, length, and
 * sample rate so there is no order dependency between separate ioctls.
 *
 * Validation is performed inside g_sound.lock so that the race window
 * against concurrent close()/ioctl()/tone_write() is closed.  The current
 * build targets CONFIG_BUILD_FLAT; if CONFIG_BUILD_PROTECTED support is
 * added later, the memcpy below will need to use a copyin() helper.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>

#include <arch/board/board_sound.h>

#include "spike_prime_hub.h"
#include "stm32_sound.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SPIKE_SOUND_PCM_BUFSAMPLES
#  define CONFIG_SPIKE_SOUND_PCM_BUFSAMPLES 1024
#endif

#define PCM_BUF_SAMPLES CONFIG_SPIKE_SOUND_PCM_BUFSAMPLES

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA source buffer for /dev/pcm0.  Placed in .bss (main SRAM) so DMA1 can
 * access it; 16-bit alignment is guaranteed by uint16_t.
 */

static uint16_t g_pcm_buf[PCM_BUF_SAMPLES];

/****************************************************************************
 * Private: char device entry points
 ****************************************************************************/

static int pcm_open(FAR struct file *filep)
{
  nxmutex_lock(&g_sound.lock);
  g_sound.open_count++;
  nxmutex_unlock(&g_sound.lock);
  return OK;
}

static int pcm_close(FAR struct file *filep)
{
  nxmutex_lock(&g_sound.lock);
  g_sound.open_count--;

  if (g_sound.owner == filep)
    {
      atomic_store_explicit(&g_sound.stop_flag, true, memory_order_relaxed);
      stm32_sound_stop_pcm();
      g_sound.owner = NULL;
      g_sound.mode  = SOUND_MODE_IDLE;
    }
  else if (g_sound.open_count == 0)
    {
      stm32_sound_stop_pcm();
      g_sound.mode = SOUND_MODE_IDLE;
    }

  nxmutex_unlock(&g_sound.lock);
  return OK;
}

static ssize_t pcm_write(FAR struct file *filep,
                         FAR const char *ubuf, size_t nbytes)
{
  if (ubuf == NULL)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_sound.lock);

  /* 1. minimum header size */

  if (nbytes < sizeof(struct pcm_write_hdr_s))
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  FAR const struct pcm_write_hdr_s *h =
    (FAR const struct pcm_write_hdr_s *)ubuf;

  /* 2. forward-compatible version gate */

  if (h->version > PCM_WRITE_VERSION)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 3. magic */

  if (h->magic != PCM_WRITE_MAGIC)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 4. hdr_size / sample_rate range */

  if (h->hdr_size < sizeof(struct pcm_write_hdr_s) || h->hdr_size > nbytes)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  if (h->sample_rate < 1000 || h->sample_rate > 100000)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 5. overflow check on sample_count * 2 */

  if (h->sample_count > (UINT32_MAX / 2))
    {
      nxmutex_unlock(&g_sound.lock);
      return -EOVERFLOW;
    }

  size_t payload_bytes = (size_t)h->sample_count * sizeof(uint16_t);

  /* 6. total bytes consistency */

  if (payload_bytes != (nbytes - h->hdr_size))
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 7. kernel buffer capacity */

  if (payload_bytes > sizeof(g_pcm_buf))
    {
      nxmutex_unlock(&g_sound.lock);
      return -E2BIG;
    }

  /* Request any in-flight tone_write to stop before we reprogram the HW. */

  atomic_store_explicit(&g_sound.stop_flag, true, memory_order_relaxed);
  stm32_sound_stop_pcm();

  /* 8. copy payload and start playback.  FLAT build: memcpy is safe.
   *    TODO: switch to copyin() helper when CONFIG_BUILD_PROTECTED is used.
   */

  memcpy(g_pcm_buf, (const uint8_t *)ubuf + h->hdr_size, payload_bytes);

  int ret = stm32_sound_play_pcm(g_pcm_buf, h->sample_count, h->sample_rate);
  if (ret >= 0)
    {
      g_sound.owner = filep;
      g_sound.mode  = SOUND_MODE_PCM;
      /* stop_flag stays as-is; any pending tone_write observes it and
       * exits with -EINTR.  It is only cleared by tone_write when a new
       * tune is started.
       */
    }

  nxmutex_unlock(&g_sound.lock);
  return (ret < 0) ? (ssize_t)ret : (ssize_t)nbytes;
}

static int pcm_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  switch (cmd)
    {
      case TONEIOC_VOLUME_SET:
        {
          nxmutex_lock(&g_sound.lock);
          stm32_sound_set_volume((uint8_t)arg);
          nxmutex_unlock(&g_sound.lock);
          return OK;
        }

      case TONEIOC_VOLUME_GET:
        {
          FAR int *out = (FAR int *)((uintptr_t)arg);
          if (out == NULL)
            {
              return -EINVAL;
            }

          *out = (int)stm32_sound_get_volume();
          return OK;
        }

      case TONEIOC_STOP:
        {
          nxmutex_lock(&g_sound.lock);
          atomic_store_explicit(&g_sound.stop_flag, true,
                                memory_order_relaxed);
          stm32_sound_stop_pcm();
          g_sound.owner = NULL;
          g_sound.mode  = SOUND_MODE_IDLE;
          nxmutex_unlock(&g_sound.lock);
          return OK;
        }

      default:
        return -ENOTTY;
    }
}

/****************************************************************************
 * Private Data (file operations)
 ****************************************************************************/

static const struct file_operations g_pcm_fops =
{
  .open  = pcm_open,
  .close = pcm_close,
  .read  = NULL,
  .write = pcm_write,
  .seek  = NULL,
  .ioctl = pcm_ioctl,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_pcm_register(void)
{
  int ret = register_driver("/dev/pcm0", &g_pcm_fops, 0666, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "pcm: register_driver failed: %d\n", ret);
      return ret;
    }

  syslog(LOG_INFO, "pcm: /dev/pcm0 registered (%u sample buf)\n",
         (unsigned int)PCM_BUF_SAMPLES);
  return OK;
}
