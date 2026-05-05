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
 * against concurrent close()/ioctl()/tone_write() is closed.  Under
 * CONFIG_BUILD_PROTECTED the user pointer is range-checked via
 * board_usercheck.h before any dereference, and the header is copied
 * into a kernel-local struct before validation to close the TOCTOU
 * window against user-side mutation between the magic/size/count
 * checks and the payload memcpy.
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

#include "board_usercheck.h"
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
  struct pcm_write_hdr_s hdr;
  size_t payload_bytes;
  int ret;

  if (ubuf == NULL)
    {
      return -EINVAL;
    }

  /* 1. minimum header size — bound the user range before any deref */

  if (nbytes < sizeof(hdr))
    {
      return -EINVAL;
    }

  /* 2. validate the entire user buffer is in user-readable memory */

  if (!board_user_in_ok(ubuf, nbytes))
    {
      return -EFAULT;
    }

  nxmutex_lock(&g_sound.lock);

  /* 3. copy header into kernel-local first to close TOCTOU */

  memcpy(&hdr, ubuf, sizeof(hdr));

  /* 4. forward-compatible version gate */

  if (hdr.version > PCM_WRITE_VERSION)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 5. magic */

  if (hdr.magic != PCM_WRITE_MAGIC)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 6. hdr_size / sample_rate range */

  if (hdr.hdr_size < sizeof(hdr) || hdr.hdr_size > nbytes)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  if (hdr.sample_rate < 1000 || hdr.sample_rate > 100000)
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 7. overflow check on sample_count * 2 */

  if (hdr.sample_count > (UINT32_MAX / 2))
    {
      nxmutex_unlock(&g_sound.lock);
      return -EOVERFLOW;
    }

  payload_bytes = (size_t)hdr.sample_count * sizeof(uint16_t);

  /* 8. total bytes consistency */

  if (payload_bytes != (nbytes - hdr.hdr_size))
    {
      nxmutex_unlock(&g_sound.lock);
      return -EINVAL;
    }

  /* 9. kernel buffer capacity */

  if (payload_bytes > sizeof(g_pcm_buf))
    {
      nxmutex_unlock(&g_sound.lock);
      return -E2BIG;
    }

  /* Request any in-flight tone_write to stop before we reprogram the HW. */

  atomic_store_explicit(&g_sound.stop_flag, true, memory_order_relaxed);
  stm32_sound_stop_pcm();

  /* 10. copy payload into kernel buffer.  Whole ubuf was range-checked
   *     above so this slice is in-range; user-side mutation between the
   *     header copy and this memcpy can only corrupt the audio output,
   *     not escape into kernel memory.
   */

  memcpy(g_pcm_buf, (const uint8_t *)ubuf + hdr.hdr_size, payload_bytes);

  ret = stm32_sound_play_pcm(g_pcm_buf, hdr.sample_count, hdr.sample_rate);
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

          if (!board_user_out_ok(out, sizeof(*out)))
            {
              return -EFAULT;
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
