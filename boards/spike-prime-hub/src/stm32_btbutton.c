/****************************************************************************
 * boards/spike-prime-hub/src/stm32_btbutton.c
 *
 * BT control button character device for SPIKE Prime Hub (Issue #56
 * Commit C).
 *
 * The Hub's "Bluetooth" button is wired through resistor ladder DEV_1
 * (PA1, ADC1 rank 5) channel CH2 — there is no GPIO IRQ for it.  This
 * driver polls the ADC at LPWORK cadence (50 ms), runs the pybricks
 * resistor-ladder decoder, and exposes the resulting state via
 * /dev/btbutton:
 *
 *   read(fd, &b, 1)  -> b = 0 (released) or 1 (pressed)
 *   poll(fd, POLLIN) -> wakes on every press / release transition
 *
 * The user-mode btsensor app uses this to implement BT button short /
 * long press detection without having to touch ADC or kernel internals.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/clock.h>
#include <nuttx/fs/fs.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/wqueue.h>

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_ADC1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BTBTN_POLL_INTERVAL_MS   50

/* Number of consecutive ADC samples that must agree before a state
 * transition is declared.  5 × 50 ms = 250 ms total — well above the
 * resistor ladder's idle noise band (the BT-pressed threshold sits
 * close to the no-button rail, so a single noisy sample crosses it
 * easily) while still feeling instant for a deliberate button press.
 */

#define BTBTN_DEBOUNCE_SAMPLES   5
#define BTBTN_NPOLLWAITERS       2

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct btbtn_dev_s
{
  mutex_t          lock;
  struct work_s    work;
  bool             pressed;
  bool             pending;          /* candidate next level */
  uint8_t          pending_count;    /* consecutive samples matching pending */
  bool             fresh;            /* state changed since last read() */
  uint32_t         openrefs;
  FAR struct pollfd *fds[BTBTN_NPOLLWAITERS];
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int     btbtn_open (FAR struct file *filep);
static int     btbtn_close(FAR struct file *filep);
static ssize_t btbtn_read (FAR struct file *filep, FAR char *buffer,
                            size_t buflen);
static int     btbtn_poll (FAR struct file *filep, FAR struct pollfd *fds,
                            bool setup);

static void    btbtn_work_handler(FAR void *arg);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_btbtn_fops =
{
  btbtn_open,    /* open */
  btbtn_close,   /* close */
  btbtn_read,    /* read */
  NULL,          /* write */
  NULL,          /* seek */
  NULL,          /* ioctl */
  NULL,          /* mmap */
  NULL,          /* truncate */
  btbtn_poll     /* poll */
};

static struct btbtn_dev_s g_btbtn;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool btbtn_sample_adc(void)
{
  uint16_t val   = stm32_adc_read(ADC_RANK_BTN_LRB);
  uint8_t  flags = resistor_ladder_decode(val, g_ladder_dev1_levels);

  /* On the SPIKE Prime Hub the resistor ladder bit for the BT button
   * is *cleared* when the button is pressed (the button shorts a pull
   * out of the divider).  Idle reads as flags including CH2; pressing
   * BT drops CH2 from the active set.  flags==0xff indicates a
   * decoder error (out-of-range ADC) and is treated as released.
   *
   * NOTE (Issue #56 follow-up): empirical testing showed the PA1 ADC
   * value oscillates around the CH2 threshold even at idle on this
   * Hub variant, so the stm32_btbutton driver currently does not
   * detect physical button presses reliably.  The BT state machine
   * in apps/btsensor still relies on this path; investigation of
   * the resistor-ladder threshold table / hardware wiring is tracked
   * separately.
   */

  if (flags == 0xff)
    {
      return false;
    }

  return (flags & RLAD_CH2) == 0;
}

static void btbtn_notify_locked(void)
{
  poll_notify(g_btbtn.fds, BTBTN_NPOLLWAITERS, POLLIN);
}

static void btbtn_work_handler(FAR void *arg)
{
  bool sample = btbtn_sample_adc();

  nxmutex_lock(&g_btbtn.lock);

  if (sample == g_btbtn.pressed)
    {
      /* Already at the stable state — just clear any pending. */

      g_btbtn.pending       = sample;
      g_btbtn.pending_count = 0;
    }
  else if (sample != g_btbtn.pending)
    {
      /* New candidate — restart the agreement counter. */

      g_btbtn.pending       = sample;
      g_btbtn.pending_count = 1;
    }
  else
    {
      g_btbtn.pending_count++;
      if (g_btbtn.pending_count >= BTBTN_DEBOUNCE_SAMPLES)
        {
          g_btbtn.pressed       = sample;
          g_btbtn.pending_count = 0;
          g_btbtn.fresh         = true;
          btbtn_notify_locked();
        }
    }

  /* Re-arm the polling tick — runs forever, regardless of open count. */

  work_queue(LPWORK, &g_btbtn.work, btbtn_work_handler, NULL,
             MSEC2TICK(BTBTN_POLL_INTERVAL_MS));

  nxmutex_unlock(&g_btbtn.lock);
}

/****************************************************************************
 * Private Functions — file operations
 ****************************************************************************/

static int btbtn_open(FAR struct file *filep)
{
  nxmutex_lock(&g_btbtn.lock);
  g_btbtn.openrefs++;
  nxmutex_unlock(&g_btbtn.lock);
  return OK;
}

static int btbtn_close(FAR struct file *filep)
{
  nxmutex_lock(&g_btbtn.lock);
  if (g_btbtn.openrefs > 0)
    {
      g_btbtn.openrefs--;
    }

  nxmutex_unlock(&g_btbtn.lock);
  return OK;
}

static ssize_t btbtn_read(FAR struct file *filep, FAR char *buffer,
                          size_t buflen)
{
  if (buffer == NULL || buflen < 1)
    {
      return -EINVAL;
    }

  nxmutex_lock(&g_btbtn.lock);
  buffer[0] = g_btbtn.pressed ? 1 : 0;
  g_btbtn.fresh = false;             /* consumed — no new POLLIN until
                                      * the next state transition */
  nxmutex_unlock(&g_btbtn.lock);
  return 1;
}

static int btbtn_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  int ret = OK;

  nxmutex_lock(&g_btbtn.lock);

  if (setup)
    {
      int slot = -1;
      for (int i = 0; i < BTBTN_NPOLLWAITERS; i++)
        {
          if (g_btbtn.fds[i] == NULL)
            {
              g_btbtn.fds[i] = fds;
              fds->priv      = &g_btbtn.fds[i];
              slot           = i;
              break;
            }
        }

      if (slot < 0)
        {
          ret = -EBUSY;
          goto out;
        }

      /* Only assert POLLIN if the state has actually changed since the
       * last read() — otherwise the run loop's poll() returns
       * immediately on every iteration, busy-looping the CPU.
       */

      if (g_btbtn.fresh)
        {
          poll_notify(&fds, 1, POLLIN);
        }
    }
  else
    {
      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;
      if (slot != NULL)
        {
          *slot = NULL;
          fds->priv = NULL;
        }
    }

out:
  nxmutex_unlock(&g_btbtn.lock);
  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_btbutton_initialize(void)
{
  int ret;

  nxmutex_init(&g_btbtn.lock);
  g_btbtn.pressed  = false;
  g_btbtn.openrefs = 0;
  memset(g_btbtn.fds, 0, sizeof(g_btbtn.fds));

  ret = register_driver("/dev/btbutton", &g_btbtn_fops, 0444, &g_btbtn);
  if (ret < 0)
    {
      return ret;
    }

  /* Kick off the ADC polling tick. */

  work_queue(LPWORK, &g_btbtn.work, btbtn_work_handler, NULL,
             MSEC2TICK(BTBTN_POLL_INTERVAL_MS));
  return OK;
}

#endif /* CONFIG_STM32_ADC1 */
