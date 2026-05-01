/****************************************************************************
 * boards/spike-prime-hub/src/stm32_hpwork_softdog.c
 *
 * Software watchdog that targets HPWORK queue health, complementing the
 * STM32 IWDG (which is petted from a kernel timer ISR and therefore can
 * stay alive while every HPWORK callback is dead).
 *
 * Symptom this guards against:
 *   - HPWORK queue gets stuck in some callback (lock contention, USB
 *     CDC TX backpressure under heavy `printf`, hung SPI/I2C, etc.)
 *   - Power button monitor (`stm32_power.c:btn_monitor_work`) is on
 *     HPWORK, so a long-press no longer triggers reset
 *   - NSH tasks blocked in `printf` to /dev/console get no input
 *   - But IWDG is still kicked (kernel-timer ISR context) → no reset
 *   - Result: bricked-feeling board with no obvious recovery path
 *
 * Mechanism:
 *   - HPWORK self-pet: a periodic HPWORK task bumps `g_last_hb_tick`
 *     every HPWORK_PET_INTERVAL_MS.
 *   - Independent kernel wdog (timer ISR ctx, NOT on HPWORK) checks
 *     `clock_systime_ticks() - g_last_hb_tick` every
 *     HPWORK_CHECK_INTERVAL_MS.  If the gap exceeds HPWORK_TIMEOUT_MS,
 *     the wdog calls `PANIC()`, which writes a backtrace to syslog
 *     (RAMLOG) and triggers `board_reset()` per
 *     CONFIG_BOARD_RESET_ON_ASSERT=2.
 *
 * Why this can detect the hang while IWDG cannot: the kernel wdog
 * timer runs in IRQ-driven softirq context (just like the IWDG kicker),
 * but its check is *gated by the HPWORK heartbeat*, not by raw kernel
 * timer ticks.  So if HPWORK stops, this wdog fires; the IWDG kicker
 * does not.
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <debug.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>

#include <nuttx/clock.h>
#include <nuttx/wdog.h>
#include <nuttx/wqueue.h>

#include "spike_prime_hub.h"

#ifdef CONFIG_SCHED_HPWORK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* HPWORK self-pet cadence — fast enough to catch a real hang within
 * HPWORK_TIMEOUT_MS, slow enough to be invisible relative to the LED
 * frame-sync (2 ms) and power-button polling (50 ms) on the same queue.
 */

#define HPWORK_PET_INTERVAL_MS      200u

/* Wdog timer cadence — runs in IRQ ctx, just bumps a deadline check.
 * Independent of HPWORK so it survives an HPWORK hang.
 */

#define HPWORK_CHECK_INTERVAL_MS    500u

/* Threshold for declaring HPWORK dead.  3 s = 15× the pet interval, so
 * we tolerate transient spikes (long FLASH program, brief lock
 * contention) but catch real lockups well before the user gives up
 * waiting on NSH.
 */

#define HPWORK_TIMEOUT_MS          3000u

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct work_s     g_pet_work;
static struct wdog_s     g_check_wdog;
static volatile clock_t  g_last_hb_tick;
static volatile uint32_t g_hb_count;
static bool              g_armed;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* HPWORK callback — bumps the heartbeat then re-arms itself. */

static void hpwork_softdog_pet(FAR void *arg)
{
  g_hb_count++;
  g_last_hb_tick = clock_systime_ticks();

  work_queue(HPWORK, &g_pet_work, hpwork_softdog_pet, NULL,
             MSEC2TICK(HPWORK_PET_INTERVAL_MS));
}

/* Wdog timer callback — runs in IRQ ctx.  Reads the heartbeat
 * timestamp and panics if HPWORK has been dark for too long.
 */

static void hpwork_softdog_check(wdparm_t arg)
{
  clock_t  now       = clock_systime_ticks();
  clock_t  age_ticks = now - g_last_hb_tick;
  uint32_t age_ms    = (uint32_t)TICK2MSEC(age_ticks);

  if (age_ms > HPWORK_TIMEOUT_MS)
    {
      /* HPWORK queue stuck.  Surface the diagnostic line, then panic.
       * `syslog(LOG_EMERG, ...)` reaches RAMLOG (NONBLOCKING) without
       * blocking on USB CDC, so the message is captured even when the
       * console is wedged.  PANIC() then dumps the kernel state and
       * resets per CONFIG_BOARD_RESET_ON_ASSERT.
       */

      syslog(LOG_EMERG,
             "HPWORK softdog FIRE: hb=%lu, age=%lu ms (timeout=%lu ms)\n",
             (unsigned long)g_hb_count,
             (unsigned long)age_ms,
             (unsigned long)HPWORK_TIMEOUT_MS);
      PANIC();
    }

  /* Re-arm */

  wd_start(&g_check_wdog, MSEC2TICK(HPWORK_CHECK_INTERVAL_MS),
           hpwork_softdog_check, 0);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_hpwork_softdog_initialize(void)
{
  if (g_armed)
    {
      return -EALREADY;
    }

  g_hb_count     = 0;
  g_last_hb_tick = clock_systime_ticks();

  /* First pet runs immediately — establishes a baseline heartbeat
   * before the check wdog ever fires.
   */

  int ret = work_queue(HPWORK, &g_pet_work, hpwork_softdog_pet, NULL, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "hpwork_softdog: pet schedule failed: %d\n", ret);
      return ret;
    }

  /* Arm the wdog to fire on the timeout cadence. */

  ret = wd_start(&g_check_wdog,
                 MSEC2TICK(HPWORK_CHECK_INTERVAL_MS),
                 hpwork_softdog_check, 0);
  if (ret < 0)
    {
      syslog(LOG_ERR, "hpwork_softdog: wdog start failed: %d\n", ret);
      return ret;
    }

  g_armed = true;
  syslog(LOG_INFO,
         "hpwork_softdog: armed (pet=%u ms, check=%u ms, timeout=%u ms)\n",
         HPWORK_PET_INTERVAL_MS, HPWORK_CHECK_INTERVAL_MS,
         HPWORK_TIMEOUT_MS);
  return OK;
}

#endif /* CONFIG_SCHED_HPWORK */
