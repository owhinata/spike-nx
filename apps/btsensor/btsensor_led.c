/****************************************************************************
 * apps/btsensor/btsensor_led.c
 *
 * BT LED helper for btsensor (Issue #56 Commit C).
 *
 * Thin wrapper over /dev/rgbled0 (TLC5955 char device).  The BT button
 * state machine in btsensor_main.c drives off / blue-blink / solid-blue
 * / fail-blink visual feedback by calling these helpers, all of which
 * run on the BTstack main thread.  Blink animations are driven by a
 * btstack timer source so they share the run loop's cadence and stop
 * cleanly when the daemon tears down.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <syslog.h>
#include <unistd.h>

#include <arch/board/board_rgbled.h>

#include "btstack_run_loop.h"

#include "btsensor_led.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LED_DEVPATH         "/dev/rgbled0"

/* TLC5955 PWM is 16-bit.  We never need a brightness gradient — full
 * scale is a comfortable indicator on the SPIKE Hub's translucent case.
 */

#define LED_DUTY_OFF        0
#define LED_DUTY_ON         0xffff

/* Fail-blink half-cycle.  Plan calls for "数百ms × 数回"; 150 ms gives
 * a clearly visible 3 Hz cadence at the default 3 pulses (~900 ms
 * total) without dragging on long enough to obscure the next state.
 */

#define FAIL_BLINK_HALF_MS  150

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum led_mode_e
{
  LED_MODE_OFF = 0,
  LED_MODE_SOLID,
  LED_MODE_BLINK,
  LED_MODE_FAIL,
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static int                    g_fd = -1;
static enum led_mode_e        g_mode = LED_MODE_OFF;

/* Slow-blink state. */

static btstack_timer_source_t g_blink_timer;
static bool                   g_blink_on;
static uint16_t               g_blink_period_ms;

/* Fail-blink state. */

static btstack_timer_source_t g_fail_timer;
static bool                   g_fail_on;
static uint8_t                g_fail_remaining;   /* remaining toggles */

/****************************************************************************
 * Private Functions — low-level channel set
 ****************************************************************************/

static void set_channel(uint8_t channel, uint16_t duty)
{
  if (g_fd < 0)
    {
      return;
    }

  struct rgbled_duty_s d =
    {
      .channel = channel,
      .value   = duty,
    };
  (void)ioctl(g_fd, RGBLEDIOC_SETDUTY, (unsigned long)(uintptr_t)&d);
  (void)ioctl(g_fd, RGBLEDIOC_UPDATE, 0);
}

static void set_blue(bool on)
{
  set_channel(TLC5955_CH_BT_B, on ? LED_DUTY_ON : LED_DUTY_OFF);
}

static void set_off(void)
{
  /* Force every BT channel low — covers any earlier state where the
   * green or red component might have been driven (none today, but
   * cheap insurance for future modes).
   */

  set_channel(TLC5955_CH_BT_B, LED_DUTY_OFF);
  set_channel(TLC5955_CH_BT_G, LED_DUTY_OFF);
  set_channel(TLC5955_CH_BT_R, LED_DUTY_OFF);
}

/****************************************************************************
 * Private Functions — timer callbacks
 ****************************************************************************/

static void blink_timer_handler(btstack_timer_source_t *ts)
{
  if (g_mode != LED_MODE_BLINK)
    {
      return;          /* mode changed under us */
    }

  g_blink_on = !g_blink_on;
  set_blue(g_blink_on);

  btstack_run_loop_set_timer(ts, g_blink_period_ms / 2);
  btstack_run_loop_add_timer(ts);
}

static void fail_timer_handler(btstack_timer_source_t *ts)
{
  if (g_mode != LED_MODE_FAIL)
    {
      return;
    }

  if (g_fail_remaining == 0)
    {
      set_off();
      g_mode = LED_MODE_OFF;
      return;
    }

  g_fail_on = !g_fail_on;
  set_blue(g_fail_on);
  g_fail_remaining--;

  btstack_run_loop_set_timer(ts, FAIL_BLINK_HALF_MS);
  btstack_run_loop_add_timer(ts);
}

/****************************************************************************
 * Private Functions — mode helpers
 ****************************************************************************/

static void cancel_animations(void)
{
  btstack_run_loop_remove_timer(&g_blink_timer);
  btstack_run_loop_remove_timer(&g_fail_timer);
  g_blink_on       = false;
  g_fail_on        = false;
  g_fail_remaining = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int btsensor_led_init(void)
{
  if (g_fd >= 0)
    {
      return 0;
    }

  g_fd = open(LED_DEVPATH, O_RDWR);
  if (g_fd < 0)
    {
      syslog(LOG_ERR, "btsensor_led: open %s errno=%d\n",
             LED_DEVPATH, errno);
      return -errno;
    }

  btstack_run_loop_set_timer_handler(&g_blink_timer, blink_timer_handler);
  btstack_run_loop_set_timer_handler(&g_fail_timer, fail_timer_handler);

  g_mode = LED_MODE_OFF;
  set_off();
  return 0;
}

void btsensor_led_deinit(void)
{
  cancel_animations();
  set_off();

  if (g_fd >= 0)
    {
      close(g_fd);
      g_fd = -1;
    }

  g_mode = LED_MODE_OFF;
}

void btsensor_led_off(void)
{
  cancel_animations();
  g_mode = LED_MODE_OFF;
  set_off();
}

void btsensor_led_solid_blue(void)
{
  cancel_animations();
  g_mode = LED_MODE_SOLID;
  set_blue(true);
}

void btsensor_led_blink_blue(uint16_t period_ms)
{
  if (period_ms == 0)
    {
      btsensor_led_off();
      return;
    }

  cancel_animations();
  g_mode             = LED_MODE_BLINK;
  g_blink_period_ms  = period_ms;
  g_blink_on         = true;
  set_blue(true);

  btstack_run_loop_set_timer(&g_blink_timer, period_ms / 2);
  btstack_run_loop_add_timer(&g_blink_timer);
}

void btsensor_led_fail_blink(uint8_t count)
{
  if (count == 0)
    {
      btsensor_led_off();
      return;
    }

  cancel_animations();
  g_mode = LED_MODE_FAIL;

  /* Each pulse = on + off, so two timer toggles. */

  g_fail_remaining = (uint8_t)(count * 2 - 1);
  g_fail_on        = true;
  set_blue(true);

  btstack_run_loop_set_timer(&g_fail_timer, FAIL_BLINK_HALF_MS);
  btstack_run_loop_add_timer(&g_fail_timer);
}
