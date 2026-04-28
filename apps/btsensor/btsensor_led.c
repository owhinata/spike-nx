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

/* Double-blink sub-pattern (Issue #73).  Each cycle is
 *   ON (DOUBLE_BLINK_ON_MS)
 *   OFF (DOUBLE_BLINK_GAP_MS)
 *   ON (DOUBLE_BLINK_ON_MS)
 *   OFF (period_ms - 3 * DOUBLE_BLINK_ON_MS)
 * so the rhythm is "ti-tick . . . . . . ti-tick" — the long rest
 * dominates the period, giving a clearly distinct feel from the
 * symmetric blink used for BT_ADVERTISING.
 */

#define DOUBLE_BLINK_ON_MS   100
#define DOUBLE_BLINK_GAP_MS  100
#define DOUBLE_BLINK_MIN_PERIOD_MS  400  /* 2*ON + GAP + 100ms rest */

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum led_mode_e
{
  LED_MODE_OFF = 0,
  LED_MODE_SOLID,
  LED_MODE_BLINK,
  LED_MODE_DOUBLE_BLINK,
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

/* Double-blink state (Issue #73).  step cycles 0..3 inside each period:
 *   step 0: LED on  for DOUBLE_BLINK_ON_MS
 *   step 1: LED off for DOUBLE_BLINK_GAP_MS
 *   step 2: LED on  for DOUBLE_BLINK_ON_MS
 *   step 3: LED off for the remainder of period_ms
 */

static btstack_timer_source_t g_double_blink_timer;
static uint16_t               g_double_blink_period_ms;
static uint8_t                g_double_blink_step;

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

static void double_blink_timer_handler(btstack_timer_source_t *ts)
{
  if (g_mode != LED_MODE_DOUBLE_BLINK)
    {
      return;          /* mode changed under us */
    }

  /* Step state machine: 0 ON / 1 OFF / 2 ON / 3 OFF (long rest), wrap. */

  uint16_t next_ms;
  switch (g_double_blink_step)
    {
      case 0:
        set_blue(true);
        next_ms = DOUBLE_BLINK_ON_MS;
        break;
      case 1:
        set_blue(false);
        next_ms = DOUBLE_BLINK_GAP_MS;
        break;
      case 2:
        set_blue(true);
        next_ms = DOUBLE_BLINK_ON_MS;
        break;
      case 3:
      default:
        set_blue(false);
        /* Long rest = total period − the three short slots above.
         * btsensor_led_double_blink_blue() guarantees period_ms >=
         * DOUBLE_BLINK_MIN_PERIOD_MS so this stays positive.
         */

        next_ms = (uint16_t)(g_double_blink_period_ms
                             - 2 * DOUBLE_BLINK_ON_MS
                             - DOUBLE_BLINK_GAP_MS);
        break;
    }

  g_double_blink_step = (uint8_t)((g_double_blink_step + 1) & 0x3);

  btstack_run_loop_set_timer(ts, next_ms);
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
  btstack_run_loop_remove_timer(&g_double_blink_timer);
  btstack_run_loop_remove_timer(&g_fail_timer);
  g_blink_on          = false;
  g_double_blink_step = 0;
  g_fail_on           = false;
  g_fail_remaining    = 0;
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
  btstack_run_loop_set_timer_handler(&g_double_blink_timer,
                                     double_blink_timer_handler);
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

void btsensor_led_double_blink_blue(uint16_t period_ms)
{
  if (period_ms == 0)
    {
      btsensor_led_off();
      return;
    }

  /* The double-blink rhythm needs ~300 ms for its two short pulses + the
   * gap between them; anything tighter would just look like a fast
   * regular blink, so collapse to the symmetric blink helper instead.
   */

  if (period_ms < DOUBLE_BLINK_MIN_PERIOD_MS)
    {
      btsensor_led_blink_blue(period_ms);
      return;
    }

  cancel_animations();
  g_mode                   = LED_MODE_DOUBLE_BLINK;
  g_double_blink_period_ms = period_ms;
  g_double_blink_step      = 1;          /* next handler call advances */
  set_blue(true);                        /* start of cycle: first pulse */

  btstack_run_loop_set_timer(&g_double_blink_timer, DOUBLE_BLINK_ON_MS);
  btstack_run_loop_add_timer(&g_double_blink_timer);
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
