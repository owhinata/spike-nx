/****************************************************************************
 * apps/btsensor/btsensor_button.c
 *
 * BT control button input driver for btsensor (Issue #56 Commit C).
 *
 * The Hub's BT button is the same PA0 input the bootloader uses to enter
 * DFU.  We talk to the kernel-side button driver through /dev/buttons0
 * (CONFIG_INPUT_BUTTONS_LOWER), which exposes board_buttons() /
 * board_button_irq() through a poll(2)-able char device.  The fd is
 * registered as a btstack data source so press/release wake the run
 * loop directly; long-press is detected with a btstack timer to keep
 * the entire BT lifecycle on the run loop's single thread.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <syslog.h>
#include <unistd.h>

#include <arch/board/board.h>

#include "btstack_defines.h"
#include "btstack_run_loop.h"

#include "btsensor_button.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_APP_BTSENSOR_BTN_LONG_PRESS_MS
#  define CONFIG_APP_BTSENSOR_BTN_LONG_PRESS_MS  1500
#endif

#define BTN_DEVPATH         "/dev/btbutton"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static btsensor_button_cb_t g_on_short;
static btsensor_button_cb_t g_on_long;

static int                    g_fd = -1;
static btstack_data_source_t  g_btn_ds;
static btstack_timer_source_t g_long_timer;

static bool g_initialized;
static bool g_pressed;        /* last observed level */
static bool g_long_fired;     /* long press already dispatched */

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static bool read_pressed(void)
{
  uint8_t state = 0;
  ssize_t n = read(g_fd, &state, sizeof(state));
  if (n <= 0)
    {
      return g_pressed;     /* no fresh data → reuse last known state */
    }

  return state != 0;
}

static void long_timer_handler(btstack_timer_source_t *ts)
{
  (void)ts;

  if (g_pressed && !g_long_fired)
    {
      g_long_fired = true;
      syslog(LOG_INFO, "btsensor_button: long press\n");
      if (g_on_long != NULL)
        {
          g_on_long();
        }
    }
}

static void btn_process(btstack_data_source_t *ds,
                        btstack_data_source_callback_type_t type)
{
  (void)ds;
  (void)type;

  bool pressed = read_pressed();
  if (pressed == g_pressed)
    {
      return;
    }

  g_pressed = pressed;

  if (pressed)
    {
      g_long_fired = false;
      btstack_run_loop_remove_timer(&g_long_timer);
      btstack_run_loop_set_timer(&g_long_timer,
                                 CONFIG_APP_BTSENSOR_BTN_LONG_PRESS_MS);
      btstack_run_loop_add_timer(&g_long_timer);
    }
  else
    {
      btstack_run_loop_remove_timer(&g_long_timer);
      if (!g_long_fired)
        {
          syslog(LOG_INFO, "btsensor_button: short press\n");
          if (g_on_short != NULL)
            {
              g_on_short();
            }
        }
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void btsensor_button_set_callbacks(btsensor_button_cb_t on_short,
                                   btsensor_button_cb_t on_long)
{
  g_on_short = on_short;
  g_on_long  = on_long;
}

int btsensor_button_init(void)
{
  if (g_initialized)
    {
      return 0;
    }

  g_fd = open(BTN_DEVPATH, O_RDONLY | O_NONBLOCK);
  if (g_fd < 0)
    {
      syslog(LOG_ERR, "btsensor_button: open %s errno=%d\n",
             BTN_DEVPATH, errno);
      return -errno;
    }

  /* Seed the cached level so we don't fire a phantom release on the
   * first poll wakeup.
   */

  g_pressed    = read_pressed();
  g_long_fired = false;

  btstack_run_loop_set_data_source_fd(&g_btn_ds, g_fd);
  btstack_run_loop_set_data_source_handler(&g_btn_ds, btn_process);
  btstack_run_loop_add_data_source(&g_btn_ds);
  btstack_run_loop_enable_data_source_callbacks(
      &g_btn_ds, DATA_SOURCE_CALLBACK_READ);

  btstack_run_loop_set_timer_handler(&g_long_timer, long_timer_handler);

  g_initialized = true;
  return 0;
}

void btsensor_button_deinit(void)
{
  if (!g_initialized)
    {
      return;
    }

  btstack_run_loop_remove_timer(&g_long_timer);
  btstack_run_loop_disable_data_source_callbacks(
      &g_btn_ds, DATA_SOURCE_CALLBACK_READ);
  btstack_run_loop_remove_data_source(&g_btn_ds);

  if (g_fd >= 0)
    {
      close(g_fd);
      g_fd = -1;
    }

  g_initialized = false;
}
