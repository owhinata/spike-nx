/****************************************************************************
 * boards/spike-prime-hub/src/stm32_bluetooth.c
 *
 * CC2564C Bluetooth power-on and UART bring-up for SPIKE Prime Hub.
 *
 * Steps performed here:
 *   1. Start the 32.768 kHz TIM8 CH4 slow clock on PC9 and hold the chip
 *      in reset (nSHUTD LOW).  The clock must be stable before we raise
 *      nSHUTD or the CC2564C fails to leave ROM boot.
 *   2. Release nSHUTD after >= 50 ms and wait for the chip to finish
 *      booting (~150 ms).
 *   3. Instantiate the USART2 lower-half and keep the pointer available
 *      for the Bluetooth host stack (see stm32_btuart_lower()).
 *
 * HCI reset, the TI init script load and baud negotiation are delegated
 * to the btstack port in apps/btsensor/port/ (Issue #52).  This file no
 * longer talks HCI directly.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_USART2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TI datasheet recommends >= 5 ms nSHUTD low to guarantee a full reset;
 * we use 50 ms to stay safe under NuttX scheduling jitter.
 */

#define BT_NSHUTD_LOW_MS       50

/* Time the CC2564C takes to finish its ROM boot after nSHUTD goes HIGH
 * before the first HCI command can be sent.  150 ms is generous and
 * matches what btstack's CC256x bring-up uses in polled ports.
 */

#define BT_BOOT_SETTLE_MS      150

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct btuart_lowerhalf_s *g_bt_lower;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bluetooth_initialize
 *
 * Description:
 *   Supply the CC2564C slow clock, toggle nSHUTD and hand back a ready
 *   USART2 lower-half.  Safe to call from stm32_bringup() on the
 *   BOARD_LATE_INITIALIZE path.  On failure the CC2564C is left in reset
 *   so a later retry starts clean.
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int stm32_bluetooth_initialize(void)
{
  FAR struct btuart_lowerhalf_s *lower;
  int ret;

  stm32_configgpio(GPIO_BT_NSHUTD);
  stm32_gpiowrite(GPIO_BT_NSHUTD, false);

  ret = stm32_bt_slowclk_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "BT: slow clock init failed: %d\n", ret);
      return ret;
    }

  lower = stm32_btuart_instantiate();
  if (lower == NULL)
    {
      syslog(LOG_ERR, "BT: btuart instantiate failed\n");
      stm32_gpiowrite(GPIO_BT_NSHUTD, false);
      return -ENODEV;
    }

  up_mdelay(BT_NSHUTD_LOW_MS);
  stm32_gpiowrite(GPIO_BT_NSHUTD, true);
  up_mdelay(BT_BOOT_SETTLE_MS);

  g_bt_lower = lower;

  ret = stm32_btuart_chardev_register(lower);
  if (ret < 0)
    {
      syslog(LOG_ERR, "BT: chardev register failed: %d\n", ret);
      stm32_gpiowrite(GPIO_BT_NSHUTD, false);
      g_bt_lower = NULL;
      return ret;
    }

  syslog(LOG_INFO, "BT: CC2564C powered, /dev/ttyBT ready\n");
  return OK;
}

/****************************************************************************
 * Name: stm32_btuart_lower
 *
 * Description:
 *   Return the USART2 lower-half initialised by
 *   stm32_bluetooth_initialize().  The btstack UART adapter (Step C) uses
 *   this to wire up H4 transport.
 *
 * Returned Value:
 *   Pointer to the lower-half instance, or NULL if bring-up has not run
 *   or failed.
 *
 ****************************************************************************/

FAR struct btuart_lowerhalf_s *stm32_btuart_lower(void)
{
  return g_bt_lower;
}

#endif /* CONFIG_STM32_USART2 */
