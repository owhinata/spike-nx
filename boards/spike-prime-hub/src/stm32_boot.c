/****************************************************************************
 * boards/spike-prime-hub/src/stm32_boot.c
 *
 * SPIKE Prime Hub boot initialization.
 * PA13 (BAT_PWR_EN) must be set HIGH immediately to keep power on.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32.h"
#include "spike_prime_hub.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_boardinitialize(void)
{
  /* Keep main power on — PA13 must be HIGH or the Hub shuts down */

  stm32_configgpio(GPIO_BAT_PWR_EN);

  /* Enable 3.3V to I/O ports */

  stm32_configgpio(GPIO_PORT_3V3_EN);

#ifdef CONFIG_STM32_OTGFS
  if (stm32_usbinitialize)
    {
      stm32_usbinitialize();
    }
#endif
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  stm32_bringup();
}
#endif
