/****************************************************************************
 * boards/b-l4s5i-iot01a/src/stm32_autoleds.c
 *
 * B-L4S5I-IOT01A has 2 user LEDs:
 *   LD1 (green) = PA5
 *   LD2 (red)   = PB14
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32l4_gpio.h"
#include "b-l4s5i-iot01a.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_autoled_initialize(void)
{
  stm32l4_configgpio(GPIO_LED1);
  stm32l4_configgpio(GPIO_LED2);
}

void board_autoled_on(int led)
{
  if (led == 1 || led == 3)
    {
      stm32l4_gpiowrite(GPIO_LED2, true);
    }
}

void board_autoled_off(int led)
{
  if (led == 3)
    {
      stm32l4_gpiowrite(GPIO_LED2, false);
    }
}

#endif /* CONFIG_ARCH_LEDS */
