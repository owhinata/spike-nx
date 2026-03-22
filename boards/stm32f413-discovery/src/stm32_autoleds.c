/****************************************************************************
 * boards/stm32f413-discovery/src/stm32_autoleds.c
 *
 * STM32F413H-Discovery has 2 LEDs:
 *   LD1 (green) = PE3
 *   LD2 (red)   = PC5
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "stm32f413_discovery.h"

#ifdef CONFIG_ARCH_LEDS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void board_autoled_initialize(void)
{
  stm32_configgpio(GPIO_LED1);
  stm32_configgpio(GPIO_LED2);
}

void board_autoled_on(int led)
{
  switch (led)
    {
      case LED_STARTED:       /* LED1 on */
      case LED_STACKCREATED:
      case LED_INIRQ:
        stm32_gpiowrite(GPIO_LED1, true);
        break;

      case LED_HEAPALLOCATE:  /* LED2 on */
      case LED_SIGNAL:
        stm32_gpiowrite(GPIO_LED2, true);
        break;

      case LED_IRQSENABLED:   /* Both on */
      case LED_ASSERTION:
        stm32_gpiowrite(GPIO_LED1, true);
        stm32_gpiowrite(GPIO_LED2, true);
        break;

      case LED_PANIC:         /* LED2 blink */
        stm32_gpiowrite(GPIO_LED2, true);
        break;

      default:
        break;
    }
}

void board_autoled_off(int led)
{
  switch (led)
    {
      case LED_STARTED:
      case LED_STACKCREATED:
      case LED_INIRQ:
        stm32_gpiowrite(GPIO_LED1, false);
        break;

      case LED_HEAPALLOCATE:
      case LED_SIGNAL:
        stm32_gpiowrite(GPIO_LED2, false);
        break;

      case LED_IRQSENABLED:
      case LED_ASSERTION:
        stm32_gpiowrite(GPIO_LED1, false);
        stm32_gpiowrite(GPIO_LED2, false);
        break;

      case LED_PANIC:
        stm32_gpiowrite(GPIO_LED2, false);
        break;

      default:
        break;
    }
}

#endif /* CONFIG_ARCH_LEDS */
