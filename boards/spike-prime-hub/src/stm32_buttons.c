/****************************************************************************
 * boards/spike-prime-hub/src/stm32_buttons.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_buttons[NUM_BUTTONS] =
{
  GPIO_BTN_USER
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      stm32_configgpio(g_buttons[i]);
    }

  return NUM_BUTTONS;
}

uint32_t board_buttons(void)
{
  uint32_t ret = 0;
  int i;

  for (i = 0; i < NUM_BUTTONS; i++)
    {
      bool released = stm32_gpioread(g_buttons[i]);
      if (!released)
        {
          ret |= (1 << i);
        }
    }

  return ret;
}

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, void *arg)
{
  int ret = -EINVAL;

  if (id >= MIN_IRQBUTTON && id <= MAX_IRQBUTTON)
    {
      ret = stm32_gpiosetevent(g_buttons[id], true, true, true,
                               irqhandler, arg);
    }

  return ret;
}
#endif

#endif /* CONFIG_ARCH_BUTTONS */
