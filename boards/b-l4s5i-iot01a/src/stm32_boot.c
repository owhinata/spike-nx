/****************************************************************************
 * boards/b-l4s5i-iot01a/src/stm32_boot.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32l4.h"
#include "b-l4s5i-iot01a.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32l4_board_initialize(void)
{
#ifdef CONFIG_ARCH_LEDS
  board_autoled_initialize();
#endif
}

#ifdef CONFIG_BOARD_LATE_INITIALIZE
void board_late_initialize(void)
{
  stm32l4_bringup();
}
#endif
