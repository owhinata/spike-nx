/****************************************************************************
 * boards/stm32f413-discovery/src/stm32_appinit.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <nuttx/board.h>

#include "stm32f413_discovery.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef CONFIG_BOARD_LATE_INITIALIZE
  return OK;
#else
  return stm32_bringup();
#endif
}
