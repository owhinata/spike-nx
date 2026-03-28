/****************************************************************************
 * boards/spike-prime-hub/src/stm32_reset.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

int board_reset(int status)
{
  up_systemreset();
  return 0;
}
