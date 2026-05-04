/****************************************************************************
 * boards/spike-prime-hub/src/stm32_reset.c
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "spike_prime_hub.h"

int board_reset(int status)
{
  /* Persist the status arg (assert.c passes CONFIG_BOARD_ASSERT_RESET_VALUE
   * = 2 on the assert/HardFault path; user-initiated NSH `reboot` passes 0)
   * so the next boot's BCRUMB log can distinguish them.
   */

  stm32_bcrumb_set_board_reset(status);
  up_systemreset();
  return 0;
}
