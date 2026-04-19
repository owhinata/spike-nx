/****************************************************************************
 * boards/spike-prime-hub/src/stm32_bt_slowclk.c
 *
 * Bluetooth 32.768 kHz slow clock generator for SPIKE Prime Hub.
 *
 * The CC2564C Bluetooth controller requires a free-running 32.768 kHz
 * reference clock on its SLOWCLK pin (PC9) while nSHUTD is HIGH so its
 * sleep oscillator and HCI state machine can run.  We generate it with
 * TIM8 CH4 configured as a 50% duty PWM.
 *
 *   Clock  : APB2 timer = PCLK2 = 96 MHz (no prescaler)
 *   PSC    : 0     (divide by 1)
 *   ARR    : 2929  (period = 2930 ticks -> 96e6 / 2930 = 32.7645 kHz,
 *                   -0.011% offset from nominal 32.768 kHz)
 *   CCR4   : 1465  (50% duty)
 *   Pin    : PC9 AF3 = TIM8_CH4
 *
 * Must be stable before the CC2564C nSHUTD line is driven HIGH, otherwise
 * the chip fails to boot.  Call stm32_bt_slowclk_initialize() from the
 * Bluetooth bring-up sequence before toggling nSHUTD.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/timers/timer.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_tim.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_TIM8

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define BT_SLOWCLK_TIM   8    /* TIM8 (advanced-control timer) */
#define BT_SLOWCLK_CH    4    /* CH4 -> PC9 AF3 */
#define BT_SLOWCLK_ARR   2929 /* period counter: 96 MHz / 2930 = 32.7645 kHz */
#define BT_SLOWCLK_CCR   1465 /* 50% duty (ARR+1)/2 */

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_tim_dev_s *g_bt_slowclk_tim;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_bt_slowclk_initialize
 *
 * Description:
 *   Configure TIM8 CH4 to output a continuous 32.768 kHz 50% duty PWM on
 *   PC9, supplying the CC2564C SLOWCLK.  Call before driving nSHUTD HIGH.
 *
 * Returned Value:
 *   OK on success; negated errno on failure.
 *
 ****************************************************************************/

int stm32_bt_slowclk_initialize(void)
{
  if (g_bt_slowclk_tim != NULL)
    {
      return OK;  /* Already running — idempotent */
    }

  stm32_configgpio(GPIO_TIM8_CH4_BT_SLOWCLK);

  g_bt_slowclk_tim = stm32_tim_init(BT_SLOWCLK_TIM);
  if (g_bt_slowclk_tim == NULL)
    {
      syslog(LOG_ERR, "BT slowclk: stm32_tim_init(%d) failed\n",
             BT_SLOWCLK_TIM);
      return -ENODEV;
    }

  STM32_TIM_SETMODE(g_bt_slowclk_tim, STM32_TIM_MODE_UP);
  STM32_TIM_SETPERIOD(g_bt_slowclk_tim, BT_SLOWCLK_ARR);
  STM32_TIM_SETCHANNEL(g_bt_slowclk_tim, BT_SLOWCLK_CH,
                       STM32_TIM_CH_OUTPWM);
  STM32_TIM_SETCOMPARE(g_bt_slowclk_tim, BT_SLOWCLK_CH, BT_SLOWCLK_CCR);
  STM32_TIM_ENABLE(g_bt_slowclk_tim);

  return OK;
}

#endif /* CONFIG_STM32_TIM8 */
