/****************************************************************************
 * boards/spike-prime-hub/src/stm32_cpuload.c
 *
 * TIM7-based CPU load sampling for TICKLESS mode.
 * Generates a periodic 100 Hz interrupt to drive
 * nxsched_process_cpuload_ticks().
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/arch.h>

#include "stm32_tim.h"
#include "hardware/stm32_tim.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPULOAD_TIM          7
#define CPULOAD_TIM_CLK_HZ   10000  /* Timer clock after prescaling */
#define CPULOAD_TIM_PERIOD   (CPULOAD_TIM_CLK_HZ / CONFIG_SCHED_CPULOAD_TICKSPERSEC - 1)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_tim_dev_s *g_cpuload_tim;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int stm32_cpuload_handler(int irq, void *context, void *arg)
{
  STM32_TIM_ACKINT(g_cpuload_tim, GTIM_DIER_UIE);
  nxsched_process_cpuload_ticks(1);
  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_cpuload_initialize(void)
{
  g_cpuload_tim = stm32_tim_init(CPULOAD_TIM);
  if (g_cpuload_tim == NULL)
    {
      return;
    }

  STM32_TIM_SETCLOCK(g_cpuload_tim, CPULOAD_TIM_CLK_HZ);
  STM32_TIM_SETPERIOD(g_cpuload_tim, CPULOAD_TIM_PERIOD);
  STM32_TIM_SETISR(g_cpuload_tim, stm32_cpuload_handler, NULL, 0);
  STM32_TIM_ENABLEINT(g_cpuload_tim, GTIM_DIER_UIE);
}
