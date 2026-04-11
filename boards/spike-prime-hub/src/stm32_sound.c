/****************************************************************************
 * boards/spike-prime-hub/src/stm32_sound.c
 *
 * Low-level PCM audio for SPIKE Prime Hub.
 *   DAC1 CH1 (PA4) -> amp enable (PC10) -> speaker
 *   DMA1 Stream5 Channel7 feeds DAC1_DHR12L1 on TIM6 TRGO
 *
 * The STM32F413 Kconfig in upstream NuttX does not "select STM32_HAVE_DAC1",
 * so CONFIG_STM32_DAC1 is not available.  Likewise stm32f413xx_pinmap.h has
 * no DAC1 OUT1 macro.  This driver therefore:
 *   - enables the DAC1/TIM6/DMA1 clocks via RCC directly
 *   - uses a board-local PA4 analog pinmux definition
 *
 * The caller is expected to hold g_sound.lock around play_pcm/stop_pcm and
 * set_volume.  stop_pcm() is idempotent; calling it on a stopped channel is
 * a no-op.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <debug.h>
#include <errno.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include <nuttx/irq.h>
#include <nuttx/signal.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32.h"
#include "hardware/stm32_dac_v1.h"
#include "hardware/stm32_tim.h"
#include "hardware/stm32_dma_v2.h"

#include "spike_prime_hub.h"
#include "stm32_sound.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* DMA1 Stream 5 feeds DAC1 CH1 using channel 7 (see RM0430 Table 27). */

#define SOUND_DMA_CHANNEL     7

/* TIM6 is on APB1; STM32F4 doubles the APB1 timer clock when the prescaler
 * is not 1.  The SPIKE board.h declares STM32_APB1_TIM6_CLKIN = 96 MHz.
 */

#define SOUND_TIM6_CLKIN      STM32_APB1_TIM6_CLKIN  /* 96 MHz */

/* Settling delay (microseconds) between enabling the DAC output and raising
 * the amplifier enable pin.  Gives the DAC time to reach its midpoint so the
 * speaker does not pop.
 */

#define SOUND_STARTUP_SETTLE_US  2000

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct sound_state_s g_sound =
{
  .lock       = NXMUTEX_INITIALIZER,
  .open_count = 0,
  .owner      = NULL,
  .mode       = SOUND_MODE_IDLE,
  .volume     = 100,
  .stop_flag  = ATOMIC_VAR_INIT(false),
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Disable DMA1 Stream 5 and wait for the hardware to acknowledge. */

static void sound_dma_disable_and_wait(void)
{
  uint32_t scr = getreg32(STM32_DMA1_S5CR);
  if ((scr & DMA_SCR_EN) != 0)
    {
      putreg32(scr & ~DMA_SCR_EN, STM32_DMA1_S5CR);
      while ((getreg32(STM32_DMA1_S5CR) & DMA_SCR_EN) != 0)
        {
          /* spin until the hardware clears EN */
        }
    }

  /* Clear the high interrupt flags for stream 5. */

  putreg32(DMA_INT_STREAM5_MASK, STM32_DMA1_HIFCR);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_sound_initialize(void)
{
  /* Enable peripheral clocks. */

  modifyreg32(STM32_RCC_APB1ENR, 0,
              RCC_APB1ENR_DAC1EN | RCC_APB1ENR_TIM6EN);
  modifyreg32(STM32_RCC_AHB1ENR, 0, RCC_AHB1ENR_DMA1EN);

  /* Configure GPIOs.  PA4 analog for DAC1 OUT1, PC10 push-pull low for
   * the amplifier enable (speaker muted at boot).
   */

  stm32_configgpio(GPIO_DAC1_OUT1_0);
  stm32_configgpio(GPIO_AMP_EN);
  stm32_gpiowrite(GPIO_AMP_EN, false);

  /* Route TIM6 update events to the DAC trigger line (TRGO = UPDATE). */

  modifyreg32(STM32_TIM6_CR2, GTIM_CR2_MMS_MASK, GTIM_CR2_MMS_UPDATE);

  /* Baseline hardware state: DAC and DMA off, timer stopped. */

  putreg32(0, STM32_DAC1_CR);
  sound_dma_disable_and_wait();
  putreg32(0, STM32_TIM6_CR1);

  syslog(LOG_INFO, "sound: initialized (DAC1 PA4, amp PC10, TIM6)\n");
  return OK;
}

int stm32_sound_play_pcm(FAR const uint16_t *data,
                         uint32_t length, uint32_t sample_rate)
{
  if (data == NULL || length < 2)
    {
      return -EINVAL;
    }

  if (sample_rate < 1000 || sample_rate > 100000)
    {
      return -EINVAL;
    }

  irqstate_t flags = enter_critical_section();

  /* Stop the timer first so no further samples are latched. */

  modifyreg32(STM32_TIM6_CR1, GTIM_CR1_CEN, 0);

  /* Reconfigure DMA1 Stream 5. */

  sound_dma_disable_and_wait();
  putreg32(STM32_DAC1_DHR12L1, STM32_DMA1_S5PAR);
  putreg32((uint32_t)data, STM32_DMA1_S5M0AR);
  putreg32(length, STM32_DMA1_S5NDTR);
  putreg32(DMA_SCR_CHSEL(SOUND_DMA_CHANNEL) |
           DMA_SCR_PRIHI |
           DMA_SCR_MSIZE_16BITS |
           DMA_SCR_PSIZE_16BITS |
           DMA_SCR_MINC |
           DMA_SCR_CIRC |
           DMA_SCR_DIR_M2P |
           DMA_SCR_EN,
           STM32_DMA1_S5CR);

  /* Enable DAC1 CH1 with DMA + TIM6 trigger. */

  putreg32(DAC_CR_DMAEN1 | DAC_CR_TSEL1_TIM6 | DAC_CR_TEN1 | DAC_CR_EN1,
           STM32_DAC1_CR);

  /* Program TIM6 for the requested sample rate and start it. */

  putreg32(0, STM32_TIM6_PSC);
  putreg32((SOUND_TIM6_CLKIN / sample_rate) - 1, STM32_TIM6_ARR);
  putreg32(GTIM_EGR_UG, STM32_TIM6_EGR);
  modifyreg32(STM32_TIM6_CR1, 0, GTIM_CR1_CEN);

  leave_critical_section(flags);

  /* Let the DAC output reach its midpoint before unmuting the amplifier. */

  nxsig_usleep(SOUND_STARTUP_SETTLE_US);
  stm32_gpiowrite(GPIO_AMP_EN, true);

  return OK;
}

int stm32_sound_stop_pcm(void)
{
  /* Mute the speaker first to avoid audible pops. */

  stm32_gpiowrite(GPIO_AMP_EN, false);

  irqstate_t flags = enter_critical_section();

  modifyreg32(STM32_TIM6_CR1, GTIM_CR1_CEN, 0);
  sound_dma_disable_and_wait();
  putreg32(0, STM32_DAC1_CR);

  leave_critical_section(flags);
  return OK;
}

void stm32_sound_set_volume(uint8_t level)
{
  if (level > 100)
    {
      level = 100;
    }

  g_sound.volume = level;
}

uint8_t stm32_sound_get_volume(void)
{
  return g_sound.volume;
}
