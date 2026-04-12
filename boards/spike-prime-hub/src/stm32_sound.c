/****************************************************************************
 * boards/spike-prime-hub/src/stm32_sound.c
 *
 * Low-level PCM audio for SPIKE Prime Hub.
 *   DAC1 CH1 (PA4) -> amp enable (PC10) -> speaker
 *   DMA1 Stream5 Channel7 feeds DAC1_DHR12L1 on TIM6 TRGO
 *
 * The DMA stream and TIM6 are managed through NuttX abstraction APIs
 * (stm32_dma*.h / stm32_tim.h).  DAC1 itself and the TIM6 TRGO master
 * mode have no corresponding abstraction in the current NuttX tree, so
 * those two small pieces remain as direct register access.
 *
 * The caller is expected to hold g_sound.lock around play_pcm/stop_pcm
 * and set_volume.  stop_pcm() is idempotent; calling it on a stopped
 * channel is a no-op.
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
#include "stm32_dma.h"
#include "stm32_tim.h"
#include "hardware/stm32_dac_v1.h"
#include "hardware/stm32_dma_v2.h"
#include "hardware/stm32_tim.h"

#include "spike_prime_hub.h"
#include "stm32_sound.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* TIM6 is on APB1; STM32F4 doubles the APB1 timer clock when the prescaler
 * is not 1.  The SPIKE board.h declares STM32_APB1_TIM6_CLKIN = 96 MHz.
 */

#define SOUND_TIM6_CLKIN      STM32_APB1_TIM6_CLKIN  /* 96 MHz */

/* Settling delay (microseconds) between enabling the DAC output and raising
 * the amplifier enable pin.  Gives the DAC time to reach its midpoint so the
 * speaker does not pop.
 */

#define SOUND_STARTUP_SETTLE_US  2000

/* DMA SCR bits shared by every play_pcm() call. Channel selection is
 * handled internally by stm32_dmachannel(DMAMAP_DAC1_CH1).
 */

#define SOUND_DMA_SCR  (DMA_SCR_PRIHI      | \
                        DMA_SCR_MSIZE_16BITS | \
                        DMA_SCR_PSIZE_16BITS | \
                        DMA_SCR_MINC         | \
                        DMA_SCR_CIRC         | \
                        DMA_SCR_DIR_M2P)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static DMA_HANDLE                g_dma;
static struct stm32_tim_dev_s   *g_tim6;

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
 * Public Functions
 ****************************************************************************/

int stm32_sound_initialize(void)
{
  /* DAC1 clock — the only manual RCC enable left. DMA1 and TIM6 clocks
   * are enabled internally by stm32_dmachannel() and stm32_tim_init().
   */

  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_DAC1EN);

  /* GPIOs: PA4 analog for DAC1 OUT1, PC10 push-pull low for the
   * amplifier enable (speaker muted at boot).
   */

  stm32_configgpio(GPIO_DAC1_OUT1_0);
  stm32_configgpio(GPIO_AMP_EN);
  stm32_gpiowrite(GPIO_AMP_EN, false);

  /* Allocate DMA1 Stream5/Channel7 for DAC1 CH1 (RM0430 Table 27). */

  g_dma = stm32_dmachannel(DMAMAP_DAC1_CH1);
  DEBUGASSERT(g_dma != NULL);

  /* TIM6: basic up-counter, full-speed prescaler (PSC=0). */

  g_tim6 = stm32_tim_init(6);
  DEBUGASSERT(g_tim6 != NULL);

  STM32_TIM_SETMODE(g_tim6, STM32_TIM_MODE_UP);
  STM32_TIM_SETCLOCK(g_tim6, SOUND_TIM6_CLKIN);
  STM32_TIM_DISABLE(g_tim6);

  /* Route TIM6 update events to the DAC trigger line (TRGO = UPDATE).
   * No abstraction exists for master mode configuration.
   */

  modifyreg32(STM32_TIM6_CR2, GTIM_CR2_MMS_MASK, GTIM_CR2_MMS_UPDATE);

  /* Baseline hardware state: DAC off, DMA stopped. */

  putreg32(0, STM32_DAC1_CR);
  stm32_dmastop(g_dma);

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

  /* Stop the timer so no further samples are latched. */

  STM32_TIM_DISABLE(g_tim6);

  /* Reconfigure DMA1 Stream5: memory-to-peripheral, 16-bit, circular. */

  stm32_dmastop(g_dma);
  stm32_dmasetup(g_dma,
                 STM32_DAC1_DHR12L1,
                 (uint32_t)data,
                 length,
                 SOUND_DMA_SCR);
  stm32_dmastart(g_dma, NULL, NULL, false);

  /* Enable DAC1 CH1 with DMA + TIM6 trigger. */

  putreg32(DAC_CR_DMAEN1 | DAC_CR_TSEL1_TIM6 | DAC_CR_TEN1 | DAC_CR_EN1,
           STM32_DAC1_CR);

  /* Program TIM6 period for the requested sample rate and start.
   * PSC is already 0 (set once in initialize); only ARR changes.
   * STM32_TIM_ENABLE generates UG (loads ARR shadow) + sets CEN.
   */

  STM32_TIM_SETPERIOD(g_tim6, (SOUND_TIM6_CLKIN / sample_rate) - 1);
  STM32_TIM_ENABLE(g_tim6);

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

  STM32_TIM_DISABLE(g_tim6);
  stm32_dmastop(g_dma);
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
