/****************************************************************************
 * boards/spike-prime-hub/src/stm32_adc_dma.c
 *
 * ADC1 DMA continuous conversion driver for SPIKE Prime Hub.
 * Uses TIM2 TRGO as external trigger, DMA2 Stream0 for data transfer.
 *
 * DMA and TIM2 are managed through NuttX abstraction APIs.  ADC1 itself
 * has no abstraction for multi-channel scan + DMA circular mode, so ADC
 * register access remains direct (using NuttX-provided offset/bit
 * constants from stm32_adc_v1.h).
 *
 * Channel configuration (same as pybricks):
 *   Rank 0: CH10 (PC0) - Battery current (IBAT)
 *   Rank 1: CH11 (PC1) - Battery voltage (VBAT)
 *   Rank 2: CH8  (PB0) - Battery temperature (NTC)
 *   Rank 3: CH3  (PA3) - USB charger current (IBUSBCH)
 *   Rank 4: CH14 (PC4) - Center button resistor ladder
 *   Rank 5: CH5  (PA1) - Left/Right/BT button resistor ladder
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_dma.h"
#include "stm32_tim.h"
#include "hardware/stm32_adc_v1.h"
#include "hardware/stm32_dma_v2.h"
#include "hardware/stm32_tim.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_ADC1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC channels and scan sequence */

#define ADC_NUM_CHANNELS      6

/* EXTSEL = 0110 = TIM2_TRGO, EXTEN = 01 = rising edge */

#define ADC_EXTSEL_TIM2TRGO   ADC_CR2_EXTSEL_T2TRGO

/* ADC clock prescaler: PCLK2/4 = 96/4 = 24 MHz */

#ifndef ADC_CCR_ADCPRE_DIV4
#  define ADC_CCR_ADCPRE_DIV4   (1 << 16)
#endif

/* Sampling time: 56 cycles = 0b011 */

#define SMPR_56CYCLES         3

/* DMA SCR bits for circular P2M, 16-bit, memory increment.
 * Channel selection is handled internally by stm32_dmachannel().
 */

#define ADC_DMA_SCR  (DMA_SCR_MSIZE_16BITS | \
                      DMA_SCR_PSIZE_16BITS | \
                      DMA_SCR_MINC         | \
                      DMA_SCR_CIRC)

/* TIM2 trigger rate: the timer input clock is divided to produce 1 kHz
 * update events (one complete ADC scan per millisecond).
 */

#define ADC_TIM2_TRIGGER_HZ   1000

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA buffer: one uint16_t per channel, continuously updated */

static volatile uint16_t g_adc_dma_buf[ADC_NUM_CHANNELS];

static DMA_HANDLE              g_dma;
static struct stm32_tim_dev_s *g_tim2;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_adc_dma_initialize
 *
 * Description:
 *   Initialize ADC1 with DMA2 and TIM2 trigger for continuous multi-channel
 *   conversion. After initialization, g_adc_dma_buf is continuously updated
 *   with the latest ADC values.
 ****************************************************************************/

int stm32_adc_dma_initialize(void)
{
  /* Enable ADC1 clock — no NuttX abstraction for our scan+DMA use case. */

  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_ADC1EN);

  /* Configure analog input GPIOs */

  stm32_configgpio(GPIO_ANALOG | GPIO_PORTC | GPIO_PIN0);  /* CH10 IBAT */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTC | GPIO_PIN1);  /* CH11 VBAT */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0);  /* CH8  NTC */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3);  /* CH3  IBUSBCH */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4);  /* CH14 BTN_CTR */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1);  /* CH5  BTN_LRB */

  /* ADC clock prescaler: PCLK2/4 = 96/4 = 24 MHz */

  modifyreg32(STM32_ADCCMN_BASE + STM32_ADC_CCR_OFFSET,
              3 << 16, ADC_CCR_ADCPRE_DIV4);

  /* ADC1 CR1: SCAN mode, 12-bit resolution */

  putreg32(ADC_CR1_SCAN | ADC_CR1_RES_12BIT,
           STM32_ADC1_BASE + STM32_ADC_CR1_OFFSET);

  /* ADC1 CR2: DMA, DDS (DMA for each conversion), external trigger TIM2 */

  putreg32(ADC_CR2_DMA | ADC_CR2_DDS |
           ADC_EXTSEL_TIM2TRGO | ADC_CR2_EXTEN_RISING | ADC_CR2_ADON,
           STM32_ADC1_BASE + STM32_ADC_CR2_OFFSET);

  /* Sampling time: 56 cycles for all channels
   * SMPR1 covers CH10-18, SMPR2 covers CH0-9.
   */

  putreg32((SMPR_56CYCLES << (0 * 3)) |  /* CH10 */
           (SMPR_56CYCLES << (1 * 3)) |  /* CH11 */
           (SMPR_56CYCLES << (4 * 3)),   /* CH14 */
           STM32_ADC1_BASE + STM32_ADC_SMPR1_OFFSET);

  putreg32((SMPR_56CYCLES << (3 * 3)) |  /* CH3 */
           (SMPR_56CYCLES << (5 * 3)) |  /* CH5 */
           (SMPR_56CYCLES << (8 * 3)),   /* CH8 */
           STM32_ADC1_BASE + STM32_ADC_SMPR2_OFFSET);

  /* Regular sequence: 6 conversions (L=5 in SQR1 bits [23:20])
   * SQR3 [4:0]=CH10, [9:5]=CH11, [14:10]=CH8, [19:15]=CH3, [24:20]=CH14
   * SQR2 [4:0]=CH5
   */

  putreg32((ADC_NUM_CHANNELS - 1) << 20,
           STM32_ADC1_BASE + STM32_ADC_SQR1_OFFSET);

  putreg32((10 << 0) | (11 << 5) | (8 << 10) | (3 << 15) | (14 << 20),
           STM32_ADC1_BASE + STM32_ADC_SQR3_OFFSET);

  putreg32(5 << 0,
           STM32_ADC1_BASE + STM32_ADC_SQR2_OFFSET);

  /* DMA2 Stream0 Channel0: peripheral=ADC1_DR, memory=g_adc_dma_buf,
   * circular mode, 16-bit, memory increment.
   */

  g_dma = stm32_dmachannel(DMAMAP_ADC1_1);
  DEBUGASSERT(g_dma != NULL);

  stm32_dmasetup(g_dma,
                 STM32_ADC1_BASE + STM32_ADC_DR_OFFSET,
                 (uint32_t)g_adc_dma_buf,
                 ADC_NUM_CHANNELS,
                 ADC_DMA_SCR);
  stm32_dmastart(g_dma, NULL, NULL, false);

  /* TIM2: trigger ADC at 1 kHz, TRGO on update.
   * stm32_tim_init() enables TIM2 clock.  SETCLOCK sets PSC for the
   * requested tick rate.  SETPERIOD(0) makes the timer overflow on every
   * tick.  TRGO master mode has no abstraction — direct CR2 write.
   */

  g_tim2 = stm32_tim_init(2);
  DEBUGASSERT(g_tim2 != NULL);

  /* SETCLOCK sets PSC for the requested tick rate and also starts the
   * timer.  We use the full 96 MHz input clock (PSC=0) and divide by
   * ARR+1 instead, because 96 MHz / 1 kHz = 96000 which overflows the
   * 16-bit PSC register.  TRGO master mode has no abstraction.
   */

  STM32_TIM_SETCLOCK(g_tim2, STM32_APB1_TIM2_CLKIN);
  STM32_TIM_DISABLE(g_tim2);

  modifyreg32(STM32_TIM2_CR2, GTIM_CR2_MMS_MASK, GTIM_CR2_MMS_UPDATE);

  STM32_TIM_SETPERIOD(g_tim2,
                       (STM32_APB1_TIM2_CLKIN / ADC_TIM2_TRIGGER_HZ) - 1);
  STM32_TIM_ENABLE(g_tim2);

  syslog(LOG_INFO, "ADC: DMA continuous conversion started "
         "(%d channels, TIM2 %d Hz trigger)\n",
         ADC_NUM_CHANNELS, ADC_TIM2_TRIGGER_HZ);
  return OK;
}

/****************************************************************************
 * Name: stm32_adc_read
 *
 * Description:
 *   Read the latest ADC value for a given rank (0-5) from the DMA buffer.
 ****************************************************************************/

uint16_t stm32_adc_read(uint8_t rank)
{
  DEBUGASSERT(rank < ADC_NUM_CHANNELS);
  return g_adc_dma_buf[rank];
}

#endif /* CONFIG_STM32_ADC1 */
