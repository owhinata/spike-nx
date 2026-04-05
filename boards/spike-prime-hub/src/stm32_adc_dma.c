/****************************************************************************
 * boards/spike-prime-hub/src/stm32_adc_dma.c
 *
 * ADC1 DMA continuous conversion driver for SPIKE Prime Hub.
 * Uses TIM2 TRGO as external trigger, DMA2 Stream0 for data transfer.
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

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_ADC1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC1 registers */

#include "hardware/stm32_adc.h"

#define ADC1_BASE             STM32_ADC1_BASE
#define ADC_SQR1_OFFSET       0x2c
#define ADC_SQR3_OFFSET       0x34
#define ADC_DR_OFFSET         0x4c

/* ADC common */

#ifndef ADC_CCR_ADCPRE_DIV4
#  define ADC_CCR_ADCPRE_DIV4   (1 << 16)
#endif

/* DMA2 Stream0 registers */

#define DMA2_BASE             0x40026400
#define DMA_S0CR_OFFSET       0x10
#define DMA_S0NDTR_OFFSET     0x14
#define DMA_S0PAR_OFFSET      0x18
#define DMA_S0M0AR_OFFSET     0x1c

#define DMA_CR_EN             (1 << 0)
#define DMA_CR_CIRC           (1 << 8)
#define DMA_CR_MINC           (1 << 10)
#define DMA_CR_PSIZE_16       (1 << 11)
#define DMA_CR_MSIZE_16       (1 << 13)
#define DMA_CR_CHSEL_0        (0 << 25)  /* Channel 0 for ADC1 */

/* TIM2 registers */

#define TIM2_BASE             0x40000000
#define TIM_CR1_OFFSET        0x00
#define TIM_CR2_OFFSET        0x04
#define TIM_EGR_OFFSET        0x14
#define TIM_PSC_OFFSET        0x28
#define TIM_ARR_OFFSET        0x2c

#define TIM_CR1_CEN           (1 << 0)
#define TIM_CR2_MMS_UPDATE    (2 << 4)   /* TRGO on update event */
#define TIM_EGR_UG            (1 << 0)

/* TIM2 trigger rate: 96 MHz / 960 / 100 = 1 kHz (1ms per scan cycle) */

#define TIM2_PSC              959
#define TIM2_ARR              99

/* ADC channels and scan sequence */

#define ADC_NUM_CHANNELS      6

/* EXTSEL = 0110 = TIM2_TRGO, EXTEN = 01 = rising edge
 * Use NuttX definitions where available, define only what's missing.
 */

#ifndef ADC_CR2_EXTSEL_TIM2
#  define ADC_CR2_EXTSEL_TIM2   (6 << 24)
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* DMA buffer: one uint16_t per channel, continuously updated */

static volatile uint16_t g_adc_dma_buf[ADC_NUM_CHANNELS];

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
  /* Enable clocks: ADC1 (APB2), DMA2 (AHB1), TIM2 (APB1) */

  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_ADC1EN);
  modifyreg32(STM32_RCC_AHB1ENR, 0, RCC_AHB1ENR_DMA2EN);
  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM2EN);

  /* Configure analog input GPIOs */

  stm32_configgpio(GPIO_ANALOG | GPIO_PORTC | GPIO_PIN0);  /* CH10 IBAT */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTC | GPIO_PIN1);  /* CH11 VBAT */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTB | GPIO_PIN0);  /* CH8  NTC */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTA | GPIO_PIN3);  /* CH3  IBUSBCH */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4);  /* CH14 BTN_CTR */
  stm32_configgpio(GPIO_ANALOG | GPIO_PORTA | GPIO_PIN1);  /* CH5  BTN_LRB */

  /* ADC clock prescaler: PCLK2/4 = 96/4 = 24 MHz */

  modifyreg32(STM32_ADC_CCR, 3 << 16, ADC_CCR_ADCPRE_DIV4);

  /* ADC1 CR1: SCAN mode, 12-bit resolution */

  putreg32(ADC_CR1_SCAN | ADC_CR1_RES_12BIT,
           ADC1_BASE + STM32_ADC_CR1_OFFSET);

  /* ADC1 CR2: DMA, DDS (DMA for each conversion), external trigger TIM2 */

  putreg32(ADC_CR2_DMA | ADC_CR2_DDS |
           ADC_CR2_EXTSEL_TIM2 | ADC_CR2_EXTEN_RISING | ADC_CR2_ADON,
           ADC1_BASE + STM32_ADC_CR2_OFFSET);

  /* Sampling time: 56 cycles for all channels (SMPR1 for CH10-18, SMPR2
   * for CH0-9).
   * 56 cycles = 0b011
   */

  putreg32((3 << (0 * 3)) |  /* CH10 */
           (3 << (1 * 3)) |  /* CH11 */
           (3 << (4 * 3)),   /* CH14 */
           ADC1_BASE + STM32_ADC_SMPR1_OFFSET);

  putreg32((3 << (3 * 3)) |  /* CH3 */
           (3 << (5 * 3)) |  /* CH5 */
           (3 << (8 * 3)),   /* CH8 */
           ADC1_BASE + STM32_ADC_SMPR2_OFFSET);

  /* Regular sequence: 6 conversions (L=5 in SQR1 bits [23:20])
   * SQR3 [4:0]=CH10, [9:5]=CH11, [14:10]=CH8, [19:15]=CH3, [24:20]=CH14
   * SQR2 [4:0]=CH5
   */

  putreg32((ADC_NUM_CHANNELS - 1) << 20,
           ADC1_BASE + ADC_SQR1_OFFSET);

  putreg32((10 << 0) | (11 << 5) | (8 << 10) | (3 << 15) | (14 << 20),
           ADC1_BASE + ADC_SQR3_OFFSET);

  putreg32(5 << 0,
           ADC1_BASE + 0x30);  /* SQR2 offset */

  /* DMA2 Stream0 Channel0: peripheral=ADC1_DR, memory=g_adc_dma_buf,
   * circular mode, 16-bit, memory increment
   */

  putreg32(0, DMA2_BASE + DMA_S0CR_OFFSET);  /* Disable first */
  while (getreg32(DMA2_BASE + DMA_S0CR_OFFSET) & DMA_CR_EN);

  putreg32(ADC1_BASE + ADC_DR_OFFSET, DMA2_BASE + DMA_S0PAR_OFFSET);
  putreg32((uint32_t)g_adc_dma_buf, DMA2_BASE + DMA_S0M0AR_OFFSET);
  putreg32(ADC_NUM_CHANNELS, DMA2_BASE + DMA_S0NDTR_OFFSET);

  putreg32(DMA_CR_CHSEL_0 | DMA_CR_CIRC | DMA_CR_MINC |
           DMA_CR_PSIZE_16 | DMA_CR_MSIZE_16 | DMA_CR_EN,
           DMA2_BASE + DMA_S0CR_OFFSET);

  /* TIM2: trigger ADC at 1 kHz, TRGO on update */

  putreg32(TIM2_PSC, TIM2_BASE + TIM_PSC_OFFSET);
  putreg32(TIM2_ARR, TIM2_BASE + TIM_ARR_OFFSET);
  putreg32(TIM_CR2_MMS_UPDATE, TIM2_BASE + TIM_CR2_OFFSET);
  putreg32(TIM_EGR_UG, TIM2_BASE + TIM_EGR_OFFSET);
  putreg32(TIM_CR1_CEN, TIM2_BASE + TIM_CR1_OFFSET);

  syslog(LOG_INFO, "ADC: DMA continuous conversion started "
         "(%d channels, TIM2 1kHz trigger)\n", ADC_NUM_CHANNELS);
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
