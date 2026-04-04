/****************************************************************************
 * boards/spike-prime-hub/src/stm32_tlc5955.c
 *
 * TLC5955 48-channel PWM LED driver for SPIKE Prime Hub.
 * Ported from pybricks pwm_tlc5955_stm32.c (MIT license).
 *
 * Hardware:
 *   SPI1: MOSI=PA7, MISO=PA6, SCK=PA5 (AF5), 24 MHz
 *   LAT:  PA15 (GPIO output, latch on HIGH->LOW edge)
 *   GSCLK: TIM12 CH2 = PB15, 9.6 MHz PWM
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_SPI1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size in bytes for 769-bit shift register */

#define TLC5955_DATA_SIZE     ((769 + 7) / 8)  /* 97 bytes */

/* Number of PWM channels */

#define TLC5955_NUM_CH        48

/* TIM12 base address and registers for GSCLK generation */

#define STM32_TIM12_BASE      0x40001800
#define TIM_CR1_OFFSET        0x00
#define TIM_CCMR1_OFFSET      0x18
#define TIM_CCER_OFFSET       0x20
#define TIM_PSC_OFFSET        0x28
#define TIM_ARR_OFFSET        0x2c
#define TIM_CCR2_OFFSET       0x38

#define TIM_CR1_CEN           (1 << 0)
#define TIM_CCMR1_OC2M_PWM1  (6 << 12)
#define TIM_CCMR1_OC2PE       (1 << 11)
#define TIM_CCER_CC2E         (1 << 4)

/* GSCLK frequency: APB1 clock (96 MHz with x2) / (PSC+1) / (ARR+1)
 * 96 MHz / 1 / 10 = 9.6 MHz
 */

#define TIM12_PSC             0
#define TIM12_ARR             10  /* period=10, same as pybricks */
#define TIM12_CCR2            5   /* 50% duty */

#define TIM_EGR_OFFSET        0x14
#define TIM_EGR_UG            (1 << 0)

/* PB15 AF9 = TIM12_CH2 (GSCLK output) */

#define GPIO_GSCLK            (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | \
                               GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN15)

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* TLC5955 maximum current values (3-bit) */

enum tlc5955_mc_e
{
  TLC5955_MC_3_2  = 0,   /* 3.2 mA */
  TLC5955_MC_8_0  = 1,   /* 8.0 mA */
  TLC5955_MC_11_2 = 2,   /* 11.2 mA */
  TLC5955_MC_15_9 = 3,   /* 15.9 mA */
  TLC5955_MC_19_1 = 4,   /* 19.1 mA */
  TLC5955_MC_23_9 = 5,   /* 23.9 mA */
  TLC5955_MC_27_1 = 6,   /* 27.1 mA */
  TLC5955_MC_31_9 = 7,   /* 31.9 mA */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct spi_dev_s *g_spi;

/* Grayscale latch buffer.
 * Channels are mapped in reverse order:
 *   CH0 -> bytes [1..2] (GSB15)
 *   CH1 -> bytes [3..4] (GSG15)
 *   ...
 *   CH47 -> bytes [95..96] (GSR0)
 * Byte [0] is always 0 (bit 768 = 0 for GS latch).
 */

static uint8_t g_grayscale[TLC5955_DATA_SIZE];

/* Control latch data: DC=127, MC=3.2mA, BC=127,
 * DSPRPT=1, TMGRST=0, RFRESH=0, ESPWM=1, LSDVLT=1
 *
 * Ported directly from pybricks TLC5955_CONTROL_DATA macro.
 */

#define DC  127
#define MC  TLC5955_MC_3_2
#define BC  127

static const uint8_t g_control_latch[TLC5955_DATA_SIZE] =
{
  /* bit 768 */      [0]  = 1,
  /* bits 767-760 */ [1]  = 0x96,
  /* bits 370-368 */ [50] = ((1 << 2) | (1 << 1) | 0) & 0xff,
  /* bits 367-360 */ [51] = ((0 << 7) | (1 << 6) | (BC >> 1)) & 0xff,
  /* bits 359-352 */ [52] = ((BC << 7) | (BC >> 0)) & 0xff,
  /* bits 351-344 */ [53] = ((BC << 1) | (MC >> 2)) & 0xff,
  /* bits 343-336 */ [54] = ((MC << 6) | (MC << 3) | MC) & 0xff,
  /* bits 335-328 */ [55] = ((DC << 1) | (DC >> 6)) & 0xff,
  /* bits 327-320 */ [56] = ((DC << 2) | (DC >> 5)) & 0xff,
  /* bits 319-312 */ [57] = ((DC << 3) | (DC >> 4)) & 0xff,
  /* bits 311-304 */ [58] = ((DC << 4) | (DC >> 3)) & 0xff,
  /* bits 303-296 */ [59] = ((DC << 5) | (DC >> 2)) & 0xff,
  /* bits 295-288 */ [60] = ((DC << 6) | (DC >> 1)) & 0xff,
  /* bits 287-280 */ [61] = ((DC << 7) | (DC >> 0)) & 0xff,
  /* bits 279-272 */ [62] = ((DC << 1) | (DC >> 6)) & 0xff,
  /* bits 271-264 */ [63] = ((DC << 2) | (DC >> 5)) & 0xff,
  /* bits 263-256 */ [64] = ((DC << 3) | (DC >> 4)) & 0xff,
  /* bits 255-248 */ [65] = ((DC << 4) | (DC >> 3)) & 0xff,
  /* bits 247-240 */ [66] = ((DC << 5) | (DC >> 2)) & 0xff,
  /* bits 239-232 */ [67] = ((DC << 6) | (DC >> 1)) & 0xff,
  /* bits 231-224 */ [68] = ((DC << 7) | (DC >> 0)) & 0xff,
  /* bits 223-216 */ [69] = ((DC << 1) | (DC >> 6)) & 0xff,
  /* bits 215-208 */ [70] = ((DC << 2) | (DC >> 5)) & 0xff,
  /* bits 207-200 */ [71] = ((DC << 3) | (DC >> 4)) & 0xff,
  /* bits 199-192 */ [72] = ((DC << 4) | (DC >> 3)) & 0xff,
  /* bits 191-184 */ [73] = ((DC << 5) | (DC >> 2)) & 0xff,
  /* bits 183-176 */ [74] = ((DC << 6) | (DC >> 1)) & 0xff,
  /* bits 175-168 */ [75] = ((DC << 7) | (DC >> 0)) & 0xff,
  /* bits 167-160 */ [76] = ((DC << 1) | (DC >> 6)) & 0xff,
  /* bits 159-152 */ [77] = ((DC << 2) | (DC >> 5)) & 0xff,
  /* bits 151-144 */ [78] = ((DC << 3) | (DC >> 4)) & 0xff,
  /* bits 143-136 */ [79] = ((DC << 4) | (DC >> 3)) & 0xff,
  /* bits 135-128 */ [80] = ((DC << 5) | (DC >> 2)) & 0xff,
  /* bits 127-120 */ [81] = ((DC << 6) | (DC >> 1)) & 0xff,
  /* bits 119-112 */ [82] = ((DC << 7) | (DC >> 0)) & 0xff,
  /* bits 111-104 */ [83] = ((DC << 1) | (DC >> 6)) & 0xff,
  /* bits 103-96 */  [84] = ((DC << 2) | (DC >> 5)) & 0xff,
  /* bits 95-88 */   [85] = ((DC << 3) | (DC >> 4)) & 0xff,
  /* bits 87-80 */   [86] = ((DC << 4) | (DC >> 3)) & 0xff,
  /* bits 79-72 */   [87] = ((DC << 5) | (DC >> 2)) & 0xff,
  /* bits 71-64 */   [88] = ((DC << 6) | (DC >> 1)) & 0xff,
  /* bits 63-56 */   [89] = ((DC << 7) | (DC >> 0)) & 0xff,
  /* bits 55-48 */   [90] = ((DC << 1) | (DC >> 6)) & 0xff,
  /* bits 47-40 */   [91] = ((DC << 2) | (DC >> 5)) & 0xff,
  /* bits 39-32 */   [92] = ((DC << 3) | (DC >> 4)) & 0xff,
  /* bits 31-24 */   [93] = ((DC << 4) | (DC >> 3)) & 0xff,
  /* bits 23-16 */   [94] = ((DC << 5) | (DC >> 2)) & 0xff,
  /* bits 15-8 */    [95] = ((DC << 6) | (DC >> 1)) & 0xff,
  /* bits 7-0 */     [96] = ((DC << 7) | (DC >> 0)) & 0xff,
};

#undef DC
#undef MC
#undef BC

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: tlc5955_toggle_latch
 *
 * Description:
 *   Toggle LAT pin HIGH then LOW to latch shift register data.
 ****************************************************************************/

static void tlc5955_toggle_latch(void)
{
  volatile int i;

  stm32_gpiowrite(GPIO_TLC5955_LAT, true);
  for (i = 0; i < 100; i++);  /* Short delay for LAT pulse width */
  stm32_gpiowrite(GPIO_TLC5955_LAT, false);
}

/****************************************************************************
 * Name: tlc5955_spi_send
 *
 * Description:
 *   Send data buffer via SPI and toggle latch.
 ****************************************************************************/

static void tlc5955_spi_send(FAR const uint8_t *data, size_t len)
{
  size_t i;

  SPI_LOCK(g_spi, true);

  for (i = 0; i < len; i++)
    {
      SPI_SEND(g_spi, data[i]);
    }

  SPI_LOCK(g_spi, false);

  /* Wait for SPI shift register to finish clocking out.
   * SPI1 base = 0x40013000, SR offset = 0x08, BSY = bit 7
   */

  while (getreg16(0x40013000 + 0x08) & (1 << 7));

  tlc5955_toggle_latch();
}

/****************************************************************************
 * Name: tlc5955_gsclk_start
 *
 * Description:
 *   Start TIM12 CH2 (PB15) as 9.6 MHz GSCLK for the TLC5955.
 *   Direct register access since NuttX PWM framework is overkill here.
 ****************************************************************************/

static void tlc5955_gsclk_start(void)
{
  /* Enable TIM12 clock (APB1) */

  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM12EN);

  /* Configure PB15 as TIM12_CH2 (AF9) */

  stm32_configgpio(GPIO_GSCLK);

  /* Set prescaler and auto-reload for 9.6 MHz output */

  putreg16(TIM12_PSC, STM32_TIM12_BASE + TIM_PSC_OFFSET);
  putreg16(TIM12_ARR, STM32_TIM12_BASE + TIM_ARR_OFFSET);

  /* Configure CH2 as PWM mode 1 with preload */

  putreg16(TIM_CCMR1_OC2M_PWM1 | TIM_CCMR1_OC2PE,
           STM32_TIM12_BASE + TIM_CCMR1_OFFSET);

  /* Set 50% duty cycle */

  putreg16(TIM12_CCR2, STM32_TIM12_BASE + TIM_CCR2_OFFSET);

  /* Enable CH2 output */

  putreg16(TIM_CCER_CC2E, STM32_TIM12_BASE + TIM_CCER_OFFSET);

  /* Start timer and force update event to load shadow registers */

  putreg16(TIM_CR1_CEN, STM32_TIM12_BASE + TIM_CR1_OFFSET);
  putreg16(TIM_EGR_UG, STM32_TIM12_BASE + TIM_EGR_OFFSET);

}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_spi1select / stm32_spi1status
 *
 * Description:
 *   Board-level SPI1 chip select and status callbacks required by NuttX
 *   SPI driver. TLC5955 uses LAT pin instead of CS, so these are no-ops.
 ****************************************************************************/

void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
  /* TLC5955 uses LAT pin for latch, not chip select */
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

/****************************************************************************
 * Name: tlc5955_initialize
 *
 * Description:
 *   Initialize the TLC5955 LED driver.
 *   Sets up SPI1, GSCLK (TIM12), and sends control latch data twice
 *   (hardware requirement for max current to take effect).
 *
 * Returned Value:
 *   OK on success, negative errno on failure.
 ****************************************************************************/

int tlc5955_initialize(void)
{
  /* Initialize LAT GPIO (low) */

  stm32_configgpio(GPIO_TLC5955_LAT);

  /* Get SPI1 device */

  g_spi = stm32_spibus_initialize(1);
  if (g_spi == NULL)
    {
      syslog(LOG_ERR, "TLC5955: Failed to initialize SPI1\n");
      return -ENODEV;
    }

  /* Configure SPI: Mode 0, MSB first, 8-bit, ~24 MHz */

  SPI_LOCK(g_spi, true);
  SPI_SETMODE(g_spi, SPIDEV_MODE0);
  SPI_SETBITS(g_spi, 8);

  SPI_SETFREQUENCY(g_spi, 24000000);
  SPI_LOCK(g_spi, false);

  /* Start GSCLK (TIM12 CH2, 9.6 MHz on PB15) */

  tlc5955_gsclk_start();

  /* Send control latch twice (TLC5955 requires this for max current) */

  tlc5955_spi_send(g_control_latch, TLC5955_DATA_SIZE);
  tlc5955_spi_send(g_control_latch, TLC5955_DATA_SIZE);

  /* Clear grayscale buffer */

  memset(g_grayscale, 0, TLC5955_DATA_SIZE);

  syslog(LOG_INFO, "TLC5955: Initialized (48ch LED driver)\n");
  return OK;
}

/****************************************************************************
 * Name: tlc5955_set_duty
 *
 * Description:
 *   Set the 16-bit PWM duty cycle for a single channel.
 *   Call tlc5955_update() to actually send data to the device.
 *
 * Input Parameters:
 *   ch    - Channel number (0-47)
 *   value - 16-bit PWM value (0=off, 0xFFFF=full on)
 ****************************************************************************/

void tlc5955_set_duty(uint8_t ch, uint16_t value)
{
  DEBUGASSERT(ch < TLC5955_NUM_CH);

  g_grayscale[ch * 2 + 1] = value >> 8;
  g_grayscale[ch * 2 + 2] = value & 0xff;
}

/****************************************************************************
 * Name: tlc5955_update
 *
 * Description:
 *   Send the grayscale buffer to the TLC5955 via SPI and latch.
 *
 * Returned Value:
 *   OK on success.
 ****************************************************************************/

int tlc5955_update(void)
{
  /* Byte [0] must be 0 for grayscale latch (bit 768 = 0) */

  g_grayscale[0] = 0;

  tlc5955_spi_send(g_grayscale, TLC5955_DATA_SIZE);
  return OK;
}

#endif /* CONFIG_STM32_SPI1 */
