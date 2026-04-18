/****************************************************************************
 * boards/spike-prime-hub/src/stm32_tlc5955.c
 *
 * TLC5955 48-channel PWM LED driver for SPIKE Prime Hub.
 * Ported from pybricks pwm_tlc5955_stm32.c (MIT license).
 *
 * Hardware:
 *   SPI1: MOSI=PA7, MISO=PA6, SCK=PA5 (AF5), 24 MHz (DMA)
 *   LAT:  PA15 (GPIO output, latch on HIGH->LOW edge)
 *   GSCLK: TIM12 CH2 = PB15, ~8.7 MHz PWM
 *
 * Update method:
 *   tlc5955_set_duty() sets data and marks changed.
 *   Deferred SPI transfer is scheduled on HPWORK queue.
 *   Multiple set_duty calls are batched into one SPI transfer.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>

#include "stm32.h"
#include "stm32_spi.h"
#include "stm32_tim.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_SPI1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Size in bytes for 769-bit shift register */

#define TLC5955_DATA_SIZE     ((769 + 7) / 8)  /* 97 bytes */

/* Number of PWM channels */

#define TLC5955_NUM_CH        48

/* GSCLK: TIM12 CH2 at full APB1 timer clock / (ARR+1) = 96 MHz / 11.
 * PB15 AF9 = TIM12_CH2.
 */

#define GSCLK_TIM             12
#define GSCLK_CH              2
#define GSCLK_ARR             10
#define GSCLK_CCR             5

#define GPIO_GSCLK            (GPIO_ALT | GPIO_AF9 | GPIO_SPEED_50MHz | \
                               GPIO_PUSHPULL | GPIO_PORTB | GPIO_PIN15)

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum tlc5955_mc_e
{
  TLC5955_MC_3_2  = 0,
  TLC5955_MC_8_0  = 1,
  TLC5955_MC_11_2 = 2,
  TLC5955_MC_15_9 = 3,
  TLC5955_MC_19_1 = 4,
  TLC5955_MC_23_9 = 5,
  TLC5955_MC_27_1 = 6,
  TLC5955_MC_31_9 = 7,
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct spi_dev_s *g_spi;
static uint8_t g_grayscale[TLC5955_DATA_SIZE];
static bool g_changed;
static struct work_s g_update_work;

/* Control latch: DC=127, MC=3.2mA, BC=127, DSPRPT=1, ESPWM=1, LSDVLT=1 */

#define DC  127
#define MC  TLC5955_MC_3_2
#define BC  127

static const uint8_t g_control_latch[TLC5955_DATA_SIZE] =
{
  [0]  = 1,
  [1]  = 0x96,
  [50] = ((1 << 2) | (1 << 1) | 0) & 0xff,
  [51] = ((0 << 7) | (1 << 6) | (BC >> 1)) & 0xff,
  [52] = ((BC << 7) | (BC >> 0)) & 0xff,
  [53] = ((BC << 1) | (MC >> 2)) & 0xff,
  [54] = ((MC << 6) | (MC << 3) | MC) & 0xff,
  [55] = ((DC << 1) | (DC >> 6)) & 0xff,
  [56] = ((DC << 2) | (DC >> 5)) & 0xff,
  [57] = ((DC << 3) | (DC >> 4)) & 0xff,
  [58] = ((DC << 4) | (DC >> 3)) & 0xff,
  [59] = ((DC << 5) | (DC >> 2)) & 0xff,
  [60] = ((DC << 6) | (DC >> 1)) & 0xff,
  [61] = ((DC << 7) | (DC >> 0)) & 0xff,
  [62] = ((DC << 1) | (DC >> 6)) & 0xff,
  [63] = ((DC << 2) | (DC >> 5)) & 0xff,
  [64] = ((DC << 3) | (DC >> 4)) & 0xff,
  [65] = ((DC << 4) | (DC >> 3)) & 0xff,
  [66] = ((DC << 5) | (DC >> 2)) & 0xff,
  [67] = ((DC << 6) | (DC >> 1)) & 0xff,
  [68] = ((DC << 7) | (DC >> 0)) & 0xff,
  [69] = ((DC << 1) | (DC >> 6)) & 0xff,
  [70] = ((DC << 2) | (DC >> 5)) & 0xff,
  [71] = ((DC << 3) | (DC >> 4)) & 0xff,
  [72] = ((DC << 4) | (DC >> 3)) & 0xff,
  [73] = ((DC << 5) | (DC >> 2)) & 0xff,
  [74] = ((DC << 6) | (DC >> 1)) & 0xff,
  [75] = ((DC << 7) | (DC >> 0)) & 0xff,
  [76] = ((DC << 1) | (DC >> 6)) & 0xff,
  [77] = ((DC << 2) | (DC >> 5)) & 0xff,
  [78] = ((DC << 3) | (DC >> 4)) & 0xff,
  [79] = ((DC << 4) | (DC >> 3)) & 0xff,
  [80] = ((DC << 5) | (DC >> 2)) & 0xff,
  [81] = ((DC << 6) | (DC >> 1)) & 0xff,
  [82] = ((DC << 7) | (DC >> 0)) & 0xff,
  [83] = ((DC << 1) | (DC >> 6)) & 0xff,
  [84] = ((DC << 2) | (DC >> 5)) & 0xff,
  [85] = ((DC << 3) | (DC >> 4)) & 0xff,
  [86] = ((DC << 4) | (DC >> 3)) & 0xff,
  [87] = ((DC << 5) | (DC >> 2)) & 0xff,
  [88] = ((DC << 6) | (DC >> 1)) & 0xff,
  [89] = ((DC << 7) | (DC >> 0)) & 0xff,
  [90] = ((DC << 1) | (DC >> 6)) & 0xff,
  [91] = ((DC << 2) | (DC >> 5)) & 0xff,
  [92] = ((DC << 3) | (DC >> 4)) & 0xff,
  [93] = ((DC << 4) | (DC >> 3)) & 0xff,
  [94] = ((DC << 5) | (DC >> 2)) & 0xff,
  [95] = ((DC << 6) | (DC >> 1)) & 0xff,
  [96] = ((DC << 7) | (DC >> 0)) & 0xff,
};

#undef DC
#undef MC
#undef BC

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void tlc5955_toggle_latch(void)
{
  volatile int i;

  stm32_gpiowrite(GPIO_TLC5955_LAT, true);
  for (i = 0; i < 100; i++);
  stm32_gpiowrite(GPIO_TLC5955_LAT, false);
}

static void tlc5955_spi_send(FAR const uint8_t *data, size_t len)
{
  SPI_LOCK(g_spi, true);
  SPI_SNDBLOCK(g_spi, data, len);
  SPI_LOCK(g_spi, false);
  tlc5955_toggle_latch();
}

/****************************************************************************
 * Name: tlc5955_update_worker
 *
 * Description:
 *   HPWORK handler that sends grayscale data if changed.
 ****************************************************************************/

static void tlc5955_update_worker(FAR void *arg)
{
  if (g_changed)
    {
      g_changed = false;
      g_grayscale[0] = 0;  /* bit 768 = 0 for GS latch */
      tlc5955_spi_send(g_grayscale, TLC5955_DATA_SIZE);
    }
}

static void tlc5955_gsclk_start(void)
{
  struct stm32_tim_dev_s *tim;

  stm32_configgpio(GPIO_GSCLK);

  tim = stm32_tim_init(GSCLK_TIM);
  DEBUGASSERT(tim != NULL);

  STM32_TIM_SETMODE(tim, STM32_TIM_MODE_UP);
  STM32_TIM_SETPERIOD(tim, GSCLK_ARR);
  STM32_TIM_SETCHANNEL(tim, GSCLK_CH, STM32_TIM_CH_OUTPWM);
  STM32_TIM_SETCOMPARE(tim, GSCLK_CH, GSCLK_CCR);
  STM32_TIM_ENABLE(tim);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_spi1select(FAR struct spi_dev_s *dev, uint32_t devid,
                      bool selected)
{
}

uint8_t stm32_spi1status(FAR struct spi_dev_s *dev, uint32_t devid)
{
  return 0;
}

int tlc5955_initialize(void)
{
  stm32_configgpio(GPIO_TLC5955_LAT);

  g_spi = stm32_spibus_initialize(1);
  if (g_spi == NULL)
    {
      syslog(LOG_ERR, "TLC5955: Failed to initialize SPI1\n");
      return -ENODEV;
    }

  /* NVIC priorities for STM32_IRQ_SPI1 / DMA2S2 / DMA2S3 are assigned
   * centrally in stm32_bringup.c (see "Issue #36 epsilon plan" block).
   */

  SPI_LOCK(g_spi, true);
  SPI_SETMODE(g_spi, SPIDEV_MODE0);
  SPI_SETBITS(g_spi, 8);
  SPI_SETFREQUENCY(g_spi, 24000000);
  SPI_LOCK(g_spi, false);

  tlc5955_gsclk_start();

  /* Send control latch twice (TLC5955 hardware requirement) */

  tlc5955_spi_send(g_control_latch, TLC5955_DATA_SIZE);
  tlc5955_spi_send(g_control_latch, TLC5955_DATA_SIZE);

  memset(g_grayscale, 0, TLC5955_DATA_SIZE);
  g_changed = false;

  syslog(LOG_INFO, "TLC5955: Initialized (48ch LED driver, SPI DMA)\n");
  return OK;
}

void tlc5955_set_duty(uint8_t ch, uint16_t value)
{
  DEBUGASSERT(ch < TLC5955_NUM_CH);

  g_grayscale[ch * 2 + 1] = value >> 8;
  g_grayscale[ch * 2 + 2] = value & 0xff;
  g_changed = true;

  /* Schedule deferred SPI transfer. work_queue() ignores duplicate
   * requests on the same work_s, so multiple set_duty calls are
   * batched into one SPI transfer (same behavior as pybricks process_poll).
   */

  work_queue(HPWORK, &g_update_work, tlc5955_update_worker, NULL, 0);
}

int tlc5955_update_sync(void)
{
  /* Immediate SPI transfer (for use during init/shutdown) */

  g_grayscale[0] = 0;
  tlc5955_spi_send(g_grayscale, TLC5955_DATA_SIZE);
  return OK;
}

#endif /* CONFIG_STM32_SPI1 */
