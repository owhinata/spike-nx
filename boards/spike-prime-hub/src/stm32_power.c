/****************************************************************************
 * boards/spike-prime-hub/src/stm32_power.c
 *
 * Center button monitor and power control for SPIKE Prime Hub.
 *
 * The center button is connected via a resistor ladder to ADC1 CH4 (PC4).
 * ADC value between 3142-3642 indicates button pressed (12-bit ADC).
 *
 * Short press (< 2s): system reset
 * Long press (>= 2s): power off (PA13 LOW)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/board.h>

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_ADC1

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Use NuttX ADC register definitions where available.
 * Only define what's missing.
 */

#include "hardware/stm32_adc.h"

#ifndef ADC_SQR3_OFFSET
#  define ADC_SQR3_OFFSET     0x34
#endif
#ifndef ADC_DR_OFFSET
#  define ADC_DR_OFFSET       0x4c
#endif

/* Button timing */

#define BTN_POLL_INTERVAL_MS  50
#define BTN_LONG_PRESS_MS     2000

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct work_s g_btn_work;
static bool g_btn_pressed;
static uint32_t g_btn_press_start;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: adc_read_channel
 *
 * Description:
 *   Read a single ADC1 channel using direct register access.
 ****************************************************************************/

static uint16_t adc_read_channel(uint8_t ch)
{
  /* Configure channel in regular sequence (single conversion) */

  putreg32(ch, STM32_ADC1_BASE + ADC_SQR3_OFFSET);

  /* Start conversion */

  modifyreg32(STM32_ADC1_BASE + STM32_ADC_CR2_OFFSET, 0, ADC_CR2_SWSTART);

  /* Wait for conversion complete */

  while (!(getreg32(STM32_ADC1_BASE + STM32_ADC_SR_OFFSET) & ADC_SR_EOC));

  /* Read result */

  return (uint16_t)(getreg32(STM32_ADC1_BASE + ADC_DR_OFFSET) & 0xfff);
}

/****************************************************************************
 * Name: center_btn_pressed
 *
 * Description:
 *   Check if center button is pressed via ADC resistor ladder.
 ****************************************************************************/

static bool center_btn_pressed(void)
{
  uint16_t val = adc_read_channel(CENTER_BTN_ADC_CH);

  return (val < CENTER_BTN_PRESS_THRESHOLD);
}

/****************************************************************************
 * Name: power_off
 *
 * Description:
 *   Shut down the system by pulling PA13 (BAT_PWR_EN) low.
 ****************************************************************************/

static void power_off(void)
{
  syslog(LOG_INFO, "POWER: Shutting down...\n");

  /* Visual feedback: turn center LED blue briefly */

#ifdef CONFIG_STM32_SPI1
  tlc5955_set_duty(TLC5955_CH_STATUS_TOP_G, 0);
  tlc5955_set_duty(TLC5955_CH_STATUS_BTM_G, 0);
  tlc5955_set_duty(TLC5955_CH_STATUS_TOP_B, 0xffff);
  tlc5955_set_duty(TLC5955_CH_STATUS_BTM_B, 0xffff);
  tlc5955_update();
#endif

  /* Wait for button release to avoid immediate re-power-on */

  while (center_btn_pressed());
  up_mdelay(100);

  /* Cut power: PA13 LOW */

  stm32_gpiowrite(GPIO_BAT_PWR_EN, false);

  /* If USB is connected, power stays on via USB.
   * Wait briefly then reset to enter a clean state.
   */

  up_mdelay(500);
  board_reset(0);
}

/****************************************************************************
 * Name: btn_monitor_work
 *
 * Description:
 *   Work queue handler that periodically polls the center button.
 ****************************************************************************/

static void btn_monitor_work(FAR void *arg)
{
  bool pressed = center_btn_pressed();
  uint32_t now = clock_systime_ticks();

  if (pressed && !g_btn_pressed)
    {
      /* Button just pressed */

      g_btn_pressed = true;
      g_btn_press_start = now;
    }
  else if (!pressed && g_btn_pressed)
    {
      /* Button released */

      g_btn_pressed = false;
    }
  else if (pressed && g_btn_pressed)
    {
      /* Button still held */

      uint32_t held_ms = TICK2MSEC(now - g_btn_press_start);

      if (held_ms >= BTN_LONG_PRESS_MS)
        {
          /* Long press: power off */

          syslog(LOG_INFO, "POWER: Center button long press - power off\n");
          power_off();
        }
    }

  /* Schedule next poll */

  work_queue(HPWORK, &g_btn_work, btn_monitor_work, NULL,
             MSEC2TICK(BTN_POLL_INTERVAL_MS));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_power_initialize
 *
 * Description:
 *   Initialize ADC1 for center button reading and start the button
 *   monitoring work queue.
 ****************************************************************************/

int stm32_power_initialize(void)
{
  /* Configure PC4 as analog input */

  stm32_configgpio(GPIO_ADC_CENTER_BTN);

  /* Enable ADC1 clock */

  modifyreg32(STM32_RCC_APB2ENR, 0, RCC_APB2ENR_ADC1EN);

  /* Set ADC clock prescaler: PCLK2/4 = 96MHz/4 = 24MHz (max 36MHz)
   * ADC_CCR at ADC common base + 0x04, ADCPRE bits [17:16] = 01
   */

  modifyreg32(STM32_ADC_CCR, 3 << 16, 1 << 16);

  /* Configure ADC1: 12-bit resolution, single conversion mode */

  putreg32(ADC_CR1_RES_12BIT, STM32_ADC1_BASE + STM32_ADC_CR1_OFFSET);

  /* Set sampling time for CH14: 56 cycles
   * CH10-18 are in SMPR1 register, 3 bits per channel
   * CH14 offset = (14 - 10) * 3 = 12
   */

  modifyreg32(STM32_ADC1_BASE + STM32_ADC_SMPR1_OFFSET,
              0x7 << 12, 3 << 12);

  /* Power on ADC */

  modifyreg32(STM32_ADC1_BASE + STM32_ADC_CR2_OFFSET, 0, ADC_CR2_ADON);

  /* Small delay for ADC startup */

  up_mdelay(1);

  syslog(LOG_INFO, "POWER: Center button monitor started "
         "(long press=poweroff)\n");

  /* Start periodic button monitoring on HPWORK queue.
   * Delay initial poll by 3 seconds to avoid false triggers during boot.
   */

  return work_queue(HPWORK, &g_btn_work, btn_monitor_work, NULL,
                    MSEC2TICK(3000));
}

#endif /* CONFIG_STM32_ADC1 */
