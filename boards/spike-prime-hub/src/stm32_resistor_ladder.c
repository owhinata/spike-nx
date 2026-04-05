/****************************************************************************
 * boards/spike-prime-hub/src/stm32_resistor_ladder.c
 *
 * Resistor ladder decoder for SPIKE Prime Hub.
 * Ported from pybricks resistor_ladder.c (MIT license).
 *
 * A resistor ladder encodes up to 3 digital signals (CH0/CH1/CH2) as a
 * single analog voltage.  The decoder compares an ADC reading against 8
 * threshold levels and returns a bitmask of active channels.
 *
 * SPIKE Prime Hub has two ladders:
 *   DEV_0 (PC4, ADC rank 4): CH1 = center button, CH2 = /CHG (MP2639A)
 *   DEV_1 (PA1, ADC rank 5): CH0 = left, CH1 = right, CH2 = BT button
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_ADC1

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Threshold levels from pybricks platform.c (prime_hub).
 * Each array has 8 entries in descending order.  The ADC reading is
 * compared top-down; the first level it exceeds determines the active
 * channel combination.
 */

const uint16_t g_ladder_dev0_levels[8] =
{
  3642, 3142, 2879, 2634, 2449, 2209, 2072, 1800
};

const uint16_t g_ladder_dev1_levels[8] =
{
  3872, 3394, 3009, 2755, 2538, 2327, 2141, 1969
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: resistor_ladder_decode
 *
 * Description:
 *   Decode an ADC reading into a bitmask of active channels using 8
 *   threshold levels.  The mapping follows the pybricks convention:
 *
 *   ADC > level[0] → 0 (no channels active)
 *   ADC > level[1] → CH2
 *   ADC > level[2] → CH1
 *   ADC > level[3] → CH1 | CH2
 *   ADC > level[4] → CH0
 *   ADC > level[5] → CH0 | CH2
 *   ADC > level[6] → CH0 | CH1
 *   ADC > level[7] → CH0 | CH1 | CH2
 *   ADC <= level[7] → 0xff (error / hardware failure)
 *
 * Input Parameters:
 *   adc_value - Raw 12-bit ADC reading
 *   levels    - Array of 8 threshold values in descending order
 *
 * Returned Value:
 *   Bitmask of RLAD_CHx flags, or 0xff on error.
 *
 ****************************************************************************/

uint8_t resistor_ladder_decode(uint16_t adc_value, const uint16_t levels[8])
{
  if (adc_value > levels[0])
    {
      return 0;
    }

  if (adc_value > levels[1])
    {
      return RLAD_CH2;
    }

  if (adc_value > levels[2])
    {
      return RLAD_CH1;
    }

  if (adc_value > levels[3])
    {
      return RLAD_CH1 | RLAD_CH2;
    }

  if (adc_value > levels[4])
    {
      return RLAD_CH0;
    }

  if (adc_value > levels[5])
    {
      return RLAD_CH0 | RLAD_CH2;
    }

  if (adc_value > levels[6])
    {
      return RLAD_CH0 | RLAD_CH1;
    }

  if (adc_value > levels[7])
    {
      return RLAD_CH0 | RLAD_CH1 | RLAD_CH2;
    }

  /* Below all levels — hardware error */

  return 0xff;
}

#endif /* CONFIG_STM32_ADC1 */
