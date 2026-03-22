/****************************************************************************
 * boards/spike-prime-hub/include/board.h
 *
 * LEGO SPIKE Prime Hub board configuration.
 * MCU: STM32F413VGT6 (100-pin, 1MB Flash, 320KB SRAM)
 *
 * Clocking:
 *   HSE = 16 MHz (on-board oscillator)
 *   SYSCLK = 96 MHz (USB requires 48 MHz PLL48CLK)
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* SPIKE Prime Hub has a 16MHz HSE oscillator.
 *
 *   PLL source  = HSE
 *   PLL_VCO     = (HSE / PLLM) * PLLN = (16MHz / 16) * 192 = 192 MHz
 *   SYSCLK      = PLL_VCO / PLLP = 192 / 2 = 96 MHz
 *   PLL48CLK    = PLL_VCO / PLLQ = 192 / 4 = 48 MHz (for USB)
 *   Flash wait  = 3 WS (96 MHz, 2.7-3.6V)
 */

#define STM32_BOARD_XTAL        16000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(16)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(192)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(4)

#define STM32_SYSCLK_FREQUENCY  96000000ul

/* AHB clock (HCLK) is SYSCLK (96MHz) */

#define STM32_RCC_CFGR_HPRE    RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY   STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/2 (48MHz) */

#define STM32_RCC_CFGR_PPRE1   RCC_CFGR_PPRE1_HCLKd2
#define STM32_PCLK1_FREQUENCY  (STM32_HCLK_FREQUENCY / 2)

/* Timers driven from APB1 will be twice PCLK1 */

#define STM32_APB1_TIM2_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN  (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM12_CLKIN (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM13_CLKIN (2 * STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM14_CLKIN (2 * STM32_PCLK1_FREQUENCY)

/* APB2 clock (PCLK2) is HCLK (96MHz) */

#define STM32_RCC_CFGR_PPRE2   RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY  STM32_HCLK_FREQUENCY

/* Timers driven from APB2 will be PCLK2 (since prescaler = 1) */

#define STM32_APB2_TIM1_CLKIN  STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM8_CLKIN  STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM9_CLKIN  STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM10_CLKIN STM32_PCLK2_FREQUENCY
#define STM32_APB2_TIM11_CLKIN STM32_PCLK2_FREQUENCY

/* Timer Frequencies */

#define BOARD_TIM1_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM2_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM3_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM4_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM5_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM6_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM7_FREQUENCY   STM32_HCLK_FREQUENCY
#define BOARD_TIM8_FREQUENCY   STM32_HCLK_FREQUENCY

/* LED definitions **********************************************************/

/* SPIKE Prime Hub LEDs are controlled via TLC5955 (SPI1).
 * No GPIO-direct LEDs available. Define minimal for NuttX status.
 */

#define BOARD_NLEDS       0

/* Button definitions *******************************************************/

/* Bluetooth button (directly accessible) */

#define BUTTON_USER       0
#define NUM_BUTTONS       1
#define BUTTON_USER_BIT   (1 << BUTTON_USER)

/* SPI1: TLC5955 LED driver */

#define GPIO_SPI1_MISO    GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI    GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK     GPIO_SPI1_SCK_1

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_H */
