/****************************************************************************
 * boards/stm32f413-discovery/include/board.h
 *
 * STM32F413H-Discovery Kit board configuration.
 * Using STM32F412ZG as a workaround (Phase B).
 *
 * Clocking:
 *   HSE = 8 MHz (on-board crystal)
 *   SYSCLK = 96 MHz (max for STM32F412)
 *   USB requires 48 MHz from PLL48CLK
 ****************************************************************************/

#ifndef __BOARDS_STM32F413_DISCOVERY_INCLUDE_BOARD_H
#define __BOARDS_STM32F413_DISCOVERY_INCLUDE_BOARD_H

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

/* STM32F413H-Discovery has an 8MHz HSE crystal.
 *
 *   PLL source  = HSE
 *   PLL_VCO     = (HSE / PLLM) * PLLN = (8MHz / 8) * 384 = 384 MHz
 *   SYSCLK      = PLL_VCO / PLLP = 384 / 4 = 96 MHz
 *   PLL48CLK    = PLL_VCO / PLLQ = 384 / 8 = 48 MHz (for USB)
 *   Flash wait  = 3 WS (96 MHz, 2.7-3.6V)
 */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     32768

#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(384)
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)

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

/* STM32F413H-Discovery has 2 LEDs:
 *   LD1 (green) = PE3
 *   LD2 (red)   = PC5
 */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED_GREEN   BOARD_LED1
#define BOARD_LED_RED     BOARD_LED2

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

/* Auto-LED definitions for NuttX status */

#define LED_STARTED       0  /* LED1 (green) */
#define LED_HEAPALLOCATE  1  /* LED2 (red) */
#define LED_IRQSENABLED   2  /* LED1 + LED2 */
#define LED_STACKCREATED  3  /* LED1 (green) */
#define LED_INIRQ         4  /* LED1 */
#define LED_SIGNAL        5  /* LED2 */
#define LED_ASSERTION     6  /* LED1 + LED2 */
#define LED_PANIC         7  /* LED2 (red, blinking) */

/* Button definitions *******************************************************/

/* STM32F413H-Discovery has 1 user button (PA0) */

#define BUTTON_USER       0
#define NUM_BUTTONS       1
#define BUTTON_USER_BIT   (1 << BUTTON_USER)

/* USART6: Connected to ST-Link VCP on Discovery Kit (UM2135)
 *   PG14 = TX (AF8)
 *   PG9  = RX (AF8)
 */

#define GPIO_USART6_RX    GPIO_USART6_RX_2
#define GPIO_USART6_TX    GPIO_USART6_TX_2

/* UART9: F413-specific (AF11)
 *   PD15 = TX
 *   PD14 = RX
 */

#define GPIO_UART9_TX     GPIO_UART9_TX_1
#define GPIO_UART9_RX     GPIO_UART9_RX_1

/* UART10: F413-specific (AF11)
 *   PG12 = TX (PE3 conflicts with LD1)
 *   PG11 = RX
 */

#define GPIO_UART10_TX    GPIO_UART10_TX_2
#define GPIO_UART10_RX    GPIO_UART10_RX_2

/* SPI1: Available on Arduino header */

#define GPIO_SPI1_MISO    GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI    GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK     GPIO_SPI1_SCK_1

#endif /* __BOARDS_STM32F413_DISCOVERY_INCLUDE_BOARD_H */
