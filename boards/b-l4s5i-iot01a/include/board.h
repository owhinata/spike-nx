/****************************************************************************
 * boards/b-l4s5i-iot01a/include/board.h
 *
 * B-L4S5I-IOT01A Discovery Kit board configuration.
 * Using STM32L4R5VI as a workaround (L4S5 differs only in AES/HASH).
 *
 * Clocking:
 *   HSI = 16 MHz (no HSE crystal populated)
 *   SYSCLK = 80 MHz via PLL
 ****************************************************************************/

#ifndef __BOARDS_B_L4S5I_IOT01A_INCLUDE_BOARD_H
#define __BOARDS_B_L4S5I_IOT01A_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdbool.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *****************************************************************/

/* Same PCB as B-L475E-IOT01A: no HSE crystal, use HSI 16 MHz -> PLL 80 MHz.
 *
 *   System Clock source         : PLL (HSI)
 *   SYSCLK(Hz)                  : 80000000
 *   HCLK(Hz)                    : 80000000
 *   AHB Prescaler               : 1
 *   APB1 Prescaler              : 1
 *   APB2 Prescaler              : 1
 *   HSI Frequency(Hz)           : 16000000
 *   PLLM                        : 1
 *   PLLN                        : 10
 *   PLLP                        : 0 (disabled)
 *   PLLQ                        : 2 (for CLK48 via PLLSAI1)
 *   PLLR                        : 2
 *   Flash Latency(WS)           : 4
 */

#define STM32L4_HSI_FREQUENCY     16000000ul
#define STM32L4_LSI_FREQUENCY     32000
#define STM32L4_LSE_FREQUENCY     32768

#define BOARD_AHB_FREQUENCY       80000000ul

#define STM32L4_BOARD_USEHSI      1

/* Main PLL: HSI(16MHz) / 1 * 10 / 2 = 80 MHz */

#define STM32L4_PLLCFG_PLLM             RCC_PLLCFG_PLLM(1)
#define STM32L4_PLLCFG_PLLN             RCC_PLLCFG_PLLN(10)
#define STM32L4_PLLCFG_PLLP             0
#undef  STM32L4_PLLCFG_PLLP_ENABLED
#define STM32L4_PLLCFG_PLLQ             RCC_PLLCFG_PLLQ_2
#define STM32L4_PLLCFG_PLLQ_ENABLED
#define STM32L4_PLLCFG_PLLR             RCC_PLLCFG_PLLR(2)
#define STM32L4_PLLCFG_PLLR_ENABLED

/* PLLSAI1: HSI(16MHz) / 1 * 12 / 4 = 48 MHz for CLK48 */

#define STM32L4_PLLSAI1CFG_PLLN         RCC_PLLSAI1CFG_PLLN(12)
#define STM32L4_PLLSAI1CFG_PLLP         0
#undef  STM32L4_PLLSAI1CFG_PLLP_ENABLED
#define STM32L4_PLLSAI1CFG_PLLQ         RCC_PLLSAI1CFG_PLLQ_4
#define STM32L4_PLLSAI1CFG_PLLQ_ENABLED
#define STM32L4_PLLSAI1CFG_PLLR         0
#undef  STM32L4_PLLSAI1CFG_PLLR_ENABLED

/* PLLSAI2: not used */

#define STM32L4_PLLSAI2CFG_PLLN         RCC_PLLSAI2CFG_PLLN(8)
#define STM32L4_PLLSAI2CFG_PLLP         0
#undef  STM32L4_PLLSAI2CFG_PLLP_ENABLED
#define STM32L4_PLLSAI2CFG_PLLR         0
#undef  STM32L4_PLLSAI2CFG_PLLR_ENABLED

#define STM32L4_SYSCLK_FREQUENCY  80000000ul

/* CLK48 from PLLSAI1 Q output */

#define STM32L4_USE_CLK48
#define STM32L4_CLK48_SEL         RCC_CCIPR_CLK48SEL_PLLSAI1

/* Enable LSE for RTC */

#define STM32L4_USE_LSE           1

/* AHB clock (HCLK) is SYSCLK (80 MHz) */

#define STM32L4_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK
#define STM32L4_HCLK_FREQUENCY    STM32L4_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK (80 MHz) */

#define STM32L4_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLK
#define STM32L4_PCLK1_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

#define STM32L4_APB1_TIM2_CLKIN   STM32L4_PCLK1_FREQUENCY
#define STM32L4_APB1_TIM3_CLKIN   STM32L4_PCLK1_FREQUENCY
#define STM32L4_APB1_TIM4_CLKIN   STM32L4_PCLK1_FREQUENCY
#define STM32L4_APB1_TIM5_CLKIN   STM32L4_PCLK1_FREQUENCY
#define STM32L4_APB1_TIM6_CLKIN   STM32L4_PCLK1_FREQUENCY
#define STM32L4_APB1_TIM7_CLKIN   STM32L4_PCLK1_FREQUENCY
#define STM32L4_APB1_LPTIM1_CLKIN STM32L4_PCLK1_FREQUENCY
#define STM32L4_APB1_LPTIM2_CLKIN STM32L4_PCLK1_FREQUENCY

/* APB2 clock (PCLK2) is HCLK (80 MHz) */

#define STM32L4_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK
#define STM32L4_PCLK2_FREQUENCY   (STM32L4_HCLK_FREQUENCY / 1)

#define STM32L4_APB2_TIM1_CLKIN   STM32L4_PCLK2_FREQUENCY
#define STM32L4_APB2_TIM8_CLKIN   STM32L4_PCLK2_FREQUENCY
#define STM32L4_APB2_TIM15_CLKIN  STM32L4_PCLK2_FREQUENCY
#define STM32L4_APB2_TIM16_CLKIN  STM32L4_PCLK2_FREQUENCY
#define STM32L4_APB2_TIM17_CLKIN  STM32L4_PCLK2_FREQUENCY

/* Timer Frequencies */

#define BOARD_TIM1_FREQUENCY    STM32L4_APB2_TIM1_CLKIN
#define BOARD_TIM2_FREQUENCY    STM32L4_APB1_TIM2_CLKIN
#define BOARD_TIM3_FREQUENCY    STM32L4_APB1_TIM3_CLKIN
#define BOARD_TIM4_FREQUENCY    STM32L4_APB1_TIM4_CLKIN
#define BOARD_TIM5_FREQUENCY    STM32L4_APB1_TIM5_CLKIN
#define BOARD_TIM6_FREQUENCY    STM32L4_APB1_TIM6_CLKIN
#define BOARD_TIM7_FREQUENCY    STM32L4_APB1_TIM7_CLKIN
#define BOARD_TIM8_FREQUENCY    STM32L4_APB2_TIM8_CLKIN
#define BOARD_TIM15_FREQUENCY   STM32L4_APB2_TIM15_CLKIN
#define BOARD_TIM16_FREQUENCY   STM32L4_APB2_TIM16_CLKIN
#define BOARD_TIM17_FREQUENCY   STM32L4_APB2_TIM17_CLKIN
#define BOARD_LPTIM1_FREQUENCY  STM32L4_APB1_LPTIM1_CLKIN
#define BOARD_LPTIM2_FREQUENCY  STM32L4_APB1_LPTIM2_CLKIN

/* LED definitions **********************************************************/

/* B-L4S5I-IOT01A has 2 user LEDs:
 *   LD1 (green) = PA5
 *   LD2 (red)   = PB14
 */

#define BOARD_LED1        0
#define BOARD_LED2        1
#define BOARD_NLEDS       2

#define BOARD_LED1_BIT    (1 << BOARD_LED1)
#define BOARD_LED2_BIT    (1 << BOARD_LED2)

#define LED_STARTED      0 /* NuttX has been started  OFF      */
#define LED_HEAPALLOCATE 0 /* Heap has been allocated OFF      */
#define LED_IRQSENABLED  0 /* Interrupts enabled      OFF      */
#define LED_STACKCREATED 1 /* Idle stack created      ON       */
#define LED_INIRQ        2 /* In an interrupt         N/C      */
#define LED_SIGNAL       2 /* In a signal handler     N/C      */
#define LED_ASSERTION    2 /* An assertion failed     N/C      */
#define LED_PANIC        3 /* The system has crashed  FLASH    */
#undef  LED_IDLE           /* MCU is in sleep mode    Not used */

/* Alternate function pin selections ****************************************/

/* USART1: Connected to ST-Link Debug via PB6 (TX), PB7 (RX) */

#define GPIO_USART1_RX GPIO_USART1_RX_2
#define GPIO_USART1_TX GPIO_USART1_TX_2

/* I2C2: Onboard sensors (LSM6DSL, etc.) via PB10 (SCL), PB11 (SDA) */

#define GPIO_I2C2_SCL  GPIO_I2C2_SCL_1
#define GPIO_I2C2_SDA  GPIO_I2C2_SDA_1

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_B_L4S5I_IOT01A_INCLUDE_BOARD_H */
