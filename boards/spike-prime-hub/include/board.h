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

/* USART6: Debug via external ST-Link (emergency use)
 *   PC6 = TX (AF8)
 *   PC7 = RX (AF8)
 *
 * Note: PG14/PG9 (USART6_TX/RX_2) are not available on the 100-pin
 * LQFP package (STM32F413VG). PC6/PC7 are the only option.
 * These pins are shared with Port E H-bridge motor PWM (TIM3 CH1/CH2),
 * so USART6 and Port E motor are mutually exclusive.
 */

#define GPIO_USART6_RX    GPIO_USART6_RX_1
#define GPIO_USART6_TX    GPIO_USART6_TX_1

/* I2C2: IMU (LSM6DS3TR-C)
 *   PB10 = I2C2_SCL (AF4)
 *   PB3  = I2C2_SDA (AF9, F413-specific)
 */

#define GPIO_I2C2_SCL     GPIO_I2C2_SCL_1   /* PB10 */
#define GPIO_I2C2_SDA     GPIO_I2C2_SDA_3   /* PB3 AF9 */

/* SPI1: TLC5955 LED driver */

#define GPIO_SPI1_MISO    GPIO_SPI1_MISO_1
#define GPIO_SPI1_MOSI    GPIO_SPI1_MOSI_1
#define GPIO_SPI1_SCK     GPIO_SPI1_SCK_1

/* SPI1 DMA channel mapping (DMA2, Channel 3)
 *   RX: DMA2 Stream2 Channel 3
 *   TX: DMA2 Stream3 Channel 3
 * NuttX SPI driver expects DMACHAN_SPI1_RX/TX symbols.
 */

#define DMACHAN_SPI1_RX   DMAMAP_SPI1_RX_2   /* DMA2 Stream2 Ch3 */
#define DMACHAN_SPI1_TX   DMAMAP_SPI1_TX_1   /* DMA2 Stream3 Ch3 */

/* SPI2: W25Q256 NOR Flash (32 MB)
 *   SCK  = PB13 (AF5)
 *   MISO = PC2  (AF5)
 *   MOSI = PC3  (AF5)
 *   /CS  = PB12 (GPIO software NSS, defined in spike_prime_hub.h)
 *
 * Pin selection follows the SPIKE Hub physical wiring (matches pybricks
 * lib/pbio/platform/prime_hub/platform.c).  The PB10 SPI2_SCK alternative
 * is unavailable here because PB10 is already used for I2C2_SCL.
 */

#define GPIO_SPI2_SCK     GPIO_SPI2_SCK_2    /* PB13 */
#define GPIO_SPI2_MISO    GPIO_SPI2_MISO_2   /* PC2 */
#define GPIO_SPI2_MOSI    GPIO_SPI2_MOSI_2   /* PC3 */

/* SPI2 DMA channel mapping (DMA1, Channel 0)
 *   RX: DMA1 Stream3 Channel 0
 *   TX: DMA1 Stream4 Channel 0
 */

#define DMACHAN_SPI2_RX   DMAMAP_SPI2_RX     /* DMA1 Stream3 Ch0 */
#define DMACHAN_SPI2_TX   DMAMAP_SPI2_TX     /* DMA1 Stream4 Ch0 */

/* USART2: CC2564C Bluetooth HCI UART (Issue #47)
 *   PD5 = USART2_TX  (AF7)
 *   PD6 = USART2_RX  (AF7)
 *   PD3 = USART2_CTS (AF7, hardware flow control)
 *   PD4 = USART2_RTS (AF7, hardware flow control)
 */

#define GPIO_USART2_TX    GPIO_USART2_TX_2    /* PD5 */
#define GPIO_USART2_RX    GPIO_USART2_RX_2    /* PD6 */
#define GPIO_USART2_CTS   GPIO_USART2_CTS_2   /* PD3 */
#define GPIO_USART2_RTS   GPIO_USART2_RTS_2   /* PD4 */

/* USART2 DMA channel mapping for Bluetooth (board-local, BT-dedicated).
 * Per RM0430 Rev 9 Table 30 (DMA1 request mapping) there are two USART2_RX
 * mappings on STM32F413 (CHSEL 4-bit extension): S5/Ch4 (NuttX default via
 * DMAMAP_USART2_RX) and S7/Ch6 (multiplexed entry).  We choose S7/Ch6 so
 * the BT driver does not collide with code that relies on the default map.
 * TX S6/Ch4 matches the NuttX default and pybricks Prime Hub platform.
 */

#define DMACHAN_USART2_BT_TX  DMAMAP_USART2_TX                        /* DMA1 S6 Ch4 */
#define DMACHAN_USART2_BT_RX  STM32_DMA_MAP(DMA1, DMA_STREAM7, DMA_CHAN6) /* DMA1 S7 Ch6 */

/* TIM8 CH4: CC2564C 32.768 kHz slow clock (BT sleep clock)
 *   PC9 = TIM8_CH4 (AF3)
 *
 * PWM at 32.768 kHz, 50% duty cycle (PSC=0, ARR=2929, CCR4=1465) from
 * APB2=96 MHz.  Must be stable before nSHUTD goes high.
 */

#define GPIO_TIM8_CH4_BT_SLOWCLK  GPIO_TIM8_CH4_0   /* PC9 */

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_H */
