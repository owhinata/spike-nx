/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport_lump_table.c
 *
 * Per-port HW descriptor table for the LUMP UART engine (Issue #43).
 *
 * Six SPIKE Prime Hub I/O ports map to UART7 / UART4 / UART8 / UART5 /
 * UART10 / UART9 (A..F).  UART4/5/7/8 are on APB1 (48 MHz @ HCLK=96 MHz);
 * UART9/10 are on APB2 (96 MHz).  See `boards/spike-prime-hub/include/board.h`
 * for the clock derivation and `docs/{ja,en}/hardware/pin-mapping.md` for
 * the GPIO alternate-function map.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "stm32.h"
#include "hardware/stm32f40xxx_uart.h"
#include "hardware/stm32f40xxx_rcc.h"

#include "stm32_legoport_uart_hw.h"

#ifdef CONFIG_LEGO_LUMP

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Indexed by `legoport_pin_s::port_index` (0=A, 1=B, ... 5=F).
 *
 * APB clock values come straight from board.h: PCLK1=HCLK/2, PCLK2=HCLK,
 * with HCLK = STM32_SYSCLK_FREQUENCY = 96 MHz.  See `board.h:53,62-63,79-80`.
 *
 * NVIC priority for these IRQs is set in `stm32_bringup.c` (LUMP slot,
 * `dma-irq.md:151`) — not stored here because `up_prioritize_irq()` is
 * called once at boot and not per-port at run time.
 */

const struct lump_uart_hw_desc_s g_lump_uart_hw_desc[BOARD_LEGOPORT_COUNT] =
{
  /* ----- Port A: UART7 (APB1) ----- */
  [0] = {
    .usart_base  = STM32_UART7_BASE,
    .apb_clk_hz  = STM32_PCLK1_FREQUENCY,
    .rcc_enr_reg = STM32_RCC_APB1ENR,
    .rcc_enr_bit = RCC_APB1ENR_UART7EN,
    .irq         = STM32_IRQ_UART7,
    .port_index  = 0,
  },
  /* ----- Port B: UART4 (APB1) ----- */
  [1] = {
    .usart_base  = STM32_UART4_BASE,
    .apb_clk_hz  = STM32_PCLK1_FREQUENCY,
    .rcc_enr_reg = STM32_RCC_APB1ENR,
    .rcc_enr_bit = RCC_APB1ENR_UART4EN,
    .irq         = STM32_IRQ_UART4,
    .port_index  = 1,
  },
  /* ----- Port C: UART8 (APB1) ----- */
  [2] = {
    .usart_base  = STM32_UART8_BASE,
    .apb_clk_hz  = STM32_PCLK1_FREQUENCY,
    .rcc_enr_reg = STM32_RCC_APB1ENR,
    .rcc_enr_bit = RCC_APB1ENR_UART8EN,
    .irq         = STM32_IRQ_UART8,
    .port_index  = 2,
  },
  /* ----- Port D: UART5 (APB1) ----- */
  [3] = {
    .usart_base  = STM32_UART5_BASE,
    .apb_clk_hz  = STM32_PCLK1_FREQUENCY,
    .rcc_enr_reg = STM32_RCC_APB1ENR,
    .rcc_enr_bit = RCC_APB1ENR_UART5EN,
    .irq         = STM32_IRQ_UART5,
    .port_index  = 3,
  },
  /* ----- Port E: UART10 (APB2) ----- */
  [4] = {
    .usart_base  = STM32_UART10_BASE,
    .apb_clk_hz  = STM32_PCLK2_FREQUENCY,
    .rcc_enr_reg = STM32_RCC_APB2ENR,
    .rcc_enr_bit = RCC_APB2ENR_UART10EN,
    .irq         = STM32_IRQ_UART10,
    .port_index  = 4,
  },
  /* ----- Port F: UART9 (APB2) ----- */
  [5] = {
    .usart_base  = STM32_UART9_BASE,
    .apb_clk_hz  = STM32_PCLK2_FREQUENCY,
    .rcc_enr_reg = STM32_RCC_APB2ENR,
    .rcc_enr_bit = RCC_APB2ENR_UART9EN,
    .irq         = STM32_IRQ_UART9,
    .port_index  = 5,
  },
};

#endif /* CONFIG_LEGO_LUMP */
