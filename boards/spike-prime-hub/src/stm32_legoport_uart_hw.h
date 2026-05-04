/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport_uart_hw.h
 *
 * Internal interface between the LUMP protocol engine
 * (`stm32_legoport_lump.c`) and the per-port USART register-level wrapper
 * (`stm32_legoport_uart_hw.c`).  Six UARTs (4/5/7/8/9/10) are owned at the
 * board level — the NuttX standard serial driver is intentionally bypassed
 * (no `_SERIALDRIVER` configs) so `/dev/ttyS*` is not registered for these
 * UARTs and the LUMP engine has full control over baud changes,
 * RX byte timing, and IRQ priority.
 *
 * See `docs/{ja,en}/drivers/lump-protocol.md` for design.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_SRC_STM32_LEGOPORT_UART_HW_H
#define __BOARDS_SPIKE_PRIME_HUB_SRC_STM32_LEGOPORT_UART_HW_H

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include <arch/board/board_legoport.h>

#ifdef CONFIG_LEGO_LUMP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* RX ring buffer size (per port).  Power of two for cheap masking.  Sized
 * generously so an unattended kthread can safely sleep through several
 * LUMP frames without overrunning at 460800 baud.
 */

#define LUMP_UART_RXRING_SIZE   256u
#define LUMP_UART_RXRING_MASK   (LUMP_UART_RXRING_SIZE - 1u)

/* RX ISR coalescing threshold.  Set to 1 so single-byte LUMP frames
 * (SYS_SYNC, SYS_ACK, SYS_NACK) reliably wake the kthread — these end
 * a sync stream and miss-waking on them stalls the engine.  The
 * `post_pending` flag still bounds the sem-post rate to one per
 * kthread service cycle, so coalescing still works under load.
 */

#define LUMP_UART_RXRING_POST_THRESH   1u

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Per-port HW descriptor (immutable) — one entry in the table for each of
 * the 6 LUMP UARTs.  Defined in stm32_legoport_lump_table.c.
 */

struct lump_uart_hw_desc_s
{
  uint32_t usart_base;   /* USART register block base (STM32_UARTn_BASE) */
  uint32_t apb_clk_hz;   /* APB1 (UART4/5/7/8) or APB2 (UART9/10) freq */
  uint32_t rcc_enr_reg;  /* STM32_RCC_APB1ENR or STM32_RCC_APB2ENR */
  uint32_t rcc_enr_bit;  /* RCC_APB1ENR_UARTnEN or RCC_APB2ENR_UARTnEN */
  uint16_t irq;          /* STM32_IRQ_UARTn */
  uint8_t  port_index;   /* 0..5 for A..F (debug only) */
};

extern const struct lump_uart_hw_desc_s g_lump_uart_hw_desc[BOARD_LEGOPORT_COUNT];

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Configure pins (TX/RX AF, BUF low) + RCC clock + USART CR1/BRR + IRQ
 * attach + up_enable_irq.  Idempotent: a second open at the same baud is
 * a no-op; a different baud triggers `lump_uart_set_baud()` semantics.
 *
 * Returns 0 on success or -EINVAL if `port` is out of range.  Other failures
 * are not currently signalled — the underlying register/IRQ helpers do not
 * fail in normal operation.
 */

int lump_uart_open(int port, uint32_t baud);

/* Symmetric tear-down.  Disables IRQ, clears CR1, and detaches the IRQ
 * handler.  Pins are returned to a safe state by the caller (DCM
 * `release_uart()` re-asserts the GPIO scan baseline).
 */

void lump_uart_close(int port);

/* Reprogram BRR for a new baud rate.  USART is briefly disabled (UE=0)
 * to avoid racing the baud generator while bytes stream — same idiom as
 * `stm32_btuart.c:btuart_apply_baud()`.
 */

int lump_uart_set_baud(int port, uint32_t baud);

/* Block waiting for one RX byte.  Returns 0 on success, -ETIMEDOUT on
 * timeout, or a negated errno.  Drains from the ISR-fed ring buffer; when
 * the ring is empty it sleeps on the per-port wakeup sem until the ISR
 * deposits new bytes (see "Wake coalescing" in the plan).
 *
 * Used during SYNC and INFO phases where the kthread reads byte-by-byte
 * to decode LUMP frames.  Callers in Phase 1 may use this for the
 * `lump-hw dump` diagnostic; full Phase 2/3 paths will call this from
 * the per-port LUMP kthread.
 */

int lump_uart_read_byte(int port, uint8_t *out, uint32_t timeout_ms);

/* Send `len` bytes by polling TXE.  byte-level timeout = 10 ms.
 * frame total timeout = max(50 ms, len * char_time * 2) where
 * char_time = 10 / baud * 1000 (ms).
 */

int lump_uart_write(int port, const uint8_t *buf, size_t len,
                    uint32_t timeout_ms);

/* Drop any pending bytes in the RX ring + clear ORE.  Called between
 * baud changes / state transitions where stale bytes could confuse the
 * LUMP frame parser.
 */

void lump_uart_flush_rx(int port);

/* Issue #100 案D: install LUMP UART HIPRI direct vectors via
 * arm_ramvec_attach().  MUST be called once from stm32_bringup() before
 * any port opens.  Returns 0 on success or a negative errno from
 * arm_ramvec_attach() on failure.
 */

int lump_uart_hipri_init(void);

#ifdef CONFIG_LEGO_LUMP_DIAG
/* Print a register-level dump of the 6 LUMP UARTs to syslog/stdout via
 * printf.  Used by `port lump-hw dump`.  No side-effects.
 */

void lump_uart_hw_dump(void);
#endif

/* Snapshot the per-port USART overrun counter (ISR detected ORE).  Used
 * by the LUMP engine for diagnostic logging.  Returns 0 if `port` is
 * out of range.
 */

uint8_t lump_uart_get_ore_count(int port);

#endif /* CONFIG_LEGO_LUMP */
#endif /* __BOARDS_SPIKE_PRIME_HUB_SRC_STM32_LEGOPORT_UART_HW_H */
