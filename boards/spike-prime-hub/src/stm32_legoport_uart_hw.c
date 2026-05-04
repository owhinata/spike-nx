/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport_uart_hw.c
 *
 * SPDX-License-Identifier: MIT
 *
 * Register-level USART wrapper for the LUMP UART engine (Issue #43).
 * Owns UART4/5/7/8/9/10 directly, bypassing `arch/arm/src/stm32/stm32_serial.c`
 * (no `CONFIG_STM32_UARTn_SERIALDRIVER` for these ports → no `/dev/ttyS*`).
 *
 * Six identical instances are held in `g_lump_uart`, one per LUMP port.
 * The base register / IRQ / RCC bits per port live in
 * `stm32_legoport_lump_table.c`.  RX uses an IRQ-fed power-of-two ring
 * buffer with a coalesced wake-up sem (`post_pending` flag) so a 6-port
 * 115200-baud stream lands at most ~ N/8 sem posts per second instead of
 * one per byte.  TX is synchronous polled-TXE — frames are short
 * (≤ 36 bytes) and per-port kthreads are independent.
 *
 * Pattern follows `stm32_btuart.c` for clock-enable / GPIO AF / IRQ
 * attach.  See `docs/{ja,en}/drivers/lump-protocol.md` for the design.
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>
#include <syslog.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/clock.h>
#include <nuttx/semaphore.h>
#include <nuttx/spinlock.h>

#include <arch/board/board.h>
#include <arch/board/board_legoport.h>

#include "arm_internal.h"
#include "ram_vectors.h"
#include "stm32.h"
#include "hardware/stm32f40xxx_uart.h"
#include "hardware/stm32f40xxx_rcc.h"
#include "nvic.h"

#include "stm32_legoport_uart_hw.h"

#ifdef CONFIG_LEGO_LUMP

/* Issue #100 case D: this driver runs the LUMP UART ISRs at NVIC priority
 * NVIC_SYSH_HIGH_PRIORITY (= 0x60), above BASEPRI 0x80.  HIPRI ISRs
 * are NOT routed through arm_doirq() / irq_dispatch() — they MUST be
 * installed via arm_ramvec_attach() which requires CONFIG_ARCH_RAMVECTORS.
 * BUILD_PROTECTED + HIPRI also requires CONFIG_ARCH_INTERRUPTSTACK >= 8.
 */

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
#  error LUMP UART case D requires CONFIG_ARCH_HIPRI_INTERRUPT
#endif

#ifndef CONFIG_ARCH_RAMVECTORS
#  error LUMP UART case D requires CONFIG_ARCH_RAMVECTORS
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Per-byte timeout for the TX poll loop.  At 2400 baud one byte is
 * 4.17 ms, at 460800 it is 22 µs.  10 ms gives plenty of slack at every
 * supported rate while still recovering quickly from a dead clock.
 */

#define LUMP_UART_TX_BYTE_TIMEOUT_MS   10u

/* TX frame total timeout floor (ms).  The actual cap is computed as
 * max(LUMP_UART_TX_FRAME_TIMEOUT_MIN_MS, len * char_time * 2) where
 * char_time = 10 / baud * 1000 ms.  See plan "TX timeout" note: do
 * NOT compute `10 / baud` directly — it integer-rounds to 0.  Keep the
 * multiplications in front: `len * 10 * 1000 * 2 / baud`.
 */

#define LUMP_UART_TX_FRAME_TIMEOUT_MIN_MS  50u

/* RX polling interval (ms).  Issue #100 Option A: lump_uart_isr() no
 * longer posts a wake sem per byte (that triggered watchdog-list churn
 * under 2-port LUMP mode-2 traffic — Issue #100).  Instead the kthread
 * caps each nxsem_tickwait() at this period and re-drains the ring on
 * timeout.  2 ms is well under LUMP mode-2 frame interval (10 ms) so
 * frame parsing is still timely; per-port wake rate ≒ 500 Hz, total
 * ≒ 3 kHz across 6 ports — far below the 16 kHz wd churn that broke.
 */

#define LUMP_UART_RX_POLL_MS               2u

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Per-port state (RAM, mutable).  ISR-touched fields use volatile so the
 * compiler does not hoist reads out of the critical sections in
 * `lump_uart_read_byte()` / `_flush_rx()`.
 */

struct lump_uart_state_s
{
  /* RX ring */

  uint8_t  rxring[LUMP_UART_RXRING_SIZE];
  volatile uint16_t rx_head;   /* ISR producer index (mod ring size) */
  volatile uint16_t rx_tail;   /* kthread consumer index */
  volatile uint8_t  ore_count; /* count of overrun events seen by ISR */

  /* Wakeup sem.  Issue #100 Option A: this sem is no longer used for
   * byte-arrival notification — the kthread polls the ring directly
   * via lump_uart_read_byte() with a 2 ms tickwait timeout.  Kept in
   * place so future non-RX wakes (engine fault recovery, baud change
   * confirm, etc.) can still nudge the kthread out of its short sleep.
   */

  sem_t    rx_sem;

  /* Configuration */

  const struct legoport_pin_s *pins;  /* cached on open() */
  uint32_t baud;
  bool     opened;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lump_uart_state_s g_lump_uart[BOARD_LEGOPORT_COUNT];

/****************************************************************************
 * Helpers
 ****************************************************************************/

/* Compute BRR for a given baud rate on the port's APB clock with
 * oversampling = 16.  Same formula as `stm32_btuart.c:btuart_calc_brr()`.
 *
 *   USARTDIV = pclk / (16 * baud), rounded to nearest
 *   BRR      = mantissa[15:4] | fraction[3:0]
 *
 * For 115200 @ 48 MHz  → div16 = 26  → BRR = 0x1A1
 * For 115200 @ 96 MHz  → div16 = 52  → BRR = 0x341
 */

static inline uint32_t lump_uart_brr(uint32_t pclk, uint32_t baud)
{
  uint32_t div16 = (pclk + (baud / 2)) / baud;
  uint32_t mantissa = div16 / 16;
  uint32_t fraction = div16 % 16;
  return (mantissa << 4) | (fraction & 0x0f);
}

static inline uintptr_t lump_uart_reg(int port, uint32_t offset)
{
  return g_lump_uart_hw_desc[port].usart_base + offset;
}

/****************************************************************************
 * HIPRI Direct-Vector ISRs (Issue #100 case D)
 ****************************************************************************/

/* Per-port direct vector handler core.  Runs at NVIC priority
 * NVIC_SYSH_HIGH_PRIORITY (= 0x60), above BASEPRI 0x80, so this ISR is
 * NEVER blocked by enter_critical_section().  Installed via
 * arm_ramvec_attach() in lump_uart_hipri_init() — bypasses arm_doirq()
 * / irq_dispatch() entirely.
 *
 * Bounded work:
 *   - read SR + DR (DR read also clears RXNE / ORE on F4)
 *   - push byte to SPSC ring
 *
 * MUST stay strictly OS-free: NuttX kernel APIs, semaphores, work
 * queues, syslog/printf, locks — all forbidden.  Only register/memory
 * ops.  Wake notification is via the per-port LUMP kthread polling the
 * ring (lump_uart_read_byte() with 2 ms tickwait cap).
 */

static void lump_uart_isr_core(int port)
{
  struct lump_uart_state_s *st = &g_lump_uart[port];
  uintptr_t base = g_lump_uart_hw_desc[port].usart_base;
  uint32_t sr = getreg32(base + STM32_USART_SR_OFFSET);

  while (sr & USART_SR_RXNE)
    {
      uint32_t dr = getreg32(base + STM32_USART_DR_OFFSET);
      uint8_t  byte = (uint8_t)(dr & 0xff);

      if (sr & USART_SR_ORE)
        {
          if (st->ore_count != UINT8_MAX)
            {
              st->ore_count++;
            }
        }

      uint16_t head = st->rx_head;
      uint16_t next = (head + 1u) & LUMP_UART_RXRING_MASK;

      if (next != st->rx_tail)
        {
          st->rxring[head] = byte;
          st->rx_head = next;
        }
      /* else: ring full — drop byte, LUMP checksum will detect */

      sr = getreg32(base + STM32_USART_SR_OFFSET);
    }
}

/* Per-port direct vector thunks (signature `void(*)(void)`).
 * port→UART map per stm32_legoport_lump_table.c:
 *   port A=0 → UART7, B=1 → UART4, C=2 → UART8, D=3 → UART5,
 *   E=4 → UART10, F=5 → UART9
 */

static void lump_uart_isr_port_a(void) { lump_uart_isr_core(0); }
static void lump_uart_isr_port_b(void) { lump_uart_isr_core(1); }
static void lump_uart_isr_port_c(void) { lump_uart_isr_core(2); }
static void lump_uart_isr_port_d(void) { lump_uart_isr_core(3); }
static void lump_uart_isr_port_e(void) { lump_uart_isr_core(4); }
static void lump_uart_isr_port_f(void) { lump_uart_isr_core(5); }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int lump_uart_open(int port, uint32_t baud)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct lump_uart_state_s     *st  = &g_lump_uart[port];
  const struct lump_uart_hw_desc_s *d = &g_lump_uart_hw_desc[port];

  if (st->opened)
    {
      /* Already open — only baud may change. */

      return (st->baud == baud) ? OK : lump_uart_set_baud(port, baud);
    }

  /* RCC clock is enabled by `stm32_rcc_enableperipherals()` at boot
   * (because CONFIG_STM32_UARTn=y).  Re-assert defensively in case
   * a future change drops the boot enable — this is idempotent.
   */

  modifyreg32(d->rcc_enr_reg, 0, d->rcc_enr_bit);

  /* Reset all CRn so we start from a known state. */

  putreg32(0, d->usart_base + STM32_USART_CR1_OFFSET);
  putreg32(0, d->usart_base + STM32_USART_CR2_OFFSET);
  putreg32(0, d->usart_base + STM32_USART_CR3_OFFSET);

  /* Program BRR with UE = 0 (avoids racing the baud generator). */

  putreg32(lump_uart_brr(d->apb_clk_hz, baud),
           d->usart_base + STM32_USART_BRR_OFFSET);

  /* Reset RX ring + wake state. */

  st->rx_head      = 0;
  st->rx_tail      = 0;
  st->ore_count    = 0;
  st->baud         = baud;

  /* Initialise the wakeup sem (idempotent: nxsem_init resets value). */

  nxsem_init(&st->rx_sem, 0, 0);

  /* Issue #100 case D: HIPRI direct vector is installed once at boot in
   * lump_uart_hipri_init().  Per-port open() no longer touches
   * irq_attach() — only NVIC enable is needed.
   */

  /* Enable CR1: 8N1, RX/TX/RXNEIE on, finally UE. */

  putreg32(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE,
           d->usart_base + STM32_USART_CR1_OFFSET);

  /* Issue #100 case D: HIPRI direct vector is permanent in the RAM vector
   * table.  Any stale USART RXNE/ORE latch from a previous close, or a
   * stale NVIC pending bit, would let the very next ISR push garbage
   * into the freshly-reset ring.  Drain SR/DR once and clear the NVIC
   * pending slot before enabling the IRQ.
   */

  (void)getreg32(d->usart_base + STM32_USART_SR_OFFSET);
  (void)getreg32(d->usart_base + STM32_USART_DR_OFFSET);

  {
    int extirq = d->irq - STM32_IRQ_FIRST;
    putreg32(1u << (extirq & 31), NVIC_IRQ_CLRPEND(extirq));
  }

  st->opened = true;
  up_enable_irq(d->irq);
  return OK;
}

void lump_uart_close(int port)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return;
    }

  struct lump_uart_state_s     *st  = &g_lump_uart[port];
  const struct lump_uart_hw_desc_s *d = &g_lump_uart_hw_desc[port];

  if (!st->opened)
    {
      return;
    }

  up_disable_irq(d->irq);

  /* Clear CR1 — UE off disables the peripheral.  Leave RCC clock as-is;
   * dropping the clock is unnecessary (BSS savings dwarfed by the
   * 320 KB SRAM budget) and would force a pclk re-stable wait on
   * subsequent `lump_uart_open()` calls.
   */

  putreg32(0, d->usart_base + STM32_USART_CR1_OFFSET);

  st->opened       = false;
}

int lump_uart_set_baud(int port, uint32_t baud)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return -EINVAL;
    }

  struct lump_uart_state_s     *st  = &g_lump_uart[port];
  const struct lump_uart_hw_desc_s *d = &g_lump_uart_hw_desc[port];

  if (!st->opened)
    {
      return -EBADF;
    }

  /* Issue #100 case D: HIPRI direct ISR is NOT blocked by BASEPRI
   * critical sections.  Mask the LUMP UART IRQ at NVIC ISER while the
   * UE/BRR/UE sequence runs, otherwise an in-flight ISR could observe
   * an inconsistent CR1/BRR pair or push a half-shifted byte into the
   * ring at the new baud.
   */

  up_disable_irq(d->irq);

  uint32_t cr1 = getreg32(d->usart_base + STM32_USART_CR1_OFFSET);
  putreg32(cr1 & ~USART_CR1_UE, d->usart_base + STM32_USART_CR1_OFFSET);
  putreg32(lump_uart_brr(d->apb_clk_hz, baud),
           d->usart_base + STM32_USART_BRR_OFFSET);
  putreg32(cr1 | USART_CR1_UE, d->usart_base + STM32_USART_CR1_OFFSET);

  /* Drop any byte left in RDR + clear NVIC pending so the new baud
   * starts with a clean slate (same logic as lump_uart_open()).
   */

  (void)getreg32(d->usart_base + STM32_USART_SR_OFFSET);
  (void)getreg32(d->usart_base + STM32_USART_DR_OFFSET);

  {
    int extirq = d->irq - STM32_IRQ_FIRST;
    putreg32(1u << (extirq & 31), NVIC_IRQ_CLRPEND(extirq));
  }

  up_enable_irq(d->irq);

  st->baud = baud;
  return OK;
}

int lump_uart_read_byte(int port, uint8_t *out, uint32_t timeout_ms)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT || out == NULL)
    {
      return -EINVAL;
    }

  struct lump_uart_state_s *st = &g_lump_uart[port];

  if (!st->opened)
    {
      return -EBADF;
    }

  clock_t deadline = clock_systime_ticks() + MSEC2TICK(timeout_ms);

  for (; ; )
    {
      irqstate_t flags = enter_critical_section();

      if (st->rx_head != st->rx_tail)
        {
          *out = st->rxring[st->rx_tail];
          st->rx_tail = (st->rx_tail + 1u) & LUMP_UART_RXRING_MASK;
          leave_critical_section(flags);
          return OK;
        }

      leave_critical_section(flags);

      /* Issue #100 Option A: ISR no longer posts on byte arrival, so
       * the kthread polls the ring with a short 2 ms tickwait.  If a
       * byte lands during the sleep, we discover it on the next loop;
       * worst-case latency = 2 ms (well under LUMP mode-2 frame interval
       * 10 ms).  -ETIMEDOUT here is the normal "poll period elapsed"
       * path — only the OUTER deadline check returns -ETIMEDOUT to the
       * caller.  Removing the byte-rate nxsem_post() collapses the
       * watchdog-list churn that triggered Issue #100.
       */

      sclock_t remaining = (sclock_t)(deadline - clock_systime_ticks());
      if (remaining <= 0)
        {
          return -ETIMEDOUT;
        }

      clock_t poll = MSEC2TICK(LUMP_UART_RX_POLL_MS);
      clock_t wait = ((sclock_t)poll < remaining) ? poll : (clock_t)remaining;
      if (wait == 0)
        {
          wait = 1;       /* never wait 0 ticks (busy-loop trap) */
        }

      int ret = nxsem_tickwait(&st->rx_sem, wait);
      if (ret == -ETIMEDOUT)
        {
          continue;       /* poll period elapsed — recheck ring */
        }
      if (ret == -EINTR)
        {
          return -EINTR;
        }
      /* else: woke on rx_sem post (rare, non-RX wakeup) — recheck ring */
    }
}

int lump_uart_write(int port, const uint8_t *buf, size_t len,
                    uint32_t timeout_ms)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT || buf == NULL)
    {
      return -EINVAL;
    }

  struct lump_uart_state_s     *st  = &g_lump_uart[port];
  const struct lump_uart_hw_desc_s *d = &g_lump_uart_hw_desc[port];

  if (!st->opened)
    {
      return -EBADF;
    }

  /* Compute frame total deadline.  Mind integer rounding: `10 / baud`
   * truncates to 0, so keep the multiplications in front.
   *
   *   char_time_ms = 10 * 1000 / baud
   *   total_ms     = max(timeout_ms, len * char_time_ms * 2)
   */

  uint32_t baud = (st->baud > 0) ? st->baud : 115200u;
  uint32_t computed_ms =
      (uint32_t)(((uint64_t)len * 10u * 1000u * 2u) / baud);
  uint32_t total_ms =
      timeout_ms > computed_ms ? timeout_ms : computed_ms;
  if (total_ms < LUMP_UART_TX_FRAME_TIMEOUT_MIN_MS)
    {
      total_ms = LUMP_UART_TX_FRAME_TIMEOUT_MIN_MS;
    }

  clock_t frame_deadline = clock_systime_ticks() + MSEC2TICK(total_ms);

  for (size_t i = 0; i < len; i++)
    {
      clock_t byte_deadline = clock_systime_ticks() +
                              MSEC2TICK(LUMP_UART_TX_BYTE_TIMEOUT_MS);

      while (!(getreg32(d->usart_base + STM32_USART_SR_OFFSET) &
               USART_SR_TXE))
        {
          if ((sclock_t)(clock_systime_ticks() - byte_deadline) >= 0)
            {
              return -ETIMEDOUT;
            }
          if ((sclock_t)(clock_systime_ticks() - frame_deadline) >= 0)
            {
              return -ETIMEDOUT;
            }
        }

      putreg32(buf[i], d->usart_base + STM32_USART_DR_OFFSET);
    }

  /* Wait for the last byte to fully clock out (TC) so callers can
   * change baud / pin direction safely on return.
   */

  clock_t tc_deadline = clock_systime_ticks() +
                        MSEC2TICK(LUMP_UART_TX_BYTE_TIMEOUT_MS);
  while (!(getreg32(d->usart_base + STM32_USART_SR_OFFSET) & USART_SR_TC))
    {
      if ((sclock_t)(clock_systime_ticks() - tc_deadline) >= 0)
        {
          return -ETIMEDOUT;
        }
    }

  return OK;
}

uint8_t lump_uart_get_ore_count(int port)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return 0;
    }
  return g_lump_uart[port].ore_count;
}

void lump_uart_flush_rx(int port)
{
  if (port < 0 || port >= BOARD_LEGOPORT_COUNT)
    {
      return;
    }

  struct lump_uart_state_s     *st  = &g_lump_uart[port];
  const struct lump_uart_hw_desc_s *d = &g_lump_uart_hw_desc[port];

  /* Issue #100 case D: HIPRI direct ISR vs. BASEPRI critical_section —
   * the latter does NOT mask priority 0x60.  Mask the UART IRQ at
   * NVIC ISER instead so the ring tail/head can be cleared atomically
   * with respect to the producer.
   */

  up_disable_irq(d->irq);

  st->rx_tail = st->rx_head;

  /* Read SR then DR to clear ORE / RXNE per RM0430 §28.6.1. */

  if (st->opened)
    {
      (void)getreg32(d->usart_base + STM32_USART_SR_OFFSET);
      (void)getreg32(d->usart_base + STM32_USART_DR_OFFSET);
    }

  up_enable_irq(d->irq);
}

int lump_uart_hipri_init(void)
{
  static void (* const handlers[BOARD_LEGOPORT_COUNT])(void) =
    {
      lump_uart_isr_port_a,
      lump_uart_isr_port_b,
      lump_uart_isr_port_c,
      lump_uart_isr_port_d,
      lump_uart_isr_port_e,
      lump_uart_isr_port_f,
    };

  /* Install HIPRI direct vectors for all 6 LUMP UART IRQs.  After this,
   * NVIC dispatches them straight to the per-port handlers above —
   * arm_doirq() / irq_dispatch() are bypassed.  IRQ numbers come from
   * g_lump_uart_hw_desc[] (single source of truth, also used by
   * lump_uart_open() / _close()).
   */

  for (int port = 0; port < BOARD_LEGOPORT_COUNT; port++)
    {
      int ret = arm_ramvec_attach(g_lump_uart_hw_desc[port].irq,
                                  handlers[port]);
      if (ret < 0)
        {
          return ret;
        }
    }

  return OK;
}

#ifdef CONFIG_LEGO_LUMP_DIAG
/* Read the NVIC priority register byte for an IRQ.  IPR is at 0xE000E400
 * with one byte per IRQ, so the byte at offset `irq` holds its 8-bit
 * priority (top 4 bits effective on Cortex-M4).
 */

static uint8_t lump_uart_nvic_priority(int irq)
{
  uintptr_t addr = NVIC_IRQ_PRIORITY(irq);
  uint32_t  word = getreg32(addr & ~3u);
  return (uint8_t)((word >> ((irq & 3) * 8)) & 0xff);
}

static bool lump_uart_nvic_enabled(int irq)
{
  uintptr_t addr = NVIC_IRQ_ENABLE(irq);
  uint32_t  bit  = 1u << (irq & 31);
  return (getreg32(addr) & bit) != 0;
}

void lump_uart_hw_dump(void)
{
  uint32_t apb1 = getreg32(STM32_RCC_APB1ENR);
  uint32_t apb2 = getreg32(STM32_RCC_APB2ENR);

  syslog(LOG_INFO,
         "lump-hw: RCC APB1ENR=0x%08lx UART4=%d UART5=%d UART7=%d UART8=%d\n",
         (unsigned long)apb1,
         !!(apb1 & RCC_APB1ENR_UART4EN),
         !!(apb1 & RCC_APB1ENR_UART5EN),
         !!(apb1 & RCC_APB1ENR_UART7EN),
         !!(apb1 & RCC_APB1ENR_UART8EN));
  syslog(LOG_INFO,
         "lump-hw: RCC APB2ENR=0x%08lx UART9=%d UART10=%d\n",
         (unsigned long)apb2,
         !!(apb2 & RCC_APB2ENR_UART9EN),
         !!(apb2 & RCC_APB2ENR_UART10EN));

  for (int p = 0; p < BOARD_LEGOPORT_COUNT; p++)
    {
      const struct lump_uart_hw_desc_s *d = &g_lump_uart_hw_desc[p];
      struct lump_uart_state_s         *st = &g_lump_uart[p];
      uint32_t brr = getreg32(d->usart_base + STM32_USART_BRR_OFFSET);
      uint32_t cr1 = getreg32(d->usart_base + STM32_USART_CR1_OFFSET);
      uint8_t  pri = lump_uart_nvic_priority(d->irq - NVIC_IRQ_FIRST);
      bool     en  = lump_uart_nvic_enabled(d->irq - NVIC_IRQ_FIRST);

      syslog(LOG_INFO,
             "lump-hw: port %c (IRQ %d) BRR=0x%03lx CR1=0x%04lx "
             "NVIC pri=0x%02x en=%d opened=%d baud=%lu\n",
             'A' + p, (int)d->irq,
             (unsigned long)brr, (unsigned long)cr1,
             pri, en ? 1 : 0,
             st->opened ? 1 : 0,
             (unsigned long)st->baud);
    }
}
#endif /* CONFIG_LEGO_LUMP_DIAG */

#endif /* CONFIG_LEGO_LUMP */
