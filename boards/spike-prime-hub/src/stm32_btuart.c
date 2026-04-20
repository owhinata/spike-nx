/****************************************************************************
 * boards/spike-prime-hub/src/stm32_btuart.c
 *
 * Board-local USART2 lower-half driver for the CC2564C HCI UART link.
 *
 * This driver provides a `struct btuart_lowerhalf_s` suitable for pairing
 * with NuttX's generic Bluetooth UART upper half (CONFIG_BLUETOOTH_UART_OTHER
 * / CONFIG_BLUETOOTH_UART_GENERIC) via `btuart_create()` / `btuart_register()`.
 *
 * Why this lives in the board tree rather than leveraging NuttX upstream:
 *   - drivers/wireless/bluetooth/bt_uart_cc2564.c assumes a firmware blob
 *     that the upstream tree does not ship (Issue #47 Step A findings).
 *   - arch/arm/src/stm32/stm32_hciuart.c has multiple compile blockers in
 *     the current NuttX submodule, and upstream fixes are out of scope.
 * Keeping the implementation board-local avoids touching the NuttX fork
 * and keeps CC2564C quirks (slow clock, nSHUTD, HCI baud negotiation) in
 * one file.
 *
 * Coexistence with NuttX's own USART2 serial driver:
 *   The Kconfig choice under CONFIG_STM32_USART2 defaults to
 *   CONFIG_STM32_USART2_SERIALDRIVER, which registers /dev/ttyS2 and
 *   configures USART2 during arm_serialinit().  We let that stand because
 *   Step D on its own does not start the Bluetooth chip, so /dev/ttyS2
 *   is registered but nothing actively uses it.  Once
 *   stm32_btuart_instantiate() runs in Step E, we reconfigure USART2
 *   registers directly (UE off, CR1/CR3/BRR rewritten, DMA re-linked);
 *   the stale /dev/ttyS2 node stays in /dev but any write to it would
 *   collide with the BT traffic.  Users should not open /dev/ttyS2 once
 *   Bluetooth is up.
 *
 * Hardware wiring (see docs/{ja,en}/hardware/pin-mapping.md):
 *   TX     = PD5 AF7
 *   RX     = PD6 AF7
 *   CTS    = PD3 AF7 (HW flow control required)
 *   RTS    = PD4 AF7
 *   TX DMA = DMA1 Stream 6 Channel 4 (RM0430 Rev 9 Table 30, VERY_HIGH)
 *   RX DMA = DMA1 Stream 7 Channel 6 (RM0430 Rev 9 Table 30, VERY_HIGH)
 *   NVIC   = 0xA0 (USART2 + DMA1S6 + DMA1S7, assigned in stm32_bringup.c)
 *
 * Implementation status (Issue #47 Step D — complete):
 *   D.2: skeleton, USART2 init, TX one-shot DMA with blocking write.
 *   D.3: RX circular DMA (DMA1 S7/Ch6) + software ring + USART2 IDLE
 *        notification + coalescing latch + NDTR snapshot.
 *   D.4: setbaud TX-quiescent / RX-DMA stop-restart sequence (matches
 *        the HCI 0xFF36 baud-negotiation flow in the bring-up driver).
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/wireless/bluetooth/bt_uart.h>

#include <arch/board/board.h>

#include "arm_internal.h"
#include "stm32.h"
#include "stm32_dma.h"
#include "hardware/stm32f40xxx_uart.h"
#include "hardware/stm32f40xxx_rcc.h"

#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_USART2

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* USART2 lives on APB1 = 48 MHz */

#define BTUART_PCLK_HZ         STM32_PCLK1_FREQUENCY
#define BTUART_INITIAL_BAUD    115200
#define BTUART_PERIPHADDR      STM32_USART2_DR

/* DMA transfer size limit for one-shot TX.  Any HCI command / init-script
 * chunk we stream is well under this.
 */

#define BTUART_TX_MAX          1024

/* RX ring buffer size.  Chosen as a power of two so producer/consumer
 * arithmetic is a simple bitmask.  HCI events are <= 257 bytes and the
 * CC2564C init script commands are similar, so 512 B gives us plenty of
 * margin while staying small enough to keep rxcallback latency low.
 */

#define BTUART_RXRING_SIZE     512u
#define BTUART_RXRING_MASK     (BTUART_RXRING_SIZE - 1u)

/* Common DMA_SCR flags for both RX and TX paths:
 *   - VERY_HIGH priority (pybricks parity)
 *   - 8-bit source / destination
 *   - No FIFO (direct mode)
 *   - No burst
 */

#define BTUART_DMA_SCR_COMMON  (DMA_SCR_PRIVERYHI | DMA_SCR_MSIZE_8BITS | \
                                DMA_SCR_PSIZE_8BITS)

/* TX: memory -> peripheral, memory increments */

#define BTUART_DMA_SCR_TX      (BTUART_DMA_SCR_COMMON | DMA_SCR_MINC | \
                                DMA_SCR_DIR_M2P)

/* RX: peripheral -> memory, memory increments, circular */

#define BTUART_DMA_SCR_RX      (BTUART_DMA_SCR_COMMON | DMA_SCR_MINC | \
                                DMA_SCR_DIR_P2M | DMA_SCR_CIRC)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct stm32_btuart_s
{
  /* Upper-half-facing interface (must be first so the upper half can cast
   * struct btuart_lowerhalf_s * to struct stm32_btuart_s *).
   */

  struct btuart_lowerhalf_s lower;

  /* RX callback plumbing. */

  btuart_rxcallback_t       rxcb;
  FAR void                 *rxcb_arg;
  bool                      rxenabled;

  /* Single-shot "upper half has been notified, waiting for it to drain"
   * latch.  Cleared in btuart_read() when the ring is fully consumed.
   * Provides coalescing so back-to-back USART2 IDLE events don't swamp
   * the upper-half's single work_s with -EBUSY work_queue() failures.
   * Accessed from both thread and ISR context -> guarded by
   * enter_critical_section() at the clear site.
   */

  bool                      rxwork_pending;

  /* DMA handles */

  DMA_HANDLE                txdma;
  DMA_HANDLE                rxdma;

  /* RX consumer index into g_rxring.  The producer is derived on demand
   * from stm32_dmaresidual(rxdma), so we never store it.
   */

  size_t                    rx_consumer;

  /* TX serialization + completion */

  mutex_t                   txlock;
  sem_t                     txdone;
  int                       txresult;

  /* Current baud rate (so setbaud() and diagnostics agree) */

  uint32_t                  baud;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct stm32_btuart_s g_btuart;

/* DMA target for the RX circular buffer.  Kept out of the device struct so
 * its alignment is under our explicit control and it lives in a known
 * rw data section.
 */

static uint8_t g_rxring[BTUART_RXRING_SIZE];


/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Compute BRR value for a given baud rate on PCLK1 = 48 MHz with
 * oversampling by 16 (CR1.OVER8 = 0).  Matches the formula in RM0430
 * section "Baud rate generation".
 *
 *   USARTDIV = PCLK / (16 * baud)
 *   BRR      = mantissa[15:4] | fraction[3:0] where fraction = frac * 16
 *
 * For 115200 @ 48 MHz  -> 26.0417 -> BRR = 0x1A1
 * For 3_000_000 @ 48 MHz -> 1.0    -> BRR = 0x010
 */

static uint32_t btuart_calc_brr(uint32_t baud)
{
  uint32_t div16 = (BTUART_PCLK_HZ + (baud / 2)) / baud;  /* round nearest */
  uint32_t mantissa = div16 / 16;
  uint32_t fraction = div16 % 16;
  return (mantissa << 4) | (fraction & 0x0f);
}

static void btuart_apply_baud(uint32_t baud)
{
  uint32_t cr1 = getreg32(STM32_USART2_CR1);

  /* Stop the peripheral before touching BRR.  This is the canonical
   * hard-path on F4 — safer than racing the baud generator while the
   * chip streams data.
   */

  putreg32(cr1 & ~USART_CR1_UE, STM32_USART2_CR1);
  putreg32(btuart_calc_brr(baud), STM32_USART2_BRR);
  putreg32(cr1 | USART_CR1_UE, STM32_USART2_CR1);

  g_btuart.baud = baud;
}

static void btuart_usart_init(uint32_t baud)
{
  /* Enable peripheral clock.  USART2 is on APB1ENR bit 17 (RM0430 7.3.11). */

  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_USART2EN);

  /* Configure GPIOs.  All four are AF7. */

  stm32_configgpio(GPIO_USART2_TX);
  stm32_configgpio(GPIO_USART2_RX);
  stm32_configgpio(GPIO_USART2_CTS);
  stm32_configgpio(GPIO_USART2_RTS);

  /* Clean reset: UE cleared first so we can safely write the other regs. */

  putreg32(0, STM32_USART2_CR1);
  putreg32(0, STM32_USART2_CR2);
  putreg32(0, STM32_USART2_CR3);

  /* CR3: enable HW flow control and request DMA on both TX and RX.
   * TX DMA is used immediately (this commit).  RX DMAR is set here so
   * the bring-up HCI exchanges (which happen before the upper half
   * registers) already benefit from it — RX ring logic lands in D.3.
   */

  putreg32(USART_CR3_RTSE | USART_CR3_CTSE | USART_CR3_DMAT | USART_CR3_DMAR,
           STM32_USART2_CR3);

  /* Baud.  Leaves UE = 0 at this point; we turn the peripheral on below. */

  putreg32(btuart_calc_brr(baud), STM32_USART2_BRR);

  /* CR1: 8N1, enable TX/RX, finally UE. */

  putreg32(USART_CR1_UE | USART_CR1_TE | USART_CR1_RE, STM32_USART2_CR1);

  g_btuart.baud = baud;
}

/* TX DMA completion handler.  Called from STM32_IRQ_DMA1S6 context. */

static void btuart_tx_dma_callback(DMA_HANDLE handle, uint8_t status,
                                   FAR void *arg)
{
  FAR struct stm32_btuart_s *priv = arg;

  /* DMA_STATUS_ERROR bits are latched; capture them for the caller. */

  priv->txresult = (status & DMA_STATUS_ERROR) ? -EIO : 0;
  nxsem_post(&priv->txdone);
}

/* Snapshot the RX producer index from the DMA residual counter.  NDTR is
 * the number of *remaining* transfers, so the producer position is
 * (RXRING_SIZE - residual).  Callers must treat this as an upper bound on
 * how much has been received so far — see docs/{ja,en}/drivers/bluetooth.md
 * for the single-threaded-consumer rationale.
 */

static size_t btuart_rx_producer(FAR struct stm32_btuart_s *priv)
{
  size_t residual = stm32_dmaresidual(priv->rxdma);

  if (residual > BTUART_RXRING_SIZE)
    {
      /* Shouldn't happen; defensively clamp */

      return priv->rx_consumer;
    }

  return (BTUART_RXRING_SIZE - residual) & BTUART_RXRING_MASK;
}

/* Call the upper-half rxcallback if enabled and not already pending.
 * Usable from both ISR and thread context.
 */

static void btuart_notify_rx(FAR struct stm32_btuart_s *priv)
{
  if (!priv->rxenabled || priv->rxcb == NULL)
    {
      return;
    }

  if (priv->rxwork_pending)
    {
      return;  /* upper still draining the ring */
    }

  if (btuart_rx_producer(priv) == priv->rx_consumer)
    {
      return;  /* empty — nothing to announce */
    }

  priv->rxwork_pending = true;
  priv->rxcb(&priv->lower, priv->rxcb_arg);
}

/* USART2 IDLE interrupt handler.  The IDLE flag latches when the line is
 * quiet for a frame time; clearing it requires the read-SR-then-read-DR
 * sequence.  We use IDLE (not DMA HT/TC) as the primary notification source
 * so small HCI events (Command Complete, 7 bytes) surface quickly instead
 * of waiting for the 256-byte half-transfer point.
 */

static int btuart_usart_isr(int irq, FAR void *context, FAR void *arg)
{
  FAR struct stm32_btuart_s *priv = &g_btuart;
  uint32_t sr = getreg32(STM32_USART2_SR);

  if ((sr & USART_SR_IDLE) != 0)
    {
      /* Canonical IDLE clear: read SR then DR.  The DR read also mops up
       * any byte that sneaked into the shift register after the DMA stop,
       * though in circular mode the DMA keeps draining so that's rare.
       */

      (void)getreg32(STM32_USART2_DR);
      btuart_notify_rx(priv);
    }

  /* ORE / FE / NE are also sticky in SR; clear them by the same SR->DR
   * sequence so they don't keep re-triggering the IRQ.  They are not
   * surfaced to the upper half today — at 3 Mbps with VERY_HIGH DMA
   * priority they have not been observed in practice.
   */

  if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0)
    {
      (void)getreg32(STM32_USART2_DR);
    }

  return OK;
}

/****************************************************************************
 * Lower-half methods (partial — full set lands in D.3/D.4)
 ****************************************************************************/

static void btuart_rxattach(FAR const struct btuart_lowerhalf_s *lower,
                            btuart_rxcallback_t callback, FAR void *arg)
{
  FAR struct stm32_btuart_s *priv = (FAR struct stm32_btuart_s *)lower;
  irqstate_t flags = enter_critical_section();

  priv->rxcb     = callback;
  priv->rxcb_arg = arg;

  /* Detaching (callback == NULL) also clears any pending latch so a later
   * re-attach starts fresh.
   */

  if (callback == NULL)
    {
      priv->rxwork_pending = false;
    }

  leave_critical_section(flags);
}

static void btuart_rxenable(FAR const struct btuart_lowerhalf_s *lower,
                            bool enable)
{
  FAR struct stm32_btuart_s *priv = (FAR struct stm32_btuart_s *)lower;
  irqstate_t flags;

  if (enable == priv->rxenabled)
    {
      return;
    }

  flags = enter_critical_section();
  priv->rxenabled = enable;

  if (enable)
    {
      /* Arm the IDLE interrupt.  DMA RX itself is always running; IDLE
       * gates the notification side so we don't invoke the callback while
       * the upper half is between attach and enable.
       */

      modifyreg32(STM32_USART2_CR1, 0, USART_CR1_IDLEIE);
    }
  else
    {
      modifyreg32(STM32_USART2_CR1, USART_CR1_IDLEIE, 0);
    }

  leave_critical_section(flags);
}

static int btuart_setbaud(FAR const struct btuart_lowerhalf_s *lower,
                          uint32_t baud)
{
  FAR struct stm32_btuart_s *priv = (FAR struct stm32_btuart_s *)lower;
  int ret;

  if (baud == priv->baud)
    {
      return OK;
    }

  /* Match pybricks bluetooth_btstack_uart_block_stm32_hal.c
   * set_baudrate(): write BRR directly, leave UE on, don't touch DMA.
   * The peripheral picks up the new divisor on the next idle period.
   *
   * Toggling UE or stopping the RX DMA around the BRR write creates a
   * short window (a few microseconds at 3 Mbps) where the receiver is
   * armed but the DMA is not, which empirically makes the CC2564C
   * report Hardware Error 0x06 (Event_Not_Served_Time_Out) shortly
   * after the switch.  We still serialize against any in-flight TX so
   * the change does not corrupt a byte being shifted out.
   */

  ret = nxmutex_lock(&priv->txlock);
  if (ret < 0)
    {
      return ret;
    }

  while ((getreg32(STM32_USART2_SR) & USART_SR_TC) == 0)
    {
      /* Spin until TX is quiescent. */
    }

  putreg32(btuart_calc_brr(baud), STM32_USART2_BRR);
  priv->baud = baud;

  nxmutex_unlock(&priv->txlock);
  return OK;
}

static ssize_t btuart_read(FAR const struct btuart_lowerhalf_s *lower,
                           FAR void *buffer, size_t buflen)
{
  FAR struct stm32_btuart_s *priv = (FAR struct stm32_btuart_s *)lower;
  FAR uint8_t *dst = buffer;
  size_t consumer;
  size_t producer;
  size_t available;
  size_t copy;
  size_t first;

  if (buflen == 0)
    {
      return 0;
    }

  consumer = priv->rx_consumer;
  producer = btuart_rx_producer(priv);

  if (producer >= consumer)
    {
      available = producer - consumer;
    }
  else
    {
      available = BTUART_RXRING_SIZE - (consumer - producer);
    }

  if (available == 0)
    {
      /* Ring is drained.  Re-arm notification under IRQ-off so the next
       * IDLE can post a fresh callback.  Re-check producer inside the
       * critical section so we don't race with a byte that arrived
       * between our load and the clear.
       */

      irqstate_t flags = enter_critical_section();
      if (btuart_rx_producer(priv) == priv->rx_consumer)
        {
          priv->rxwork_pending = false;
        }

      leave_critical_section(flags);
      return 0;
    }

  copy  = (available < buflen) ? available : buflen;
  first = BTUART_RXRING_SIZE - consumer;
  if (first >= copy)
    {
      memcpy(dst, &g_rxring[consumer], copy);
    }
  else
    {
      memcpy(dst, &g_rxring[consumer], first);
      memcpy(dst + first, &g_rxring[0], copy - first);
    }

  priv->rx_consumer = (consumer + copy) & BTUART_RXRING_MASK;
  return (ssize_t)copy;
}

static ssize_t btuart_write(FAR const struct btuart_lowerhalf_s *lower,
                            FAR const void *buffer, size_t buflen)
{
  FAR struct stm32_btuart_s *priv = (FAR struct stm32_btuart_s *)lower;
  int ret;

  if (buflen == 0)
    {
      return 0;
    }

  if (buflen > BTUART_TX_MAX)
    {
      return -E2BIG;
    }

  ret = nxmutex_lock(&priv->txlock);
  if (ret < 0)
    {
      return ret;
    }

  priv->txresult = -EIO;  /* overwritten by callback on success */

  stm32_dmasetup(priv->txdma, BTUART_PERIPHADDR, (uint32_t)buffer,
                 buflen, BTUART_DMA_SCR_TX);

  stm32_dmastart(priv->txdma, btuart_tx_dma_callback, priv, false);

  nxsem_wait_uninterruptible(&priv->txdone);
  stm32_dmastop(priv->txdma);

  ret = priv->txresult;
  nxmutex_unlock(&priv->txlock);

  if (ret < 0)
    {
      return ret;
    }

  return (ssize_t)buflen;
}

static ssize_t btuart_rxdrain(FAR const struct btuart_lowerhalf_s *lower)
{
  FAR struct stm32_btuart_s *priv = (FAR struct stm32_btuart_s *)lower;
  irqstate_t flags = enter_critical_section();

  /* Discard whatever the DMA has produced so far and rearm the coalescing
   * latch.  The upper half calls this during btuart_open() to guarantee a
   * clean slate after our bring-up-time script exchanges.
   */

  priv->rx_consumer     = btuart_rx_producer(priv);
  priv->rxwork_pending  = false;

  leave_critical_section(flags);
  return 0;
}

static int btuart_ioctl(FAR const struct btuart_lowerhalf_s *lower,
                        int cmd, unsigned long arg)
{
  UNUSED(lower);
  UNUSED(cmd);
  UNUSED(arg);

  /* No board-specific ioctls yet. */

  return -ENOTTY;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_btuart_instantiate
 *
 * Description:
 *   Initialise USART2 + DMA for the CC2564C HCI link and return a
 *   struct btuart_lowerhalf_s the caller (stm32_bluetooth_initialize()) can
 *   use to stream the init-script and eventually hand to btuart_register().
 *
 *   This function *must* be called before the chip reset sequence (nSHUTD
 *   LOW -> HIGH) so the UART is already accepting traffic when the chip
 *   finishes its ROM boot.
 *
 * Returned Value:
 *   Pointer to the lower-half instance, or NULL on failure.
 *
 ****************************************************************************/

FAR struct btuart_lowerhalf_s *stm32_btuart_instantiate(void)
{
  FAR struct stm32_btuart_s *priv = &g_btuart;

  if (priv->txdma != NULL)
    {
      /* Already set up — callers may invoke this more than once during
       * bring-up retries.
       */

      return &priv->lower;
    }

  memset(priv, 0, sizeof(*priv));

  priv->lower.rxattach = btuart_rxattach;
  priv->lower.rxenable = btuart_rxenable;
  priv->lower.setbaud  = btuart_setbaud;
  priv->lower.read     = btuart_read;
  priv->lower.write    = btuart_write;
  priv->lower.rxdrain  = btuart_rxdrain;
  priv->lower.ioctl    = btuart_ioctl;

  nxmutex_init(&priv->txlock);
  nxsem_init(&priv->txdone, 0, 0);

  btuart_usart_init(BTUART_INITIAL_BAUD);

  priv->txdma = stm32_dmachannel(DMACHAN_USART2_BT_TX);
  if (priv->txdma == NULL)
    {
      syslog(LOG_ERR, "BTUART: TX DMA alloc failed\n");
      goto err;
    }

  priv->rxdma = stm32_dmachannel(DMACHAN_USART2_BT_RX);
  if (priv->rxdma == NULL)
    {
      syslog(LOG_ERR, "BTUART: RX DMA alloc failed\n");
      stm32_dmafree(priv->txdma);
      priv->txdma = NULL;
      goto err;
    }

  /* Start the RX circular DMA.  Runs forever; producer is tracked through
   * stm32_dmaresidual(rxdma) / NDTR.  No HT/TC callback is registered —
   * notification is driven by USART2 IDLE so small HCI events are
   * surfaced immediately (see btuart_usart_isr).
   */

  stm32_dmasetup(priv->rxdma, BTUART_PERIPHADDR, (uint32_t)g_rxring,
                 BTUART_RXRING_SIZE, BTUART_DMA_SCR_RX);
  stm32_dmastart(priv->rxdma, NULL, NULL, false);
  priv->rx_consumer    = 0;
  priv->rxwork_pending = false;

  /* Hook USART2 IRQ.  IDLEIE itself is left off until btuart_rxenable(true)
   * so we don't fire the callback into a NULL rxcb during bring-up.
   */

  irq_attach(STM32_IRQ_USART2, btuart_usart_isr, priv);
  up_enable_irq(STM32_IRQ_USART2);

  return &priv->lower;

err:
  nxsem_destroy(&priv->txdone);
  nxmutex_destroy(&priv->txlock);
  return NULL;
}

#endif /* CONFIG_STM32_USART2 */
