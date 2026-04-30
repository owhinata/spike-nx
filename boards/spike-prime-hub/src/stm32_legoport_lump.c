/****************************************************************************
 * boards/spike-prime-hub/src/stm32_legoport_lump.c
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2018-2023 The Pybricks Authors
 *
 * LUMP UART protocol engine for the SPIKE Prime Hub I/O ports
 * (Issue #43).  Based on `pybricks/lib/pbio/drv/legodev/legodev_pup_uart.c`,
 * ported from Contiki protothreads to one NuttX kthread per port.
 *
 * Phase 1 scope (this commit): skeleton only.  `stm32_legoport_lump_register()`
 * is a no-op — Phase 2 fills in the per-port kthreads, DCM handoff
 * registration, and the SYNCING/INFO/ACK state machine.  The public API
 * stubs return `-ENOSYS` so motor/sensor drivers can compile and link
 * against `board_lump.h` without functional dependency yet.
 *
 * See `docs/{ja,en}/drivers/lump-protocol.md` (added in Phase 4) for
 * the full design.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <syslog.h>

#include <arch/board/board_lump.h>

#include "spike_prime_hub.h"

#include "stm32_legoport_uart_hw.h"

#ifdef CONFIG_LEGO_LUMP

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_legoport_lump_register(void)
{
  /* Phase 1: no-op.  Phase 2 will:
   *   1. Pre-create 6 kthreads (sleeping on per-port `lump_wakeup` sem)
   *   2. Register `lump_handoff_cb` for each port via
   *      `stm32_legoport_register_uart_handoff()`
   *   3. Install per-port watchdog timers (Phase 4)
   */

  syslog(LOG_INFO, "lump: engine register (Phase 1 skeleton)\n");
  return OK;
}

/****************************************************************************
 * Public API stubs (filled in Phase 2/3/4)
 ****************************************************************************/

int lump_attach(int port, const struct lump_callbacks_s *cb)
{
  UNUSED(port);
  UNUSED(cb);
  return -ENOSYS;
}

int lump_detach(int port)
{
  UNUSED(port);
  return -ENOSYS;
}

int lump_select_mode(int port, uint8_t mode)
{
  UNUSED(port);
  UNUSED(mode);
  return -ENOSYS;
}

int lump_send_data(int port, uint8_t mode, const uint8_t *buf, size_t len)
{
  UNUSED(port);
  UNUSED(mode);
  UNUSED(buf);
  UNUSED(len);
  return -ENOSYS;
}

int lump_get_info(int port, struct lump_device_info_s *out)
{
  UNUSED(port);
  UNUSED(out);
  return -ENOSYS;
}

int lump_get_status(int port, uint8_t *flags_out,
                    uint32_t *rx_bytes_out,
                    uint32_t *tx_bytes_out)
{
  UNUSED(port);
  UNUSED(flags_out);
  UNUSED(rx_bytes_out);
  UNUSED(tx_bytes_out);
  return -ENOSYS;
}

#endif /* CONFIG_LEGO_LUMP */
