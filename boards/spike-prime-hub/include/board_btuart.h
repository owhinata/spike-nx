/****************************************************************************
 * boards/spike-prime-hub/include/board_btuart.h
 *
 * Public ABI for the SPIKE Prime Hub Bluetooth HCI-UART char device exposed
 * as /dev/ttyBT (Issue #52).  The node is backed by stm32_btuart.c (USART2 +
 * DMA1 S6/S7) and is intended to be consumed by a user-mode Bluetooth host
 * stack such as btstack.  See docs/{ja,en}/drivers/bluetooth.md.
 *
 * Semantics:
 *   - read(fd, buf, n): copy up to n bytes from the RX ring; blocks when the
 *     ring is empty unless the file was opened with O_NONBLOCK.
 *   - write(fd, buf, n): DMA-driven blocking write of n bytes.
 *   - poll(fd, POLLIN): POLLIN is raised whenever the RX ring is non-empty.
 *   - ioctl(fd, BTUART_IOC_SETBAUD, (unsigned long)baud): reconfigure the
 *     UART baud rate.  The caller is responsible for issuing a matching
 *     HCI_VS command to the controller first.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_BTUART_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_BTUART_H

#include <nuttx/fs/ioctl.h>

/* Device node registered by stm32_btuart_chardev_register(). */

#define BOARD_BTUART_DEVPATH   "/dev/ttyBT"

/* ioctl(fd, BTUART_IOC_SETBAUD, (unsigned long)baud) — change baud rate.
 * Uses the NuttX _BLUETOOTHIOC base so the command is unambiguous while
 * keeping clear of the numbers that the upstream BT stack used to own
 * (see nuttx/include/nuttx/wireless/bluetooth/bt_ioctl.h for the legacy
 * space, 1..27).  We pick 0x40 to leave room above.
 */

#define BTUART_IOC_SETBAUD     _BLUETOOTHIOC(0x40)

/* ioctl(fd, BTUART_IOC_CHIPRESET, 0) — pulse nSHUTD low/high to force
 * the CC2564C through a fresh ROM boot.  No argument; returns 0 on
 * success or -ENODEV if the board-level bring-up has not run.
 *
 * Issue #56 follow-up: btstack `hci_power_off` leaves the CC2564C in
 * a post-init-script state that the next btstack session cannot
 * drive back to HCI_STATE_WORKING, so the daemon issues this ioctl
 * before each `hci_init` to hand btstack a chip identical to the
 * cold-boot state.
 */

#define BTUART_IOC_CHIPRESET   _BLUETOOTHIOC(0x41)

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_BTUART_H */
