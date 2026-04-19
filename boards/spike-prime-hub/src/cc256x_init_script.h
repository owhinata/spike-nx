/****************************************************************************
 * boards/spike-prime-hub/src/cc256x_init_script.h
 *
 * Public declarations for the CC2564C Bluetooth controller firmware patch /
 * init script stored in cc256x_init_script.c.  See that file for the license
 * terms (TI Text File License) and source provenance.
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_SRC_CC256X_INIT_SCRIPT_H
#define __BOARDS_SPIKE_PRIME_HUB_SRC_CC256X_INIT_SCRIPT_H

#include <stdint.h>

/* LMP subversion number the init script targets (CC2564C Orca C ROM
 * 12.0.26 / TIInit_6.12.26 v1.4).  The board-local Bluetooth bring-up
 * code can compare this with the LMP subversion reported by the chip's
 * HCI_Read_Local_Version_Information response before streaming the script.
 */

extern const uint16_t cc256x_init_script_lmp_subversion;

/* The init script itself, a sequence of length-prefixed HCI vendor-specific
 * commands.  Streamed to the CC2564C over USART2 after nSHUTD HIGH to load
 * the firmware patch + BLE add-on, before HCI baud-rate negotiation.
 */

extern const uint8_t  cc256x_init_script[];
extern const uint32_t cc256x_init_script_size;

#endif /* __BOARDS_SPIKE_PRIME_HUB_SRC_CC256X_INIT_SCRIPT_H */
