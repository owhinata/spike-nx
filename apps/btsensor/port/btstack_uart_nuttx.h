/****************************************************************************
 * apps/btsensor/port/btstack_uart_nuttx.h
 *
 * btstack_uart_t backed by the NuttX /dev/ttyBT character device (Issue #52).
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_PORT_BTSTACK_UART_NUTTX_H
#define __APPS_BTSENSOR_PORT_BTSTACK_UART_NUTTX_H

#include "btstack_uart.h"

#if defined __cplusplus
extern "C" {
#endif

const btstack_uart_t *btstack_uart_nuttx_instance(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_PORT_BTSTACK_UART_NUTTX_H */
