/****************************************************************************
 * apps/btsensor/port/btstack_run_loop_nuttx.h
 *
 * Single-threaded btstack run loop for NuttX user apps.  Drives the registered
 * data sources with poll(2) and the timer list with clock_gettime(CLOCK_MONOTONIC).
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_PORT_BTSTACK_RUN_LOOP_NUTTX_H
#define __APPS_BTSENSOR_PORT_BTSTACK_RUN_LOOP_NUTTX_H

#include "btstack_run_loop.h"

#if defined __cplusplus
extern "C" {
#endif

/* Provide a btstack_run_loop_t instance backed by NuttX poll(). */

const btstack_run_loop_t *btstack_run_loop_nuttx_get_instance(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_PORT_BTSTACK_RUN_LOOP_NUTTX_H */
