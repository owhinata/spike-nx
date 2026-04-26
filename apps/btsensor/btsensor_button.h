/****************************************************************************
 * apps/btsensor/btsensor_button.h
 *
 * BT control button (PA0 / EXTI0) input driver for btsensor.
 *
 * Hooks the board's GPIO IRQ (board_button_irq()) and dispatches debounced
 * short / long press events to the BT state machine via callbacks invoked
 * on the BTstack main thread.  The IRQ handler itself runs in interrupt
 * context, defers to LPWORK, and routes the final event through
 * btstack_run_loop_execute_on_main_thread() — short press registers when
 * the button is released before the long-press threshold; long press
 * registers when the press is still active after the threshold elapses.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_BUTTON_H
#define __APPS_BTSENSOR_BTSENSOR_BUTTON_H

#if defined __cplusplus
extern "C" {
#endif

/* Callbacks invoked on the BTstack main thread.  Must be set before
 * btsensor_button_init() so init can wire the IRQ.  Either may be NULL
 * to ignore that event class.
 */

typedef void (*btsensor_button_cb_t)(void);

void btsensor_button_set_callbacks(btsensor_button_cb_t on_short,
                                   btsensor_button_cb_t on_long);

/* Initialise the GPIO input + IRQ + LPWORK plumbing.  Returns 0 on
 * success; a negated errno on failure.
 */

int  btsensor_button_init(void);

/* Reverse of init: detach the IRQ + cancel any pending work. */

void btsensor_button_deinit(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_BUTTON_H */
