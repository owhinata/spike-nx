/****************************************************************************
 * apps/btsensor/btsensor_led.h
 *
 * BT LED control wrapping /dev/rgbled0 (TLC5955 channels BT_B / BT_G /
 * BT_R = 18 / 19 / 20).  All public functions must be called from the
 * BTstack main thread (the slow-blink helper schedules a btstack timer
 * which is not thread-safe).
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_LED_H
#define __APPS_BTSENSOR_BTSENSOR_LED_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/* Open /dev/rgbled0 and initialise internal timer state.  Returns 0 on
 * success or a negated errno; on failure subsequent led_* calls are
 * silent no-ops.
 */

int  btsensor_led_init(void);

/* Reverse of init: stop any running blink timer and close the fd. */

void btsensor_led_deinit(void);

/* Force the LED off (cancel any blink in progress). */

void btsensor_led_off(void);

/* Solid blue, full brightness. */

void btsensor_led_solid_blue(void);

/* Slow blue blink (50% duty) at the given period.  Re-arming with a
 * new period is allowed; calling with period_ms == 0 is equivalent to
 * btsensor_led_off().
 */

void btsensor_led_blink_blue(uint16_t period_ms);

/* Double-blink (Issue #73): two short ON pulses + a long OFF rest,
 * repeating with the given total period.  Used for BT_CONNECTABLE so
 * the rhythm is visually distinct from BT_ADVERTISING's symmetric
 * blink.  period_ms == 0 -> off; period_ms < 400 -> falls back to
 * btsensor_led_blink_blue() (the two short pulses + gap need at least
 * ~300 ms, so anything tighter would not be a recognisable double).
 */

void btsensor_led_double_blink_blue(uint16_t period_ms);

/* N short pulses (~150 ms on / 150 ms off) then leave the LED off. */

void btsensor_led_fail_blink(uint8_t count);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_LED_H */
