/****************************************************************************
 * apps/btsensor/bundle_emitter.h
 *
 * 100 Hz BUNDLE frame emitter (Issue #88).
 *
 * Owns a BTstack run-loop timer that fires every CONFIG_APP_BTSENSOR_-
 * BUNDLE_TICK_MS (10 ms by default).  Each tick:
 *   - drains imu_sampler for up to BTSENSOR_IMU_SAMPLES_MAX samples,
 *   - takes a sensor_sampler_snapshot() of all 6 LEGO classes,
 *   - serialises the BUNDLE frame into a single static scratch buffer,
 *   - hands it to btsensor_tx_try_enqueue_frame() which memcpy's it.
 *
 * The timer is only armed while at least one of IMU / SENSOR is on, so
 * an idle daemon (`btsensor start` with no PC connected) doesn't burn
 * 100 Hz wake-ups.
 *
 * All public functions must run on the BTstack main thread (route via
 * btstack_run_loop_execute_on_main_thread() if called from another
 * thread, e.g. NSH dispatch in btsensor_main.c).
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BUNDLE_EMITTER_H
#define __APPS_BTSENSOR_BUNDLE_EMITTER_H

#include <stdbool.h>

#if defined __cplusplus
extern "C" {
#endif

int  bundle_emitter_init(void);
void bundle_emitter_deinit(void);

/* Toggle IMU / SENSOR streaming.  Setting either one to true arms the
 * 100 Hz timer if it isn't already running; setting both to false stops
 * the timer.  Both delegate to imu_sampler_set_enabled() /
 * sensor_sampler_set_enabled() under the hood so callers don't need to
 * reach those APIs directly.
 */

int  bundle_emitter_set_imu_enabled(bool on);
int  bundle_emitter_set_sensor_enabled(bool on);

bool bundle_emitter_is_imu_enabled(void);
bool bundle_emitter_is_sensor_enabled(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BUNDLE_EMITTER_H */
