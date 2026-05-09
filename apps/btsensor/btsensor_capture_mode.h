/****************************************************************************
 * apps/btsensor/btsensor_capture_mode.h
 *
 * BT-side reader/forwarder for the /dev/btcap chardev (Issue #122).
 *
 * Mutually exclusive with MODE TELEMETRY and MODE SHELL: while CAPTURE
 * is active the IMU / LEGO sensor BUNDLE pumps are paused so the BT
 * byte stream contains only the capture session framing (BTCS + meta
 * + payload + BTCE / BTAB).  CAPTURE auto-exits when /dev/btcap signals
 * end-of-session and the daemon goes back to TELEMETRY.
 *
 * All public functions other than is_active() must run on the btstack
 * main thread.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_CAPTURE_MODE_H
#define __APPS_BTSENSOR_BTSENSOR_CAPTURE_MODE_H

#include <stdbool.h>

#if defined __cplusplus
extern "C" {
#endif

/****************************************************************************
 * Name: btsensor_capture_mode_init / deinit
 *
 * Description:
 *   Allocate / release the kernel-side resources owned by the CAPTURE
 *   handler (data source, cached fd state).  Called by the daemon
 *   start / stop sequence; idempotent.
 ****************************************************************************/

int  btsensor_capture_mode_init(void);
void btsensor_capture_mode_deinit(void);

/****************************************************************************
 * Name: btsensor_capture_mode_enter
 *
 * Description:
 *   Open /dev/btcap, fetch the registered session metadata, post the
 *   `BTCS` + meta frame, and register a btstack data source so the
 *   next reads happen asynchronously off the run-loop poll.  Returns
 *   -ENOENT (and does not change state) if no writer has registered a
 *   session yet — the caller should report the error to whoever asked
 *   for the mode change so they can retry once the writer is ready.
 *
 *   Pauses IMU / sensor BUNDLE emission for the duration of the
 *   session.  The previous on/off state is restored on exit.
 ****************************************************************************/

int btsensor_capture_mode_enter(void);

/****************************************************************************
 * Name: btsensor_capture_mode_is_active
 *
 * Description:
 *   True while a session is being drained.  Safe to call from any
 *   thread (atomic load).
 ****************************************************************************/

bool btsensor_capture_mode_is_active(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_CAPTURE_MODE_H */
