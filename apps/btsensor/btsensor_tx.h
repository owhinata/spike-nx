/****************************************************************************
 * apps/btsensor/btsensor_tx.h
 *
 * Single RFCOMM send arbiter for btsensor.  Holds two queues — a small
 * response queue (Commit D ASCII command replies, must always make it
 * through) and a frame ringbuf (IMU telemetry, may be dropped under
 * back-pressure).  All public functions must be called from the BTstack
 * main thread (via btstack_run_loop_execute_on_main_thread() if invoked
 * from another thread).
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_TX_H
#define __APPS_BTSENSOR_BTSENSOR_TX_H

#include <stddef.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/* Maximum size of a single RFCOMM payload buffered in the frame ring.
 * Sized to cover the post-#88 BUNDLE frame layout (envelope + bundle
 * header + 8 IMU samples + 6 TLVs of full 32 B payload = 401 bytes;
 * see btsensor_wire.h::BTSENSOR_BUNDLE_FRAME_MAX) with headroom.  Each
 * ring slot allocates this many bytes statically.
 */

#define BTSENSOR_TX_FRAME_MAX_SIZE   1408

/* Maximum size of a single response line (ASCII).  Plenty for the
 * `OK` / `ERR <reason>` style replies introduced in Commit D.
 */

#define BTSENSOR_TX_RESPONSE_MAX_LEN 64

int  btsensor_tx_init(void);
void btsensor_tx_deinit(void);

/* Bind the active RFCOMM cid (0 means "channel closed").  Resets the
 * pending-can-send flag so the next enqueue retriggers the request.
 */

void btsensor_tx_set_rfcomm_cid(uint16_t cid);

/* Enqueue an ASCII response line.  Trailing newline is the caller's
 * responsibility.  Returns -ENOSPC if the response queue is full,
 * -E2BIG if the payload exceeds BTSENSOR_TX_RESPONSE_MAX_LEN.
 */

int  btsensor_tx_enqueue_response(const char *line);

/* Enqueue a telemetry frame.  If the ring is full, the oldest entry
 * is dropped to favour newer data and -ENOSPC is returned (the call
 * still succeeds in storing the new frame).  The caller's `buf` is
 * memcpy'd into the ring before this function returns, so the buffer
 * may be reused (or stack-resident) immediately after the call.
 */

int  btsensor_tx_try_enqueue_frame(const uint8_t *buf, size_t len);

/* RFCOMM_EVENT_CAN_SEND_NOW handler.  Drains one response (priority)
 * or one frame, then re-arms the can-send-now request if more is
 * pending.
 */

void btsensor_tx_on_can_send_now(void);

/* True when an RFCOMM channel is bound (cid != 0).  Lets producers
 * (e.g. imu_sampler) short-circuit frame encoding when there's no
 * consumer.
 */

bool btsensor_tx_has_consumer(void);

/* Telemetry counters.  Pass NULL for any counter you don't need.
 * - frames_sent:          successful rfcomm_send() completions
 * - frames_dropped_oldest: ring was full at enqueue time; oldest slot
 *                          was overwritten to make room (the new frame
 *                          is preserved)
 * - frames_dropped_full:   reserved for future paths that fail to
 *                          enqueue at all (currently always 0; the
 *                          drop-oldest path always succeeds)
 */

void btsensor_tx_get_stats(uint32_t *frames_sent,
                           uint32_t *frames_dropped_oldest,
                           uint32_t *frames_dropped_full);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_TX_H */
