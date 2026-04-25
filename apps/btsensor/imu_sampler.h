/****************************************************************************
 * apps/btsensor/imu_sampler.h
 *
 * LSM6DS3TR-C → RFCOMM streaming for the SPIKE Prime Hub SPP app
 * (Issue #52 Step E).  Opens the uORB accel + gyro feeds, pairs incoming
 * samples into 16-sample batches and pushes them out over an RFCOMM
 * channel as the SPP server announces one open.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_IMU_SAMPLER_H
#define __APPS_BTSENSOR_IMU_SAMPLER_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/* Wire-format frame header that leads every SPP payload.  16 samples at
 * sizeof(imu_sample_s) = 12 byte follow.  Values are little-endian.
 */

#define BTSENSOR_FRAME_MAGIC       0xA55A
#define BTSENSOR_FRAME_TYPE_IMU    0x01

/* Override the runtime batch size (samples per RFCOMM frame) before
 * imu_sampler_init().  batch is clamped to [1, 80] (compile-time
 * upper bound for the per-frame sample buffer).  Lets the btsensor
 * entry point reconfigure via argv without rebuilding.
 */

void imu_sampler_configure(uint8_t batch);

/* Register the uORB accel + gyro file descriptors as btstack data
 * sources, initialise the frame ring and hook us up to the btstack run
 * loop.  Must be called after btstack_run_loop_init().
 */

int imu_sampler_init(void);

/* Called from the SPP packet handler whenever RFCOMM_EVENT_CHANNEL_OPENED
 * or RFCOMM_EVENT_CHANNEL_CLOSED arrives.  Pass cid=0 on close.
 */

void imu_sampler_set_rfcomm_cid(uint16_t rfcomm_cid, uint16_t mtu);

/* Called from the SPP packet handler on RFCOMM_EVENT_CAN_SEND_NOW. */

void imu_sampler_on_can_send_now(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_IMU_SAMPLER_H */
