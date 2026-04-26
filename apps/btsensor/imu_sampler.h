/****************************************************************************
 * apps/btsensor/imu_sampler.h
 *
 * LSM6DS3TR-C uORB -> RFCOMM frame producer.  Reads paired raw int16
 * accel + gyro samples from /dev/uorb/sensor_imu0 (Issue #56 Commit A),
 * batches them into a fixed-size IMU frame, and hands each completed
 * frame to btsensor_tx for arbitrated send.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_IMU_SAMPLER_H
#define __APPS_BTSENSOR_IMU_SAMPLER_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/* Wire-format frame header magic and type.  Reshaped in Commit E to
 * embed FSR + per-sample ts_delta; for Commit A/B these mirror the
 * existing layout so PC scripts keep parsing.
 */

#define BTSENSOR_FRAME_MAGIC       0xA55A
#define BTSENSOR_FRAME_TYPE_IMU    0x01

/* Override the runtime batch size (samples per RFCOMM frame) before
 * imu_sampler_init().  batch is clamped to [1, 80] (compile-time
 * upper bound for the per-frame sample buffer).  Lets the btsensor
 * entry point reconfigure via argv without rebuilding.
 */

void imu_sampler_configure(uint8_t batch);

/* Open /dev/uorb/sensor_imu0 and register it as a btstack data source.
 * Must be called after btstack_run_loop_init().  Returns 0 on success
 * or a negated errno; on failure the data source is not registered and
 * subsequent set_rfcomm_cid / on_can_send_now calls are no-ops.
 */

int  imu_sampler_init(void);

/* Reverse of imu_sampler_init(): remove the data source, close the fd,
 * and reset internal state so the next init() starts clean.  Safe to
 * call from the BTstack main thread; not safe from interrupt context.
 */

void imu_sampler_deinit(void);

/* Notify the sampler that the RFCOMM channel is open (cid != 0) or
 * closed (cid == 0).  Forwarded to btsensor_tx so the arbiter knows
 * where to send.
 */

void imu_sampler_set_rfcomm_cid(uint16_t rfcomm_cid, uint16_t mtu);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_IMU_SAMPLER_H */
