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

#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

/* Wire-format frame header magic and type (Commit E).  The new magic
 * 0xB66B intentionally breaks compatibility with pre-Commit-E PC
 * scripts (which expected 0xA55A); the header now embeds the runtime
 * FSR / ODR plus a per-sample ts_delta_us so the receiver can derive
 * physical units and per-sample timing without out-of-band state.
 */

#define BTSENSOR_FRAME_MAGIC       0xB66B
#define BTSENSOR_FRAME_TYPE_IMU    0x01

/* Wire-format sizes (used by both the producer and any in-tree
 * consumers / parsers that need the layout).  See the wire-format
 * table in apps/btsensor/imu_sampler.c.
 */

#define BTSENSOR_HDR_SIZE          18
#define BTSENSOR_SAMPLE_SIZE       16

/* Initialise the sampler module.  Does NOT start sampling — call
 * imu_sampler_set_enabled(true) (or have the PC send `IMU ON`) to
 * begin streaming.  Returns 0 on success.
 */

int  imu_sampler_init(void);

/* Module shutdown.  Implicitly disables sampling first so the kernel
 * driver is dropped to OFF.  Safe from the BTstack main thread.
 */

void imu_sampler_deinit(void);

/* Toggle IMU sampling.  Returns 0 on success.  The driver is opened
 * (auto-activates) when on=true and closed (auto-deactivates) when
 * on=false; subsequent set_* / get-state calls work without an open
 * data fd because they use a transient O_WRONLY ioctl fd that does
 * not subscribe.
 */

int  imu_sampler_set_enabled(bool on);

/* Current sampling state. */

bool imu_sampler_is_enabled(void);

/* Reconfiguration helpers — return -EBUSY while sampling is enabled.
 * On ioctl failure the local cache is rolled back to the previous value
 * so reads via get-state stay consistent with the hardware.
 */

int  imu_sampler_set_odr_hz(uint32_t hz);
int  imu_sampler_set_batch(uint8_t n);
int  imu_sampler_set_accel_fsr(uint32_t g);
int  imu_sampler_set_gyro_fsr(uint32_t dps);

/* Read current cache (mirrors the live driver state).  Returns the
 * value matching the most recent successful set_*; defaults are 833 Hz
 * / 8 g / 2000 dps / Kconfig batch.
 */

uint32_t imu_sampler_get_odr_hz(void);
uint32_t imu_sampler_get_accel_fsr_g(void);
uint32_t imu_sampler_get_gyro_fsr_dps(void);
uint8_t  imu_sampler_get_batch(void);

/* Notify the sampler that the RFCOMM channel is open (cid != 0) or
 * closed (cid == 0).  Forwarded to btsensor_tx so the arbiter knows
 * where to send.
 */

void imu_sampler_set_rfcomm_cid(uint16_t rfcomm_cid, uint16_t mtu);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_IMU_SAMPLER_H */
