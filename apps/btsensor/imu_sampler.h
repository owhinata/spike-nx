/****************************************************************************
 * apps/btsensor/imu_sampler.h
 *
 * LSM6DS3TR-C uORB drain helper.  Subscribes to /dev/uorb/sensor_imu0,
 * pulls every available sample into a private ring on each btstack data
 * source read callback, and exposes a non-blocking drain API for
 * bundle_emitter (Issue #88) to assemble the IMU section of each 100 Hz
 * BUNDLE frame.  Framing / TX arbitration is no longer this module's job.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_IMU_SAMPLER_H
#define __APPS_BTSENSOR_IMU_SAMPLER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <arch/board/board_lsm6dsl.h>

#if defined __cplusplus
extern "C" {
#endif

/* Initialise the sampler module.  Does NOT open the IMU fd — call
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
 *
 * imu_sampler_set_odr_hz() rejects values >833 Hz with -EINVAL: btsensor
 * runs a 100 Hz BUNDLE tick and caps imu_sample_count per frame at 8, so
 * higher ODRs would produce >8 samples per 10 ms window and we would
 * silently drop most of them.  Use the local `btsensor dump` path for
 * higher rate IMU work.
 */

int  imu_sampler_set_odr_hz(uint32_t hz);
int  imu_sampler_set_accel_fsr(uint32_t g);
int  imu_sampler_set_gyro_fsr(uint32_t dps);

/* Read current cache (mirrors the live driver state).  Defaults are
 * 833 Hz / 8 g / 2000 dps.
 */

uint32_t imu_sampler_get_odr_hz(void);
uint32_t imu_sampler_get_accel_fsr_g(void);
uint32_t imu_sampler_get_gyro_fsr_dps(void);

/* Drain up to `max` samples from the private ring into `out` (oldest
 * first), returning the count.  Called from the BTstack run loop
 * (bundle_emitter tick).  Samples that don't fit in this drain remain
 * in the ring for the next tick — bundle_emitter's 8-cap is meant to
 * shape the bundle; we keep oldest-wins behaviour internally if the
 * ring overflows (32 slots, ~38 ms at 833 Hz).
 */

size_t   imu_sampler_drain(struct sensor_imu *out, size_t max);

/* Low 32 bits of CLOCK_BOOTTIME us.  Same time base as the driver's
 * sensor_imu.timestamp; bundle_emitter uses this for tick_ts_us when
 * the IMU section is empty.
 */

uint32_t imu_sampler_now_us(void);

/* Notify the sampler that the RFCOMM channel is open (cid != 0) or
 * closed (cid == 0).  Forwarded to btsensor_tx so the arbiter knows
 * where to send.
 */

void imu_sampler_set_rfcomm_cid(uint16_t rfcomm_cid, uint16_t mtu);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_IMU_SAMPLER_H */
