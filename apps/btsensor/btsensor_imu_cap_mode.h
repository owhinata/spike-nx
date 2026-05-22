/****************************************************************************
 * apps/btsensor/btsensor_imu_cap_mode.h
 *
 * IMU capture mode for Tedaldi (imu_tk) offline calibration (Phase 2.5,
 * Issue #145).
 *
 * Subscribes to /dev/uorb/sensor_imu0 with its own fd, switches the
 * driver to 104 Hz, and emits one BTSENSOR_FRAME_TYPE_IMU_CAP (0x03)
 * frame per IMU sample over the existing btsensor_tx ring.  BUNDLE
 * emission is paused while IMU_CAP is active so the BT byte stream
 * carries only IMU_CAP frames — the host scanner demuxes on
 * frame_type.
 *
 * Mutually exclusive with MODE TELEMETRY / SHELL / CAPTURE; entry
 * fails with -EBUSY when one of those is already active.  Exit
 * always restores driver ODR to 833 Hz (layer 1 of the 3-layer ODR
 * rollback defense — see [[project_phase_2_5_plan]]).
 *
 * Public functions other than is_active() must run on the BTstack
 * main thread.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_IMU_CAP_MODE_H
#define __APPS_BTSENSOR_BTSENSOR_IMU_CAP_MODE_H

#include <stdbool.h>
#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

int  btsensor_imu_cap_mode_init(void);
void btsensor_imu_cap_mode_deinit(void);

/* Enter IMU_CAP mode.  duration_sec == 0 means infinite (until exit is
 * called explicitly).  Returns 0 on success, -EBUSY if already active,
 * negated errno on driver SET/open failures.
 */

int  btsensor_imu_cap_mode_enter(uint32_t duration_sec);

/* Exit IMU_CAP mode.  Idempotent: no-op when not active.  Always
 * restores driver ODR to 833 Hz on the way out.
 */

int  btsensor_imu_cap_mode_exit(void);

bool btsensor_imu_cap_mode_is_active(void);

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_IMU_CAP_MODE_H */
