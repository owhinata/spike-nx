/****************************************************************************
 * apps/btsensor/btsensor_wire.h
 *
 * Shared wire-format constants for the BUNDLE frame (Issue #88).
 *
 * One BUNDLE frame is emitted every 10 ms while IMU or SENSOR streaming is
 * on.  Each frame carries:
 *
 *   envelope (5 B)        magic + type + frame_len
 *   bundle hdr (16 B)     seq, tick_ts_us, IMU section length / count,
 *                         TLV count, IMU FSR / ODR snapshot, flags
 *   IMU subsection        16 B per sample, 0..BTSENSOR_IMU_SAMPLES_MAX
 *   TLV subsection        BTSENSOR_TLV_COUNT (= 6) entries, one per
 *                         legosensor uORB class; payload only when FRESH
 *
 * tick_ts_us is the absolute timestamp of the oldest IMU sample in the
 * bundle (or the run-loop's current time when imu_sample_count == 0); per
 * IMU sample ts_delta_us = sample_ts_us - tick_ts_us is therefore always
 * >= 0.  Receivers reconstruct sample_ts_abs = tick_ts_us + ts_delta_us
 * and use that for jitter analysis / Madgwick dt.
 ****************************************************************************/

#ifndef __APPS_BTSENSOR_BTSENSOR_WIRE_H
#define __APPS_BTSENSOR_BTSENSOR_WIRE_H

#include <stdint.h>

#if defined __cplusplus
extern "C" {
#endif

#define BTSENSOR_FRAME_MAGIC          0xB66B
#define BTSENSOR_FRAME_TYPE_BUNDLE    0x02

/* Phase 2.5 (#145): IMU capture frame type emitted by btsensor_imu_cap_-
 * mode.c during a Tedaldi calibration session.  Mutually exclusive with
 * BUNDLE on the BT byte stream — BUNDLE emission is paused while
 * IMU_CAP is active.  One frame per IMU sample (no batching) so the
 * host-side scanner can demux per-sample and the seq counter detects
 * any drop with single-sample granularity.
 */

#define BTSENSOR_FRAME_TYPE_IMU_CAP   0x03

#define BTSENSOR_BUNDLE_ENVELOPE_SIZE 5
#define BTSENSOR_BUNDLE_HEADER_SIZE   16
#define BTSENSOR_IMU_SAMPLE_SIZE      16
#define BTSENSOR_TLV_HEADER_SIZE      10
#define BTSENSOR_TLV_PAYLOAD_MAX      32
#define BTSENSOR_TLV_COUNT            6
#define BTSENSOR_IMU_SAMPLES_MAX      8

/* IMU_CAP payload layout (22 B, frame = envelope 5 B + payload = 27 B):
 *
 *   +0   timestamp_us (uint32 LE, low 32 bits of CLOCK_BOOTTIME at the
 *                      ISR-captured sample time)
 *   +4   ax,ay,az     (int16 LE × 3, raw LSB)
 *   +10  gx,gy,gz     (int16 LE × 3, raw LSB)
 *   +16  temp_raw     (int16 LE, LSM6DSL OUT_TEMP_L/H — Tedaldi ambient
 *                      temperature record for the cal session)
 *   +18  fsr_xl_idx   (uint8, per-sample, session-invariant — host
 *                      rejects the file if it changes mid-session)
 *   +19  fsr_gy_idx   (uint8, per-sample, session-invariant)
 *   +20  seq          (uint16 LE, wraps at 0xFFFF — ~10 min @ 104 Hz
 *                      stays unique, enough for a 240 s Tedaldi run)
 *
 * Throughput: 104 Hz × 27 B = 2.8 KB/s, comfortably inside SPP bandwidth.
 */

#define BTSENSOR_IMU_CAP_PAYLOAD_SIZE 22
#define BTSENSOR_IMU_CAP_FRAME_SIZE                                \
    (BTSENSOR_BUNDLE_ENVELOPE_SIZE + BTSENSOR_IMU_CAP_PAYLOAD_SIZE)

/* Worst case bundle size when every TLV is FRESH with full 32 B payload
 * and the IMU section is full (8 samples).  Used to size scratch buffers
 * and to assert against BTSENSOR_TX_FRAME_MAX_SIZE.
 */

#define BTSENSOR_BUNDLE_FRAME_MAX                                  \
    (BTSENSOR_BUNDLE_ENVELOPE_SIZE + BTSENSOR_BUNDLE_HEADER_SIZE   \
     + BTSENSOR_IMU_SAMPLE_SIZE * BTSENSOR_IMU_SAMPLES_MAX         \
     + (BTSENSOR_TLV_HEADER_SIZE + BTSENSOR_TLV_PAYLOAD_MAX)       \
       * BTSENSOR_TLV_COUNT)

/* Bundle header layout (16 B starting at offset 5 of the frame):
 *
 *   +0  seq               (uint16 LE)
 *   +2  tick_ts_us        (uint32 LE, low 32 bits of CLOCK_BOOTTIME)
 *   +6  imu_section_len   (uint16 LE)
 *   +8  imu_sample_count  (uint8, 0..BTSENSOR_IMU_SAMPLES_MAX)
 *   +9  tlv_count         (uint8, fixed BTSENSOR_TLV_COUNT)
 *   +10 imu_sample_rate_hz(uint16 LE)
 *   +12 imu_accel_fsr_g   (uint8, 2/4/8/16)
 *   +13 imu_gyro_fsr_dps  (uint16 LE, 125/250/500/1000/2000)
 *   +15 flags             (uint8, bit0=IMU_ON bit1=SENSOR_ON)
 */

#define BTSENSOR_FLAG_IMU_ON          0x01
#define BTSENSOR_FLAG_SENSOR_ON       0x02

/* TLV header.flags bits */

#define BTSENSOR_TLV_FLAG_BOUND       0x01
#define BTSENSOR_TLV_FLAG_FRESH       0x02

/* age_10ms saturates at this value when the last publish is far enough
 * in the past or the class has never published.
 */

#define BTSENSOR_TLV_AGE_SATURATED    0xFF

#if defined __cplusplus
}
#endif

#endif /* __APPS_BTSENSOR_BTSENSOR_WIRE_H */
