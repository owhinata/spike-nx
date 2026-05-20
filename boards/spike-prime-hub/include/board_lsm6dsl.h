/****************************************************************************
 * boards/spike-prime-hub/include/board_lsm6dsl.h
 *
 * SPIKE Prime Hub LSM6DSL board-local ioctl numbers.
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LSM6DSL_H
#define __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LSM6DSL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <stddef.h>
#include <stdint.h>

#include <nuttx/sensors/ioctl.h>
#include <nuttx/uorb.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Board-local ioctls for the LSM6DSL uORB driver.  Allocated outside the
 * upstream-defined SNIOC range (0x80/0x81 are taken by SNIOC_SETFULLSCALE /
 * SNIOC_GETFULLSCALE on some sensors) to avoid collisions.
 *
 *   LSM6DSL_IOC_SETACCELFSR  arg uint32: 2 | 4 | 8 | 16          (g)
 *   LSM6DSL_IOC_SETGYROFSR   arg uint32: 125 | 250 | 500 | 1000 | 2000 (dps)
 *   SNIOC_GETSAMPLERATE      arg uint32 out: enum lsm6dsl_odr_e value
 *   LSM6DSL_IOC_GETACCELFSR  arg uint32 out: enum lsm6dsl_fsr_xl_e value
 *   LSM6DSL_IOC_GETGYROFSR   arg uint32 out: enum lsm6dsl_fsr_gy_e value
 *
 * SET ioctls take physical values and convert to driver-internal enum
 * indices.  GET ioctls return the enum index directly (consumers reverse
 * it via their own lookup table); this is symmetric with the per-sample
 * idx fields embedded in struct sensor_imu below.
 */

#define LSM6DSL_IOC_SETACCELFSR  _SNIOC(0x00e0)
#define LSM6DSL_IOC_SETGYROFSR   _SNIOC(0x00e1)
#define SNIOC_GETSAMPLERATE      _SNIOC(0x00e2)
#define LSM6DSL_IOC_GETACCELFSR  _SNIOC(0x00e3)
#define LSM6DSL_IOC_GETGYROFSR   _SNIOC(0x00e4)

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Combined accel + gyro raw sample with ISR-captured timestamp.  Defined
 * here (board-local) instead of in upstream <nuttx/uorb.h> so the fork
 * stays untouched.  The driver registers /dev/uorb/sensor_imu0 via
 * sensor_custom_register(); callers read this struct directly with a
 * single read().
 */

struct sensor_imu
{
  uint32_t timestamp;       /* +0  Low 32 bits of CLOCK_BOOTTIME us
                             * (mod 2^32, ~71m35s wrap).  ARMv7-M 4-byte
                             * aligned word load/store is single-copy
                             * atomic, so ISR/worker handoff is tearing-
                             * free without a critical section. */
  int16_t  ax;              /* +4  Accel X raw LSB, Hub body frame */
  int16_t  ay;              /* +6  Accel Y raw LSB, Hub body frame */
  int16_t  az;              /* +8  Accel Z raw LSB, Hub body frame */
  int16_t  gx;              /* +10 Gyro X raw LSB, Hub body frame */
  int16_t  gy;              /* +12 Gyro Y raw LSB, Hub body frame */
  int16_t  gz;              /* +14 Gyro Z raw LSB, Hub body frame */
  int16_t  temperature_raw; /* +16 OUT_TEMP raw, stale (refreshed every
                             * N samples by the lower half) */
  uint8_t  odr_idx;         /* +18 enum lsm6dsl_odr_e value (0..0xA);
                             * lets consumers integrate / unscale across
                             * live ODR changes without an extra ioctl. */
  uint8_t  fsr_xl_idx;      /* +19 enum lsm6dsl_fsr_xl_e value (0..3).
                             * Sparse: 0=2g, 1=16g, 2=4g, 3=8g. */
  uint8_t  fsr_gy_idx;      /* +20 enum lsm6dsl_fsr_gy_e value (0..6).
                             * Sparse: 0=250, 1=125, 2=500, 4=1000,
                             * 6=2000 dps (3/5/7 unused). */
  uint8_t  reserved[3];     /* +21..+23 explicit padding to 24 B.  Array
                             * (not single byte) so push_data() can
                             * memset() all three bytes; a lone uint8_t
                             * would leave +22/+23 as compiler-implicit
                             * padding that the publish path cannot zero
                             * via field assignment. */
};                          /* sizeof = 24, alignment = 4 (u32 timestamp) */

#if defined(__cplusplus)
static_assert(sizeof(struct sensor_imu) == 24,
              "sensor_imu wire ABI: total size must stay 24 B");
static_assert(offsetof(struct sensor_imu, odr_idx) == 18,
              "sensor_imu wire ABI: odr_idx offset");
static_assert(offsetof(struct sensor_imu, fsr_xl_idx) == 19,
              "sensor_imu wire ABI: fsr_xl_idx offset");
static_assert(offsetof(struct sensor_imu, fsr_gy_idx) == 20,
              "sensor_imu wire ABI: fsr_gy_idx offset");
#else
_Static_assert(sizeof(struct sensor_imu) == 24,
               "sensor_imu wire ABI: total size must stay 24 B");
_Static_assert(offsetof(struct sensor_imu, odr_idx) == 18,
               "sensor_imu wire ABI: odr_idx offset");
_Static_assert(offsetof(struct sensor_imu, fsr_xl_idx) == 19,
               "sensor_imu wire ABI: fsr_xl_idx offset");
_Static_assert(offsetof(struct sensor_imu, fsr_gy_idx) == 20,
               "sensor_imu wire ABI: fsr_gy_idx offset");
#endif

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LSM6DSL_H */
