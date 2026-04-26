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
 *
 * Both ioctls return -EBUSY while the sensor is active (sampling).
 */

#define LSM6DSL_IOC_SETACCELFSR  _SNIOC(0x00e0)
#define LSM6DSL_IOC_SETGYROFSR   _SNIOC(0x00e1)

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
  uint32_t timestamp;       /* Low 32 bits of CLOCK_BOOTTIME us
                             * (mod 2^32, ~71m35s wrap).  ARMv7-M 4-byte
                             * aligned word load/store is single-copy
                             * atomic, so ISR/worker handoff is tearing-
                             * free without a critical section. */
  int16_t  ax;              /* Accel X raw LSB, chip frame */
  int16_t  ay;              /* Accel Y raw LSB, chip frame */
  int16_t  az;              /* Accel Z raw LSB, chip frame */
  int16_t  gx;              /* Gyro X raw LSB, chip frame */
  int16_t  gy;              /* Gyro Y raw LSB, chip frame */
  int16_t  gz;              /* Gyro Z raw LSB, chip frame */
  int16_t  temperature_raw; /* OUT_TEMP raw, stale (refreshed every
                             * N samples by the lower half) */
  int16_t  reserved;        /* Padding, struct = 20B */
};

#endif /* __BOARDS_SPIKE_PRIME_HUB_INCLUDE_BOARD_LSM6DSL_H */
