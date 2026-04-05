/****************************************************************************
 * include/nuttx/sensors/lsm6dsl_uorb.h
 *
 * LSM6DSL IMU uORB driver interface.
 * Supports interrupt-driven (DRDY) and kthread polling modes.
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

#ifndef __INCLUDE_NUTTX_SENSORS_LSM6DSL_UORB_H
#define __INCLUDE_NUTTX_SENSORS_LSM6DSL_UORB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/ioctl.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

typedef int (*lsm6dsl_attach_t)(xcpt_t handler, FAR void *arg);

/* Configuration for the LSM6DSL uORB driver.
 * A single INT1 pin is used for gyro DRDY; accel+gyro data are read
 * together in one burst.  Set attach to NULL to use kthread polling.
 */

struct lsm6dsl_uorb_config_s
{
  lsm6dsl_attach_t attach;  /* INT1 GPIO attach (NULL for kthread) */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: lsm6dsl_register_uorb
 *
 * Description:
 *   Register the LSM6DSL as a uORB sensor with accel and gyro topics.
 *   If used with interrupts and device registration fails, it is the
 *   caller's responsibility to detach the interrupt handler.
 *
 * Input Parameters:
 *   i2c     - An instance of the I2C interface to communicate with the
 *             LSM6DSL
 *   addr    - The I2C address of the LSM6DSL (0x6a or 0x6b).
 *   devno   - The device number for the uORB topics (sensor_accel<n>)
 *   config  - Configuration for interrupt-driven or polling data fetching.
 *             Leave attach NULL to use kthread polling.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int lsm6dsl_register_uorb(FAR struct i2c_master_s *i2c, uint8_t addr,
                           uint8_t devno,
                           FAR struct lsm6dsl_uorb_config_s *config);

#endif /* __INCLUDE_NUTTX_SENSORS_LSM6DSL_UORB_H */
