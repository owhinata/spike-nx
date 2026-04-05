/****************************************************************************
 * drivers/sensors/lsm6dsl_uorb.c
 *
 * LSM6DSL IMU uORB driver — accelerometer + gyroscope.
 * Single DRDY interrupt on INT1 reads both accel and gyro in one burst,
 * following the pybricks LSM6DS3TR-C pattern.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/nuttx.h>

#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/kmalloc.h>
#include <nuttx/kthread.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include "lsm6dsl_uorb.h"
#include <nuttx/sensors/sensor.h>
#include <nuttx/signal.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration defaults (previously from NuttX Kconfig) */

#ifndef CONFIG_LSM6DSL_UORB_I2C_FREQUENCY
#  define CONFIG_LSM6DSL_UORB_I2C_FREQUENCY    400000
#endif

#ifndef CONFIG_LSM6DSL_UORB_THREAD_STACKSIZE
#  define CONFIG_LSM6DSL_UORB_THREAD_STACKSIZE  1024
#endif

#ifndef CONFIG_LSM6DSL_UORB_ACCEL_BUFSIZE
#  define CONFIG_LSM6DSL_UORB_ACCEL_BUFSIZE     10
#endif

#ifndef CONFIG_LSM6DSL_UORB_GYRO_BUFSIZE
#  define CONFIG_LSM6DSL_UORB_GYRO_BUFSIZE      10
#endif

#define WHO_AM_I_VAL    0x6a

#define MILLIG_TO_MS2   (0.0098067f)
#define MDPS_TO_RADS    (3.141592653f / (180.0f * 1000.0f))

/* Registers */

#define DRDY_PULSE_CFG  0x0b   /* DRDY pulsed/latched config */
#define INT1_CTRL       0x0d   /* INT1 pin control */
#define WHO_AM_I        0x0f
#define CTRL1_XL        0x10   /* Accel control */
#define CTRL2_G         0x11   /* Gyro control */
#define CTRL3_C         0x12   /* Control reg 3 (BDU, IF_INC, etc.) */
#define CTRL5_C         0x14   /* Control reg 5 (rounding) */
#define STATUS_REG      0x1e
#define OUTX_L_G        0x22   /* Gyro output start (12 bytes: G+XL) */

/* Bits */

#define BIT_BDU         (1 << 6)  /* Block Data Update in CTRL3_C */
#define BIT_IF_INC      (1 << 2)  /* Auto-increment in CTRL3_C */
#define ROUNDING_GY_XL  (3 << 2)  /* Rounding for gyro+accel in CTRL5_C */
#define BIT_DRDY_PULSED (1 << 7)  /* Pulsed DRDY mode */
#define BIT_INT1_DRDY_G (1 << 1)  /* Gyro DRDY on INT1 */
#define BIT_SW_RESET    (1 << 0)  /* Software reset in CTRL3_C */

/* Burst read: 6 bytes gyro (0x22-0x27) + 6 bytes accel (0x28-0x2D) */

#define BURST_DATA_LEN  12

/****************************************************************************
 * Private Types
 ****************************************************************************/

enum lsm6dsl_odr_e
{
  ODR_OFF     = 0x0,
  ODR_12_5HZ  = 0x1,
  ODR_26HZ    = 0x2,
  ODR_52HZ    = 0x3,
  ODR_104HZ   = 0x4,
  ODR_208HZ   = 0x5,
  ODR_416HZ   = 0x6,
  ODR_833HZ   = 0x7,
  ODR_1660HZ  = 0x8,
  ODR_3330HZ  = 0x9,
  ODR_6660HZ  = 0xa,
};

enum lsm6dsl_fsr_gy_e
{
  FSR_GY_250DPS  = 0x0,
  FSR_GY_125DPS  = 0x1,
  FSR_GY_500DPS  = 0x2,
  FSR_GY_1000DPS = 0x4,
  FSR_GY_2000DPS = 0x6,
};

enum lsm6dsl_fsr_xl_e
{
  FSR_XL_2G  = 0x0,
  FSR_XL_16G = 0x1,
  FSR_XL_4G  = 0x2,
  FSR_XL_8G  = 0x3,
};

struct lsm6dsl_dev_s
{
  struct sensor_lowerhalf_s gyro_lower;
  struct sensor_lowerhalf_s accel_lower;
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  mutex_t devlock;
  sem_t run;                   /* Polling thread wakeup */
  struct work_s work;          /* HPWORK for interrupt mode */
  enum lsm6dsl_odr_e odr;     /* Shared ODR for both sensors */
  int fsr_gy;
  int fsr_xl;
  bool gyro_enabled;
  bool accel_enabled;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int lsm6dsl_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable);
static int lsm6dsl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us);
static int lsm6dsl_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, int cmd,
                            unsigned long arg);
static int lsm6dsl_get_info(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR struct sensor_device_info_s *info);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const uint32_t g_odr_interval[] =
{
  0,       /* ODR_OFF */
  80000,   /* ODR_12_5HZ */
  38462,   /* ODR_26HZ */
  19230,   /* ODR_52HZ */
  9615,    /* ODR_104HZ */
  4807,    /* ODR_208HZ */
  2403,    /* ODR_416HZ */
  1200,    /* ODR_833HZ */
  602,     /* ODR_1660HZ */
  300,     /* ODR_3330HZ */
  150,     /* ODR_6660HZ */
};

static const float g_fsr_xl_sens[] =
{
  0.061f * MILLIG_TO_MS2,  /* 2g */
  0.488f * MILLIG_TO_MS2,  /* 16g */
  0.122f * MILLIG_TO_MS2,  /* 4g */
  0.244f * MILLIG_TO_MS2,  /* 8g */
};

static const float g_fsr_gy_sens[] =
{
  8.75f * MDPS_TO_RADS,    /* 250 dps */
  4.375f * MDPS_TO_RADS,   /* 125 dps */
  17.50f * MDPS_TO_RADS,   /* 500 dps */
  0.0f,                    /* unused (3) */
  35.0f * MDPS_TO_RADS,    /* 1000 dps */
  0.0f,                    /* unused (5) */
  70.0f * MDPS_TO_RADS,    /* 2000 dps */
};

static const struct sensor_ops_s g_sensor_ops =
{
  .activate     = lsm6dsl_activate,
  .set_interval = lsm6dsl_set_interval,
  .control      = lsm6dsl_control,
  .get_info     = lsm6dsl_get_info,
};

/****************************************************************************
 * Private Functions — I2C helpers
 ****************************************************************************/

static int lsm6dsl_write_byte(FAR struct lsm6dsl_dev_s *priv,
                               uint8_t reg, uint8_t val)
{
  uint8_t buf[2] = { reg, val };
  struct i2c_msg_s msg;

  msg.frequency = CONFIG_LSM6DSL_UORB_I2C_FREQUENCY;
  msg.addr      = priv->addr;
  msg.flags     = 0;
  msg.buffer    = buf;
  msg.length    = 2;

  return I2C_TRANSFER(priv->i2c, &msg, 1);
}

static int lsm6dsl_read_bytes(FAR struct lsm6dsl_dev_s *priv,
                               uint8_t reg, FAR void *buf, size_t len)
{
  struct i2c_msg_s cmd[2];

  cmd[0].frequency = CONFIG_LSM6DSL_UORB_I2C_FREQUENCY;
  cmd[0].addr      = priv->addr;
  cmd[0].flags     = I2C_M_NOSTOP;
  cmd[0].buffer    = &reg;
  cmd[0].length    = 1;

  cmd[1].frequency = CONFIG_LSM6DSL_UORB_I2C_FREQUENCY;
  cmd[1].addr      = priv->addr;
  cmd[1].flags     = I2C_M_READ;
  cmd[1].buffer    = buf;
  cmd[1].length    = len;

  return I2C_TRANSFER(priv->i2c, cmd, 2);
}

static int lsm6dsl_set_bits(FAR struct lsm6dsl_dev_s *priv, uint8_t reg,
                             uint8_t set, uint8_t clear)
{
  int err;
  uint8_t val;

  err = lsm6dsl_read_bytes(priv, reg, &val, 1);
  if (err < 0)
    {
      return err;
    }

  val = (val & ~clear) | set;
  return lsm6dsl_write_byte(priv, reg, val);
}

/****************************************************************************
 * Private Functions — ODR / FSR
 ****************************************************************************/

static int lsm6dsl_set_odr(FAR struct lsm6dsl_dev_s *dev,
                             enum lsm6dsl_odr_e odr)
{
  int err;

  /* Set both accel and gyro to the same ODR */

  err = lsm6dsl_set_bits(dev, CTRL1_XL, (odr & 0xf) << 4, 0xf0);
  if (err < 0)
    {
      return err;
    }

  err = lsm6dsl_set_bits(dev, CTRL2_G, (odr & 0xf) << 4, 0xf0);
  if (err < 0)
    {
      return err;
    }

  dev->odr = odr;
  return OK;
}

static int accel_set_fsr(FAR struct lsm6dsl_dev_s *dev,
                          enum lsm6dsl_fsr_xl_e fsr)
{
  int err;

  err = lsm6dsl_set_bits(dev, CTRL1_XL, (fsr & 0x3) << 2, 0x0c);
  if (err < 0)
    {
      return err;
    }

  dev->fsr_xl = fsr;
  return OK;
}

static int gyro_set_fsr(FAR struct lsm6dsl_dev_s *dev,
                         enum lsm6dsl_fsr_gy_e fsr)
{
  int err;

  err = lsm6dsl_set_bits(dev, CTRL2_G, (fsr & 0x7) << 1, 0x0e);
  if (err < 0)
    {
      return err;
    }

  dev->fsr_gy = fsr;
  return OK;
}

/****************************************************************************
 * Private Functions — Data acquisition
 ****************************************************************************/

static int push_data(FAR struct lsm6dsl_dev_s *dev)
{
  int16_t raw[6];  /* gyro XYZ + accel XYZ */
  struct sensor_gyro gyro;
  struct sensor_accel accel;
  uint64_t ts;
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Burst read: 12 bytes from OUTX_L_G (0x22)
   * [0..2] = gyro X,Y,Z   [3..5] = accel X,Y,Z
   */

  err = lsm6dsl_read_bytes(dev, OUTX_L_G, raw, BURST_DATA_LEN);
  if (err < 0)
    {
      goto unlock;
    }

  ts = sensor_get_timestamp();

  gyro.timestamp   = ts;
  gyro.temperature = 0;
  gyro.x = (float)(raw[0]) * g_fsr_gy_sens[dev->fsr_gy];
  gyro.y = (float)(raw[1]) * g_fsr_gy_sens[dev->fsr_gy];
  gyro.z = (float)(raw[2]) * g_fsr_gy_sens[dev->fsr_gy];

  accel.timestamp   = ts;
  accel.temperature = 0;
  accel.x = (float)(raw[3]) * g_fsr_xl_sens[dev->fsr_xl];
  accel.y = (float)(raw[4]) * g_fsr_xl_sens[dev->fsr_xl];
  accel.z = (float)(raw[5]) * g_fsr_xl_sens[dev->fsr_xl];

  if (dev->gyro_enabled)
    {
      dev->gyro_lower.push_event(dev->gyro_lower.priv,
                                 &gyro, sizeof(gyro));
    }

  if (dev->accel_enabled)
    {
      dev->accel_lower.push_event(dev->accel_lower.priv,
                                  &accel, sizeof(accel));
    }

unlock:
  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Private Functions — Interrupt handler
 ****************************************************************************/

static void drdy_worker(FAR void *arg)
{
  push_data(arg);
}

static int drdy_int_handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lsm6dsl_dev_s *dev = (FAR struct lsm6dsl_dev_s *)arg;

  DEBUGASSERT(dev != NULL);
  work_queue(HPWORK, &dev->work, drdy_worker, dev, 0);
  return OK;
}

/****************************************************************************
 * Private Functions — Polling thread
 ****************************************************************************/

static int poll_thread(int argc, char **argv)
{
  FAR struct lsm6dsl_dev_s *dev =
      (FAR struct lsm6dsl_dev_s *)((uintptr_t)strtoul(argv[1], NULL, 16));

  while (true)
    {
      if (!dev->gyro_enabled && !dev->accel_enabled)
        {
          nxsem_wait(&dev->run);
          continue;
        }

      push_data(dev);
      nxsched_usleep(g_odr_interval[dev->odr]);
    }

  return OK;
}

/****************************************************************************
 * Private Functions — sensor_ops
 ****************************************************************************/

static int lsm6dsl_activate(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep, bool enable)
{
  FAR struct lsm6dsl_dev_s *dev;
  bool was_active;
  bool now_active;
  int err;

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, gyro_lower);
    }
  else
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, accel_lower);
    }

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  was_active = dev->gyro_enabled || dev->accel_enabled;

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      dev->gyro_enabled = enable;
    }
  else
    {
      dev->accel_enabled = enable;
    }

  now_active = dev->gyro_enabled || dev->accel_enabled;

  /* Start sampling when first sensor activates */

  if (!was_active && now_active)
    {
      err = lsm6dsl_set_odr(dev, ODR_833HZ);
      if (err < 0)
        {
          goto unlock;
        }

      /* Wake polling thread if present */

      nxsem_post(&dev->run);
    }

  /* Stop sampling when last sensor deactivates */

  if (was_active && !now_active)
    {
      err = lsm6dsl_set_odr(dev, ODR_OFF);
    }

unlock:
  nxmutex_unlock(&dev->devlock);
  return err;
}

static int lsm6dsl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  FAR struct lsm6dsl_dev_s *dev;
  enum lsm6dsl_odr_e odr;
  int err;

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, gyro_lower);
    }
  else
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, accel_lower);
    }

  if (*period_us >= 80000)
    {
      odr = ODR_12_5HZ;
    }
  else if (*period_us >= 38462)
    {
      odr = ODR_26HZ;
    }
  else if (*period_us >= 19231)
    {
      odr = ODR_52HZ;
    }
  else if (*period_us >= 9615)
    {
      odr = ODR_104HZ;
    }
  else if (*period_us >= 4808)
    {
      odr = ODR_208HZ;
    }
  else if (*period_us >= 2404)
    {
      odr = ODR_416HZ;
    }
  else if (*period_us >= 1200)
    {
      odr = ODR_833HZ;
    }
  else if (*period_us >= 602)
    {
      odr = ODR_1660HZ;
    }
  else if (*period_us >= 300)
    {
      odr = ODR_3330HZ;
    }
  else
    {
      odr = ODR_6660HZ;
    }

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* Both accel and gyro share the same ODR */

  err = lsm6dsl_set_odr(dev, odr);
  if (err >= 0)
    {
      *period_us = g_odr_interval[odr];
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

static int lsm6dsl_get_info(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR struct sensor_device_info_s *info)
{
  FAR struct lsm6dsl_dev_s *dev;

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, gyro_lower);
    }
  else
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, accel_lower);
    }

  memset(info, 0, sizeof(*info));
  info->power = 0.55f;
  memcpy(info->name, "LSM6DSL", sizeof("LSM6DSL"));
  memcpy(info->vendor, "STMicro", sizeof("STMicro"));

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      info->resolution = g_fsr_gy_sens[dev->fsr_gy];
      info->max_range  = g_fsr_gy_sens[dev->fsr_gy] * INT16_MAX;
    }
  else
    {
      info->resolution = g_fsr_xl_sens[dev->fsr_xl];
      info->max_range  = g_fsr_xl_sens[dev->fsr_xl] * INT16_MAX;
    }

  info->min_delay = (int32_t)g_odr_interval[ODR_6660HZ];
  info->max_delay = (int32_t)g_odr_interval[ODR_12_5HZ];
  return OK;
}

static int lsm6dsl_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct lsm6dsl_dev_s *dev;
  int err;

  if (lower->type == SENSOR_TYPE_GYROSCOPE)
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, gyro_lower);
    }
  else
    {
      dev = container_of(lower, struct lsm6dsl_dev_s, accel_lower);
    }

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  switch (cmd)
    {
    case SNIOC_WHO_AM_I:
      {
        FAR uint8_t *id = (FAR uint8_t *)(arg);
        if (id == NULL)
          {
            err = -EINVAL;
            break;
          }

        err = lsm6dsl_read_bytes(dev, WHO_AM_I, id, 1);
      }
      break;

    case SNIOC_SETFULLSCALE:
      {
        if (lower->type == SENSOR_TYPE_ACCELEROMETER)
          {
            switch (arg)
              {
              case 2:  err = accel_set_fsr(dev, FSR_XL_2G);  break;
              case 4:  err = accel_set_fsr(dev, FSR_XL_4G);  break;
              case 8:  err = accel_set_fsr(dev, FSR_XL_8G);  break;
              case 16: err = accel_set_fsr(dev, FSR_XL_16G); break;
              default: err = -EINVAL;                         break;
              }
          }
        else
          {
            switch (arg)
              {
              case 125:  err = gyro_set_fsr(dev, FSR_GY_125DPS);  break;
              case 250:  err = gyro_set_fsr(dev, FSR_GY_250DPS);  break;
              case 500:  err = gyro_set_fsr(dev, FSR_GY_500DPS);  break;
              case 1000: err = gyro_set_fsr(dev, FSR_GY_1000DPS); break;
              case 2000: err = gyro_set_fsr(dev, FSR_GY_2000DPS); break;
              default:   err = -EINVAL;                            break;
              }
          }
      }
      break;

    default:
      err = -EINVAL;
      break;
    }

  nxmutex_unlock(&dev->devlock);
  return err;
}

/****************************************************************************
 * Private Functions — Hardware init
 ****************************************************************************/

static int lsm6dsl_hw_init(FAR struct lsm6dsl_dev_s *dev)
{
  int err;
  uint8_t val;

  /* Software reset */

  err = lsm6dsl_write_byte(dev, CTRL3_C, BIT_SW_RESET);
  if (err < 0)
    {
      return err;
    }

  /* Wait for reset to complete */

  do
    {
      nxsig_usleep(1000);
      err = lsm6dsl_read_bytes(dev, CTRL3_C, &val, 1);
      if (err < 0)
        {
          return err;
        }
    }
  while (val & BIT_SW_RESET);

  /* Enable BDU (prevent data tearing) and IF_INC (auto-increment) */

  err = lsm6dsl_set_bits(dev, CTRL3_C, BIT_BDU | BIT_IF_INC, 0);
  if (err < 0)
    {
      return err;
    }

  /* Enable rounding for gyro+accel burst reads */

  err = lsm6dsl_set_bits(dev, CTRL5_C, ROUNDING_GY_XL, ROUNDING_GY_XL);
  if (err < 0)
    {
      return err;
    }

  /* Set DRDY to pulsed mode (more reliable than latched) */

  err = lsm6dsl_write_byte(dev, DRDY_PULSE_CFG, BIT_DRDY_PULSED);
  if (err < 0)
    {
      return err;
    }

  /* Set FSR: accel ±8g, gyro 2000 dps (matching pybricks) */

  err = accel_set_fsr(dev, FSR_XL_8G);
  if (err < 0)
    {
      return err;
    }

  err = gyro_set_fsr(dev, FSR_GY_2000DPS);
  if (err < 0)
    {
      return err;
    }

  /* Route gyro DRDY to INT1 */

  err = lsm6dsl_write_byte(dev, INT1_CTRL, BIT_INT1_DRDY_G);
  if (err < 0)
    {
      return err;
    }

  return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int lsm6dsl_register_uorb(FAR struct i2c_master_s *i2c, uint8_t addr,
                           uint8_t devno,
                           FAR struct lsm6dsl_uorb_config_s *config)
{
  FAR struct lsm6dsl_dev_s *priv;
  int err;
  FAR char *argv[2];
  char arg1[32];

  DEBUGASSERT(i2c != NULL);
  DEBUGASSERT(addr == 0x6a || addr == 0x6b);

#if !defined(CONFIG_SCHED_HPWORK)
  if (config->attach != NULL)
    {
      snerr("CONFIG_SCHED_HPWORK required for interrupt mode.\n");
      return -ENOSYS;
    }
#endif

  priv = kmm_zalloc(sizeof(struct lsm6dsl_dev_s));
  if (priv == NULL)
    {
      return -ENOMEM;
    }

  priv->i2c  = i2c;
  priv->addr = addr;
  priv->odr  = ODR_OFF;
  priv->fsr_gy = FSR_GY_2000DPS;
  priv->fsr_xl = FSR_XL_8G;

  err = nxmutex_init(&priv->devlock);
  if (err < 0)
    {
      goto free_mem;
    }

  err = nxsem_init(&priv->run, 0, 0);
  if (err < 0)
    {
      goto del_mutex;
    }

  /* Hardware init: BDU, rounding, DRDY pulse, INT1 routing */

  err = lsm6dsl_hw_init(priv);
  if (err < 0)
    {
      snerr("ERROR: LSM6DSL hw init failed: %d\n", err);
      goto del_sem;
    }

  /* Register gyro */

  priv->gyro_lower.type    = SENSOR_TYPE_GYROSCOPE;
  priv->gyro_lower.ops     = &g_sensor_ops;
  priv->gyro_lower.nbuffer = CONFIG_LSM6DSL_UORB_GYRO_BUFSIZE;

  err = sensor_register(&priv->gyro_lower, devno);
  if (err < 0)
    {
      snerr("ERROR: gyro register failed: %d\n", err);
      goto del_sem;
    }

  /* Register accel */

  priv->accel_lower.type    = SENSOR_TYPE_ACCELEROMETER;
  priv->accel_lower.ops     = &g_sensor_ops;
  priv->accel_lower.nbuffer = CONFIG_LSM6DSL_UORB_ACCEL_BUFSIZE;

  err = sensor_register(&priv->accel_lower, devno);
  if (err < 0)
    {
      snerr("ERROR: accel register failed: %d\n", err);
      goto unreg_gyro;
    }

  /* Data acquisition: interrupt or polling */

  if (config->attach != NULL)
    {
      err = config->attach(drdy_int_handler, priv);
      if (err < 0)
        {
          snerr("ERROR: INT1 attach failed: %d\n", err);
          goto unreg_accel;
        }

      sninfo("LSM6DSL using INT1 interrupt.\n");
    }
  else
    {
      snprintf(arg1, sizeof(arg1), "%p", priv);
      argv[0] = arg1;
      argv[1] = NULL;
      err = kthread_create("lsm6dsl", SCHED_PRIORITY_DEFAULT,
                           CONFIG_LSM6DSL_UORB_THREAD_STACKSIZE,
                           poll_thread, argv);
      if (err < 0)
        {
          snerr("ERROR: poll thread create failed: %d\n", err);
          goto unreg_accel;
        }

      sninfo("LSM6DSL using polling thread.\n");
    }

  sninfo("LSM6DSL driver registered!\n");
  return OK;

unreg_accel:
  sensor_unregister(&priv->accel_lower, devno);
unreg_gyro:
  sensor_unregister(&priv->gyro_lower, devno);
del_sem:
  nxsem_destroy(&priv->run);
del_mutex:
  nxmutex_destroy(&priv->devlock);
free_mem:
  kmm_free(priv);
  return err;
}
