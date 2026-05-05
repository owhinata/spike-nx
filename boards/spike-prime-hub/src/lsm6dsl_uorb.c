/****************************************************************************
 * boards/spike-prime-hub/src/lsm6dsl_uorb.c
 *
 * LSM6DSL IMU uORB driver — combined accel + gyro raw publisher.
 * Single DRDY interrupt on INT1 reads both accel and gyro in one burst,
 * then publishes a single struct sensor_imu on /dev/uorb/sensor_imu0.
 * Physical-unit conversion is left to the consumer; FSR/ODR are
 * runtime-tunable through ioctl while the sensor is inactive.
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

#include <arch/board/board_lsm6dsl.h>

#include "board_usercheck.h"

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

#ifndef CONFIG_LSM6DSL_UORB_BUFSIZE
#  define CONFIG_LSM6DSL_UORB_BUFSIZE           10
#endif

#define WHO_AM_I_VAL    0x6a

/* Registers */

#define DRDY_PULSE_CFG  0x0b   /* DRDY pulsed/latched config */
#define INT1_CTRL       0x0d   /* INT1 pin control */
#define WHO_AM_I        0x0f
#define CTRL1_XL        0x10   /* Accel control */
#define CTRL2_G         0x11   /* Gyro control */
#define CTRL3_C         0x12   /* Control reg 3 (BDU, IF_INC, etc.) */
#define CTRL5_C         0x14   /* Control reg 5 (rounding) */
#define STATUS_REG      0x1e
#define OUT_TEMP_L      0x20   /* Temperature low byte */
#define OUTX_L_G        0x22   /* Gyro output start (12 bytes: G+XL) */

/* Bits */

#define BIT_BDU         (1 << 6)  /* Block Data Update in CTRL3_C */
#define BIT_IF_INC      (1 << 2)  /* Auto-increment in CTRL3_C */
/* CTRL5_C layout (datasheet Table 49):
 *   bits 7-5: ROUNDING[2:0]  (Table 50: 0b011 = gyro+accel rounding)
 *   bit  4:   DEN_LH
 *   bits 3-2: ST_G[1:0]      (gyro self-test; 0b11 is reserved/invalid)
 *   bits 1-0: ST_XL[1:0]
 * Setting (3 << 2) accidentally enables ST_G=0b11, which puts the gyro in a
 * reserved self-test mode and adds a large constant offset to all three axes
 * (~-400 dps observed on hardware).  Use (3 << 5) so ROUNDING[2:0]=0b011.
 */
#define ROUNDING_GY_XL  (3 << 5)  /* Rounding for gyro+accel in CTRL5_C */
#define BIT_DRDY_PULSED (1 << 7)  /* Pulsed DRDY mode */
#define BIT_INT1_DRDY_G (1 << 1)  /* Gyro DRDY on INT1 */
#define BIT_SW_RESET    (1 << 0)  /* Software reset in CTRL3_C */

/* Burst read: 6 bytes gyro (0x22-0x27) + 6 bytes accel (0x28-0x2D) */

#define BURST_DATA_LEN  12

/* Read OUT_TEMP every TEMP_DECIMATE samples; samples in between reuse
 * the cached value.  Keeps the per-sample I2C burst at 12 bytes.
 */

#define TEMP_DECIMATE   16

/* Default startup configuration (matches pybricks). */

#define DEFAULT_ODR     ODR_833HZ
#define DEFAULT_FSR_XL  FSR_XL_8G
#define DEFAULT_FSR_GY  FSR_GY_2000DPS

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
  struct sensor_lowerhalf_s imu_lower;
  FAR struct i2c_master_s *i2c;
  uint8_t addr;
  mutex_t devlock;
  sem_t run;                   /* Polling thread wakeup */
  struct work_s work;          /* HPWORK for interrupt mode */
  enum lsm6dsl_odr_e odr;        /* Live HW ODR (OFF when deactivated) */
  enum lsm6dsl_odr_e cfg_odr;    /* User-configured ODR; persists across
                                  * activate cycles so SET ODR while OFF
                                  * is remembered and applied at the next
                                  * SNIOC_ACTIVATE(true). */
  int fsr_gy;
  int fsr_xl;
  bool active;                 /* Sampling enabled */

  /* ISR -> worker handoff.  ARMv7-M 4-byte aligned word load/store is
   * single-copy atomic, so no critical section is needed for the read
   * in drdy_worker.
   */

  volatile uint32_t ts_irq;

  /* Temperature is decimated to once per TEMP_DECIMATE samples; the
   * cached value is reused for the in-between samples.
   */

  uint32_t sample_index;
  int16_t  temperature_raw;
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
  struct sensor_imu sample;
  int err;
  bool publish = false;

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

  /* Refresh temperature once per TEMP_DECIMATE samples; in between, reuse
   * the cached value.  OUT_TEMP_L / OUT_TEMP_H is two bytes at 0x20.
   */

  if ((dev->sample_index % TEMP_DECIMATE) == 0)
    {
      int16_t temp;
      if (lsm6dsl_read_bytes(dev, OUT_TEMP_L, &temp, sizeof(temp)) >= 0)
        {
          dev->temperature_raw = temp;
        }
    }

  dev->sample_index++;

  /* Rotate chip frame -> SPIKE Prime Hub body frame.  The LSM6DSL on
   * this board is mounted such that body frame is the chip frame
   * rotated 180 deg around X, so Y and Z flip while X is unchanged.
   * Doing the rotation here lets every consumer (uORB clients,
   * btsensor, the PC viewer) treat the published sample as body
   * frame directly instead of carrying a per-board transform.
   *
   * raw[i] == INT16_MIN would overflow on negation, but that value
   * is the LSM6DSL's full-scale saturation sentinel and does not
   * occur from a usable signal -- accept the implementation-defined
   * wrap (-INT16_MIN -> INT16_MIN on this target) rather than
   * complicating the per-sample inner loop with saturation arithmetic.
   */
  sample.timestamp       = dev->ts_irq;
  sample.gx              =  raw[0];
  sample.gy              = -raw[1];
  sample.gz              = -raw[2];
  sample.ax              =  raw[3];
  sample.ay              = -raw[4];
  sample.az              = -raw[5];
  sample.temperature_raw = dev->temperature_raw;
  sample.reserved        = 0;
  publish                = dev->active;

unlock:
  nxmutex_unlock(&dev->devlock);

  /* Issue #96: push_event() reaches into the NuttX uORB sensor framework
   * which takes its own subscriber-list lock.  sensor_close() takes that
   * same lock and then calls activate(false) which wants devlock — so
   * holding devlock across push_event would invert the lock order and
   * deadlock the second daemon stop.  Publish after releasing devlock.
   */

  if (err >= 0 && publish)
    {
      dev->imu_lower.push_event(dev->imu_lower.priv,
                                &sample, sizeof(sample));
    }
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

  /* Capture the timestamp inside the ISR so the worker carries the true
   * sample-ready time, free of HPWORK + I2C burst latency.  ARMv7-M
   * 4-byte aligned STR is single-copy atomic, so the worker's matching
   * LDR can never see a torn value.
   */

  dev->ts_irq = (uint32_t)sensor_get_timestamp();
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
      if (!dev->active)
        {
          nxsem_wait(&dev->run);
          continue;
        }

      dev->ts_irq = (uint32_t)sensor_get_timestamp();
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
  FAR struct lsm6dsl_dev_s *dev =
      container_of(lower, struct lsm6dsl_dev_s, imu_lower);
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  if (enable && !dev->active)
    {
      err = lsm6dsl_set_odr(dev, dev->cfg_odr);
      if (err < 0)
        {
          goto unlock;
        }

      dev->sample_index = 0;
      dev->active       = true;
      nxsem_post(&dev->run);
    }
  else if (!enable && dev->active)
    {
      err = lsm6dsl_set_odr(dev, ODR_OFF);
      dev->active = false;

      /* Drain any HPWORK item that the chip's last DRDY interrupt may
       * have queued; otherwise drdy_worker can fire after sensor_close
       * has freed `user` and use sensor_pollnotify_one on a dangling
       * pointer.  Issue #97 found this manifests as a deterministic
       * crash on the 4th rapid open/close cycle.
       */

      work_cancel(HPWORK, &dev->work);
    }

unlock:
  nxmutex_unlock(&dev->devlock);
  return err;
}

static int lsm6dsl_set_interval(FAR struct sensor_lowerhalf_s *lower,
                                 FAR struct file *filep,
                                 FAR uint32_t *period_us)
{
  FAR struct lsm6dsl_dev_s *dev =
      container_of(lower, struct lsm6dsl_dev_s, imu_lower);
  enum lsm6dsl_odr_e odr;
  int err;

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

  if (dev->active)
    {
      err = -EBUSY;
      goto unlock;
    }

  /* Just remember the new ODR; the chip is in OFF state while !active,
   * and lsm6dsl_activate() will write the configured ODR to the
   * hardware on the next SNIOC_ACTIVATE(true).
   */

  dev->cfg_odr = odr;
  *period_us   = g_odr_interval[odr];
  err = OK;

unlock:
  nxmutex_unlock(&dev->devlock);
  return err;
}

static int set_samplerate_hz(FAR struct lsm6dsl_dev_s *dev, uint32_t hz)
{
  enum lsm6dsl_odr_e odr;

  switch (hz)
    {
    case 13:    odr = ODR_12_5HZ;  break;
    case 26:    odr = ODR_26HZ;    break;
    case 52:    odr = ODR_52HZ;    break;
    case 104:   odr = ODR_104HZ;   break;
    case 208:   odr = ODR_208HZ;   break;
    case 416:   odr = ODR_416HZ;   break;
    case 833:   odr = ODR_833HZ;   break;
    case 1660:  odr = ODR_1660HZ;  break;
    case 3330:  odr = ODR_3330HZ;  break;
    case 6660:  odr = ODR_6660HZ;  break;
    default:    return -EINVAL;
    }

  /* Caller (lsm6dsl_control) already enforces -EBUSY when active. */

  dev->cfg_odr = odr;
  return OK;
}

static int lsm6dsl_get_info(FAR struct sensor_lowerhalf_s *lower,
                             FAR struct file *filep,
                             FAR struct sensor_device_info_s *info)
{
  FAR struct lsm6dsl_dev_s *dev =
      container_of(lower, struct lsm6dsl_dev_s, imu_lower);

  /* nuttx/drivers/sensors/sensor.c forwards arg as a raw user pointer;
   * validate the destination range before any write under BUILD_PROTECTED.
   */

  if (!board_user_out_ok(info, sizeof(*info)))
    {
      return -EFAULT;
    }

  memset(info, 0, sizeof(*info));
  info->power      = 0.55f;
  info->resolution = 1.0f;        /* Raw LSB */
  info->max_range  = (float)INT16_MAX;
  info->min_delay  = (int32_t)g_odr_interval[ODR_6660HZ];
  info->max_delay  = (int32_t)g_odr_interval[ODR_12_5HZ];
  memcpy(info->name, "LSM6DSL", sizeof("LSM6DSL"));
  memcpy(info->vendor, "STMicro", sizeof("STMicro"));
  UNUSED(dev);
  return OK;
}

static int lsm6dsl_control(FAR struct sensor_lowerhalf_s *lower,
                            FAR struct file *filep, int cmd,
                            unsigned long arg)
{
  FAR struct lsm6dsl_dev_s *dev =
      container_of(lower, struct lsm6dsl_dev_s, imu_lower);
  int err;

  err = nxmutex_lock(&dev->devlock);
  if (err < 0)
    {
      return err;
    }

  /* All control commands except WHO_AM_I are reconfiguration; reject them
   * while sampling is active so callers must explicitly stop the sensor
   * first (and are not racing against the publish path).
   */

  if (cmd != SNIOC_WHO_AM_I && dev->active)
    {
      err = -EBUSY;
      goto unlock;
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

        if (!board_user_out_ok(id, sizeof(*id)))
          {
            err = -EFAULT;
            break;
          }

        err = lsm6dsl_read_bytes(dev, WHO_AM_I, id, 1);
      }
      break;

    case SNIOC_SETSAMPLERATE:
      err = set_samplerate_hz(dev, (uint32_t)arg);
      break;

    case LSM6DSL_IOC_SETACCELFSR:
      switch (arg)
        {
        case 2:  err = accel_set_fsr(dev, FSR_XL_2G);  break;
        case 4:  err = accel_set_fsr(dev, FSR_XL_4G);  break;
        case 8:  err = accel_set_fsr(dev, FSR_XL_8G);  break;
        case 16: err = accel_set_fsr(dev, FSR_XL_16G); break;
        default: err = -EINVAL;                         break;
        }
      break;

    case LSM6DSL_IOC_SETGYROFSR:
      switch (arg)
        {
        case 125:  err = gyro_set_fsr(dev, FSR_GY_125DPS);  break;
        case 250:  err = gyro_set_fsr(dev, FSR_GY_250DPS);  break;
        case 500:  err = gyro_set_fsr(dev, FSR_GY_500DPS);  break;
        case 1000: err = gyro_set_fsr(dev, FSR_GY_1000DPS); break;
        case 2000: err = gyro_set_fsr(dev, FSR_GY_2000DPS); break;
        default:   err = -EINVAL;                            break;
        }
      break;

    default:
      err = -EINVAL;
      break;
    }

unlock:
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
  int i;

  /* Pre-flight WHO_AM_I read.  If the chip is wedged on the I2C bus
   * (typically SDA stuck low after a soft-reset interrupted a transaction)
   * fail fast with -ENODEV so the board-level retry can do bus recovery
   * and try again, instead of blocking inside the SW_RESET poll below.
   */

  err = lsm6dsl_read_bytes(dev, WHO_AM_I, &val, 1);
  if (err < 0)
    {
      return err;
    }

  if (val != WHO_AM_I_VAL)
    {
      return -ENODEV;
    }

  /* Software reset */

  err = lsm6dsl_write_byte(dev, CTRL3_C, BIT_SW_RESET);
  if (err < 0)
    {
      return err;
    }

  /* Wait for reset to complete.  Datasheet says SW_RESET typically clears
   * within ~50 us; cap at 50 iterations (~50 ms) so a wedged bus cannot
   * trap us here forever.
   */

  for (i = 0; i < 50; i++)
    {
      nxsig_usleep(1000);
      err = lsm6dsl_read_bytes(dev, CTRL3_C, &val, 1);
      if (err < 0)
        {
          return err;
        }

      if ((val & BIT_SW_RESET) == 0)
        {
          break;
        }
    }

  if (val & BIT_SW_RESET)
    {
      return -ETIMEDOUT;
    }

  /* Defensive settle after SW_RESET before the next register write. */

  nxsig_usleep(10 * 1000);

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

  /* Set startup FSR */

  err = accel_set_fsr(dev, DEFAULT_FSR_XL);
  if (err < 0)
    {
      return err;
    }

  err = gyro_set_fsr(dev, DEFAULT_FSR_GY);
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
  char path[32];
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

  priv->i2c     = i2c;
  priv->addr    = addr;
  priv->odr     = ODR_OFF;
  priv->cfg_odr = DEFAULT_ODR;
  priv->fsr_gy  = DEFAULT_FSR_GY;
  priv->fsr_xl  = DEFAULT_FSR_XL;

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

  /* Register a single combined accel+gyro+temperature topic. */

  priv->imu_lower.type    = SENSOR_TYPE_CUSTOM;
  priv->imu_lower.ops     = &g_sensor_ops;
  priv->imu_lower.nbuffer = CONFIG_LSM6DSL_UORB_BUFSIZE;

  snprintf(path, sizeof(path), "/dev/uorb/sensor_imu%u", devno);

  err = sensor_custom_register(&priv->imu_lower, path,
                               sizeof(struct sensor_imu));
  if (err < 0)
    {
      snerr("ERROR: imu register failed: %d\n", err);
      goto del_sem;
    }

  /* Data acquisition: interrupt or polling */

  if (config->attach != NULL)
    {
      err = config->attach(drdy_int_handler, priv);
      if (err < 0)
        {
          snerr("ERROR: INT1 attach failed: %d\n", err);
          goto unreg_imu;
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
          goto unreg_imu;
        }

      sninfo("LSM6DSL using polling thread.\n");
    }

  sninfo("LSM6DSL driver registered as %s\n", path);
  return OK;

unreg_imu:
  sensor_custom_unregister(&priv->imu_lower, path);
del_sem:
  nxsem_destroy(&priv->run);
del_mutex:
  nxmutex_destroy(&priv->devlock);
free_mem:
  kmm_free(priv);
  return err;
}
