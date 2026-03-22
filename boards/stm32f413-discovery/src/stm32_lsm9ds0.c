/****************************************************************************
 * boards/stm32f413-discovery/src/stm32_lsm9ds0.c
 *
 * Board-level initialization for LSM9DS0 IMU on I2C1.
 *
 * Wiring:
 *   PB6 = I2C1_SCL (Arduino D15)
 *   PB7 = I2C1_SDA (Arduino D14)
 *
 * LSM9DS0 I2C addresses depend on SA0 pin:
 *   SA0=HIGH: G=0x6B, XM=0x1D (default on most breakout boards)
 *   SA0=LOW:  G=0x6A, XM=0x1E
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm9ds0.h>

#include "stm32.h"
#include "stm32f413_discovery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_LSM9DS0_SA0_HIGH
/* Default: SA0=HIGH (most breakout boards pull SA0 high) */

#  define LSM9DS0_I2C_ADDR_G   LSM9DS0_G_ADDR1    /* 0x6B */
#  define LSM9DS0_I2C_ADDR_XM  LSM9DS0_XM_ADDR1   /* 0x1D */
#else
#  define LSM9DS0_I2C_ADDR_G   LSM9DS0_G_ADDR1    /* 0x6B */
#  define LSM9DS0_I2C_ADDR_XM  LSM9DS0_XM_ADDR1   /* 0x1D */
#endif

#ifdef CONFIG_LSM9DS0_SA0_LOW
#  undef  LSM9DS0_I2C_ADDR_G
#  undef  LSM9DS0_I2C_ADDR_XM
#  define LSM9DS0_I2C_ADDR_G   LSM9DS0_G_ADDR0    /* 0x6A */
#  define LSM9DS0_I2C_ADDR_XM  LSM9DS0_XM_ADDR0   /* 0x1E */
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_lsm9ds0_initialize
 *
 * Description:
 *   Initialize and register the LSM9DS0 IMU driver on I2C1.
 *
 ****************************************************************************/

int stm32_lsm9ds0_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  struct lsm9ds0_config_s config;
  int ret;

  /* Get I2C1 bus instance */

  i2c = stm32_i2cbus_initialize(1);
  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C1\n");
      return -ENODEV;
    }

  /* Configure LSM9DS0 */

  config.i2c     = i2c;
  config.addr_g  = LSM9DS0_I2C_ADDR_G;
  config.addr_xm = LSM9DS0_I2C_ADDR_XM;

  /* Register the LSM9DS0 as uORB sensor devices */

  ret = lsm9ds0_register_uorb(0, &config);
  if (ret < 0)
    {
      snerr("ERROR: lsm9ds0_register_uorb failed: %d\n", ret);
      return ret;
    }

  sninfo("LSM9DS0 registered (G=0x%02x, XM=0x%02x)\n",
         LSM9DS0_I2C_ADDR_G, LSM9DS0_I2C_ADDR_XM);

  return OK;
}
