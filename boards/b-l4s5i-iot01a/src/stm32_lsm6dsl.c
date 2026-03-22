/****************************************************************************
 * boards/b-l4s5i-iot01a/src/stm32_lsm6dsl.c
 *
 * Board-level initialization for LSM6DSL IMU on I2C2.
 *
 * Wiring (B-L4S5I-IOT01A onboard):
 *   PB10 = I2C2_SCL
 *   PB11 = I2C2_SDA
 *   PD11 = LSM6DSL INT1 (EXTI11)
 *
 * LSM6DSL I2C address: 0x6A (SDO/SA0 = GND)
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/sensors/lsm6dsl_uorb.h>

#include "stm32l4.h"
#include "stm32l4_gpio.h"
#include "stm32l4_exti.h"
#include "b-l4s5i-iot01a.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSM6DSL_I2C_BUS     2
#define LSM6DSL_I2C_ADDR    0x6a

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int lsm6dsl_int1_attach(xcpt_t handler, FAR void *arg)
{
  stm32l4_configgpio(GPIO_LSM6DSL_INT1);
  return stm32l4_gpiosetevent(GPIO_LSM6DSL_INT1,
                              true,   /* rising edge */
                              false,  /* no falling edge */
                              false,  /* no event mode */
                              handler, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32l4_lsm6dsl_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  struct lsm6dsl_uorb_config_s config;
  int ret;

  /* Get I2C2 bus instance */

  i2c = stm32l4_i2cbus_initialize(LSM6DSL_I2C_BUS);
  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C%d\n", LSM6DSL_I2C_BUS);
      return -ENODEV;
    }

  /* Configure LSM6DSL with INT1 for accel DRDY, gyro via kthread */

  config.xl_int    = LSM6DSL_INT1;
  config.gy_int    = LSM6DSL_INT2;
  config.xl_attach = lsm6dsl_int1_attach;
  config.gy_attach = NULL;  /* No INT2 GPIO — use kthread polling */

  /* Register LSM6DSL as uORB sensor */

  ret = lsm6dsl_register_uorb(i2c, LSM6DSL_I2C_ADDR, 0, &config);
  if (ret < 0)
    {
      snerr("ERROR: lsm6dsl_register_uorb failed: %d\n", ret);
      return ret;
    }

  sninfo("LSM6DSL registered on I2C%d addr=0x%02x\n",
         LSM6DSL_I2C_BUS, LSM6DSL_I2C_ADDR);

  return OK;
}
