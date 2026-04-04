/****************************************************************************
 * boards/spike-prime-hub/src/stm32_lsm6dsl.c
 *
 * Board-level initialization for LSM6DS3TR-C IMU on I2C2.
 *
 * Wiring (SPIKE Prime Hub):
 *   PB10 = I2C2_SCL (AF4)
 *   PB3  = I2C2_SDA (AF9)
 *   PB4  = LSM6DS3TR-C INT1 (EXTI4)
 *
 * LSM6DS3TR-C I2C address: 0x6A (SDO/SA0 = GND)
 * Register-compatible with LSM6DSL — uses the same NuttX uORB driver.
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

#include "stm32.h"
#include "stm32_gpio.h"
#include "spike_prime_hub.h"

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
  stm32_configgpio(GPIO_LSM6DSL_INT1);
  return stm32_gpiosetevent(GPIO_LSM6DSL_INT1,
                            true,   /* rising edge */
                            false,  /* no falling edge */
                            false,  /* no event mode */
                            handler, arg);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_lsm6dsl_initialize(void)
{
  FAR struct i2c_master_s *i2c;
  struct lsm6dsl_uorb_config_s config;
  int ret;

  /* Get I2C2 bus instance */

  i2c = stm32_i2cbus_initialize(LSM6DSL_I2C_BUS);
  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C%d\n", LSM6DSL_I2C_BUS);
      return -ENODEV;
    }

  /* Configure LSM6DS3TR-C with INT1 for gyro DRDY (burst reads both) */

  config.attach = lsm6dsl_int1_attach;

  /* Register as uORB sensor */

  ret = lsm6dsl_register_uorb(i2c, LSM6DSL_I2C_ADDR, 0, &config);
  if (ret < 0)
    {
      snerr("ERROR: lsm6dsl_register_uorb failed: %d\n", ret);
      return ret;
    }

  sninfo("LSM6DS3TR-C registered on I2C%d addr=0x%02x\n",
         LSM6DSL_I2C_BUS, LSM6DSL_I2C_ADDR);

  return OK;
}
