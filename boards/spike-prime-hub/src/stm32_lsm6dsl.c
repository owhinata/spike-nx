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
#include <syslog.h>

#include <nuttx/irq.h>
#include <nuttx/signal.h>
#include <nuttx/i2c/i2c_master.h>
#include "lsm6dsl_uorb.h"

#include "stm32.h"
#include "stm32_gpio.h"
#include "stm32_i2c.h"
#include "spike_prime_hub.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LSM6DSL_I2C_BUS         2
#define LSM6DSL_I2C_ADDR        0x6a
#define LSM6DSL_INIT_RETRIES    3
#define LSM6DSL_RETRY_DELAY_MS  10

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
  int reset_ret;
  int ret = -ENODEV;
  int attempt;

  /* Get I2C2 bus instance */

  i2c = stm32_i2cbus_initialize(LSM6DSL_I2C_BUS);
  if (i2c == NULL)
    {
      snerr("ERROR: Failed to initialize I2C%d\n", LSM6DSL_I2C_BUS);
      return -ENODEV;
    }

  config.attach = lsm6dsl_int1_attach;

  /* Pair I2C bus recovery (SCL clock toggle + START/STOP) with the LSM6DSL
   * register sequence and retry as a unit.  A WDOG/crash soft-reset can
   * leave the chip mid-transaction with SDA stuck low; the bus recovery
   * unwedges it before SW_RESET is attempted.
   */

  for (attempt = 0; attempt < LSM6DSL_INIT_RETRIES; attempt++)
    {
      /* NuttX stm32_i2c_reset() leaves the bus deinitialized with the pins
       * still in GPIO mode if it bails out (SDA never released, or clock
       * stretch never relaxes).  In that case re-bind the bus by uninit /
       * init so the next attempt starts from a clean peripheral state.
       */

      reset_ret = I2C_RESET(i2c);
      if (reset_ret < 0)
        {
          snwarn("I2C_RESET attempt %d failed: %d (re-binding bus)\n",
                 attempt, reset_ret);
          stm32_i2cbus_uninitialize(i2c);
          i2c = stm32_i2cbus_initialize(LSM6DSL_I2C_BUS);
          if (i2c == NULL)
            {
              snerr("ERROR: I2C%d re-bind failed\n", LSM6DSL_I2C_BUS);
              return -ENODEV;
            }
        }

      ret = lsm6dsl_register_uorb(i2c, LSM6DSL_I2C_ADDR, 0, &config);
      if (ret == OK)
        {
          break;
        }

      snwarn("lsm6dsl_register_uorb attempt %d failed: %d\n", attempt, ret);
      nxsig_usleep(LSM6DSL_RETRY_DELAY_MS * 1000);
    }

  if (ret < 0)
    {
      snerr("ERROR: LSM6DSL init gave up after %d attempts: %d\n",
            LSM6DSL_INIT_RETRIES, ret);
      return ret;
    }

  syslog(LOG_INFO,
         "IMU: LSM6DS3TR-C registered on I2C%d addr=0x%02x (attempt %d)\n",
         LSM6DSL_I2C_BUS, LSM6DSL_I2C_ADDR, attempt);

  return OK;
}
