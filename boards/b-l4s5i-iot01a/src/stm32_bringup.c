/****************************************************************************
 * boards/b-l4s5i-iot01a/src/stm32_bringup.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdio.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/fs/fs.h>
#include <nuttx/i2c/i2c_master.h>

#include "stm32l4.h"
#include "stm32l4_i2c.h"
#include "b-l4s5i-iot01a.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32l4_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_SCHED_CPULOAD_EXTCLK
  stm32_cpuload_initialize();
#endif

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_I2C_DRIVER
  FAR struct i2c_master_s *i2c;

  i2c = stm32l4_i2cbus_initialize(2);
  if (i2c != NULL)
    {
      ret = i2c_register(i2c, 2);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: i2c_register(2) failed: %d\n", ret);
        }
    }
  else
    {
      syslog(LOG_ERR, "ERROR: stm32l4_i2cbus_initialize(2) failed\n");
    }
#endif

#ifdef CONFIG_SENSORS_LSM6DSL_UORB
  ret = stm32l4_lsm6dsl_initialize();
  if (ret < 0)
    {
      snerr("ERROR: Failed to initialize LSM6DSL: %d\n", ret);
    }
#endif

  return ret;
}
