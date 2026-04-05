/****************************************************************************
 * boards/spike-prime-hub/src/stm32_bringup.c
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

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#include <arch/board/board.h>

#include "stm32.h"
#include "stm32_i2c.h"
#ifdef CONFIG_STM32_IWDG
#  include "stm32_wdg.h"
#endif
#include "spike_prime_hub.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

#ifdef CONFIG_STM32_IWDG
  stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif

#ifdef CONFIG_SCHED_CPULOAD_EXTCLK
  stm32_cpuload_initialize();
#endif

#ifdef CONFIG_SYSLOG_REGISTER
  panic_syslog_initialize();
#endif

#ifdef CONFIG_FS_PROCFS
  ret = nx_mount(NULL, STM32_PROCFS_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to mount procfs at %s: %d\n",
             STM32_PROCFS_MOUNTPOINT, ret);
    }
#endif

#ifdef CONFIG_INPUT_BUTTONS
  ret = btn_lower_initialize("/dev/buttons");
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: btn_lower_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_I2C_DRIVER
  FAR struct i2c_master_s *i2c;

  i2c = stm32_i2cbus_initialize(2);
  if (i2c != NULL)
    {
      ret = i2c_register(i2c, 2);
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: i2c_register(2) failed: %d\n", ret);
        }
    }
#endif

#ifdef CONFIG_STM32_I2C2
  ret = stm32_lsm6dsl_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to initialize LSM6DSL: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_SPI1
  ret = tlc5955_initialize();
  if (ret == OK)
    {
      /* NSH ready: light center button LED green */

      tlc5955_set_duty(TLC5955_CH_STATUS_TOP_G, 0xffff);
      tlc5955_set_duty(TLC5955_CH_STATUS_BTM_G, 0xffff);
      tlc5955_update_sync();
    }
#endif

#ifdef CONFIG_STM32_ADC1
  stm32_adc_dma_initialize();
  ret = stm32_power_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_power_initialize() failed: %d\n", ret);
    }
#endif

  return ret;
}
