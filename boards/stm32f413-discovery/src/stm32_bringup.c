/****************************************************************************
 * boards/stm32f413-discovery/src/stm32_bringup.c
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

#ifdef CONFIG_INPUT_BUTTONS
#  include <nuttx/input/buttons.h>
#endif

#ifdef CONFIG_SENSORS_LSM9DS0
#  include <nuttx/sensors/lsm9ds0.h>
#endif

#include "stm32.h"
#include "stm32f413_discovery.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32_bringup(void)
{
  int ret = OK;

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

#ifdef CONFIG_SENSORS_LSM9DS0
  ret = stm32_lsm9ds0_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_lsm9ds0_initialize() failed: %d\n",
             ret);
    }
#endif

  return ret;
}
