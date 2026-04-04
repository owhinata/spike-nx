/****************************************************************************
 * boards/b-l4s5i-iot01a/src/stm32_mx25r.c
 *
 * Initialize MX25R6435F QSPI flash and mount LittleFS.
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/spi/qspi.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/fs/fs.h>

#include "stm32l4_octospi.h"

#include "b-l4s5i-iot01a.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int stm32l4_mx25r_initialize(void)
{
  struct qspi_dev_s *qspi;
  struct mtd_dev_s *mtd;
  int ret;

  /* Initialize the OCTOSPI bus (port 0) */

  qspi = stm32l4_octospi_initialize(0);
  if (qspi == NULL)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_octospi_initialize failed\n");
      return -EIO;
    }

  /* Initialize the MX25R6435F flash device */

  mtd = mx25rxx_initialize(qspi, true);
  if (mtd == NULL)
    {
      syslog(LOG_ERR, "ERROR: mx25rxx_initialize failed\n");
      return -EIO;
    }

  /* Register as MTD block device */

  ret = register_mtddriver("/dev/mtdblock0", mtd, 0755, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: register_mtddriver failed: %d\n", ret);
      return ret;
    }

#ifdef CONFIG_FS_LITTLEFS
  /* Mount LittleFS; format on first boot */

  ret = nx_mount("/dev/mtdblock0", "/data", "littlefs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_WARNING, "WARNING: LittleFS mount failed: %d, formatting\n",
             ret);
      ret = nx_mount("/dev/mtdblock0", "/data", "littlefs", 0,
                     "forceformat");
      if (ret < 0)
        {
          syslog(LOG_ERR, "ERROR: LittleFS format+mount failed: %d\n", ret);
          return ret;
        }
    }

  syslog(LOG_INFO, "LittleFS mounted at /data\n");
#endif

  return OK;
}
