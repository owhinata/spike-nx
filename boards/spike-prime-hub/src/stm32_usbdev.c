/****************************************************************************
 * boards/spike-prime-hub/src/stm32_usbdev.c
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <stdbool.h>
#include <debug.h>

#include <nuttx/usb/usbdev.h>

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_STM32_OTGFS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32_usbinitialize(void)
{
  stm32_configgpio(GPIO_OTGFS_VBUS);
}

#ifdef CONFIG_USBDEV
void stm32_usbsuspend(struct usbdev_s *dev, bool resume)
{
  uinfo("resume: %d\n", resume);
}
#endif

#endif /* CONFIG_STM32_OTGFS */
