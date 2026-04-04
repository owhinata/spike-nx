/****************************************************************************
 * boards/spike-prime-hub/src/spike_prime_hub.h
 ****************************************************************************/

#ifndef __BOARDS_SPIKE_PRIME_HUB_SRC_SPIKE_PRIME_HUB_H
#define __BOARDS_SPIKE_PRIME_HUB_SRC_SPIKE_PRIME_HUB_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <stdint.h>
#include <arch/stm32/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Power control
 *   PA13 = BAT_PWR_EN  (main battery power, must be HIGH to stay on)
 *   PA14 = PORT_3V3_EN (3.3V to I/O ports)
 */

#define GPIO_BAT_PWR_EN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                         GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN13)
#define GPIO_PORT_3V3_EN (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                          GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN14)

/* Bluetooth button */

#define MIN_IRQBUTTON   BUTTON_USER
#define MAX_IRQBUTTON   BUTTON_USER
#define NUM_IRQBUTTONS  1

#define GPIO_BTN_USER   (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                         GPIO_PORTA | GPIO_PIN0)

/* USB OTG FS
 *   PA9  = OTG_FS_VBUS
 */

#undef  GPIO_OTGFS_VBUS
#define GPIO_OTGFS_VBUS (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                         GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN9)

/* procfs */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

int stm32_bringup(void);

void weak_function stm32_usbinitialize(void);

#ifdef CONFIG_SCHED_CPULOAD_EXTCLK
void stm32_cpuload_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_SPIKE_PRIME_HUB_SRC_SPIKE_PRIME_HUB_H */
