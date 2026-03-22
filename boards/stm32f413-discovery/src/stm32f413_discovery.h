/****************************************************************************
 * boards/stm32f413-discovery/src/stm32f413_discovery.h
 ****************************************************************************/

#ifndef __BOARDS_STM32F413_DISCOVERY_SRC_STM32F413_DISCOVERY_H
#define __BOARDS_STM32F413_DISCOVERY_SRC_STM32F413_DISCOVERY_H

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

/* LEDs
 *   LD1 (green) = PE3
 *   LD2 (red)   = PC5
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTE | GPIO_PIN3)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN5)

/* BUTTONS */

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

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_STM32F413_DISCOVERY_SRC_STM32F413_DISCOVERY_H */
