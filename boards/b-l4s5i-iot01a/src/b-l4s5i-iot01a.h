/****************************************************************************
 * boards/b-l4s5i-iot01a/src/b-l4s5i-iot01a.h
 ****************************************************************************/

#ifndef __BOARDS_B_L4S5I_IOT01A_SRC_B_L4S5I_IOT01A_H
#define __BOARDS_B_L4S5I_IOT01A_SRC_B_L4S5I_IOT01A_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>

#include <arch/stm32l4/chip.h>

#include "stm32l4_gpio.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* LEDs
 *   LD1 (green) = PA5
 *   LD2 (red)   = PB14
 */

#define GPIO_LED1       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN5)
#define GPIO_LED2       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                         GPIO_OUTPUT_CLEAR | GPIO_PORTB | GPIO_PIN14)

/* LSM6DSL IMU on I2C2
 *   INT1 = PD11 (EXTI11)
 */

#define GPIO_LSM6DSL_INT1 (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                           GPIO_PORTD | GPIO_PIN11)

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

int stm32l4_bringup(void);

#ifdef CONFIG_SENSORS_LSM6DSL_UORB
int stm32l4_lsm6dsl_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_B_L4S5I_IOT01A_SRC_B_L4S5I_IOT01A_H */
