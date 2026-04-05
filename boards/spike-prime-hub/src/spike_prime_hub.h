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

/* IMU (LSM6DS3TR-C)
 *   PB4 = INT1 (gyro DRDY, EXTI4)
 */

#define GPIO_LSM6DSL_INT1 (GPIO_INPUT | GPIO_FLOAT | GPIO_EXTI | \
                           GPIO_PORTB | GPIO_PIN4)

/* TLC5955 LED Driver
 *   SPI1: MOSI=PA7, MISO=PA6, SCK=PA5 (AF5)
 *   LAT:  PA15 (GPIO output, latch data on HIGH->LOW)
 *   GSCLK: TIM12 CH2 = PB15 (9.6 MHz PWM clock for LED driver)
 */

#define GPIO_TLC5955_LAT  (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN15)

/* TLC5955 channel IDs (48 channels total) */

#define TLC5955_NUM_CHANNELS  48

#define TLC5955_CH_BATTERY_B       0
#define TLC5955_CH_BATTERY_G       1
#define TLC5955_CH_BATTERY_R       2
#define TLC5955_CH_STATUS_TOP_B    3
#define TLC5955_CH_STATUS_TOP_G    4
#define TLC5955_CH_STATUS_TOP_R    5
#define TLC5955_CH_STATUS_BTM_B    6
#define TLC5955_CH_STATUS_BTM_G    7
#define TLC5955_CH_STATUS_BTM_R    8
#define TLC5955_CH_BT_B            18
#define TLC5955_CH_BT_G            19
#define TLC5955_CH_BT_R            20

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

/* Center button (ADC resistor ladder)
 *   PC4 = ADC1_CH14 (not CH4! PC4 maps to CH14 on STM32F413)
 *   Unpressed: ADC ~3645, Pressed: ADC ~2872
 *   Threshold: below 3200 = pressed
 */

#define GPIO_ADC_CENTER_BTN       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4)
#define CENTER_BTN_ADC_CH         14
#define CENTER_BTN_PRESS_THRESHOLD 3200

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

int stm32_bringup(void);

#ifdef CONFIG_SENSORS_LSM6DSL_UORB
int stm32_lsm6dsl_initialize(void);
#endif

void weak_function stm32_usbinitialize(void);

#ifdef CONFIG_STM32_SPI1
int tlc5955_initialize(void);
void tlc5955_set_duty(uint8_t ch, uint16_t value); /* Auto-schedules update */
int tlc5955_update_sync(void);  /* Immediate update (init/shutdown) */
#endif

#ifdef CONFIG_STM32_ADC1
/* ADC DMA scan ranks (index into DMA buffer) */

#define ADC_RANK_IBAT         0   /* CH10 PC0: Battery current */
#define ADC_RANK_VBAT         1   /* CH11 PC1: Battery voltage */
#define ADC_RANK_NTC          2   /* CH8  PB0: Battery temperature */
#define ADC_RANK_IBUSBCH      3   /* CH3  PA3: USB charger current */
#define ADC_RANK_BTN_CENTER   4   /* CH14 PC4: Center button */
#define ADC_RANK_BTN_LRB      5   /* CH5  PA1: Left/Right/BT buttons */

int stm32_adc_dma_initialize(void);
uint16_t stm32_adc_read(uint8_t rank);
int stm32_power_initialize(void);
#endif

#ifdef CONFIG_SCHED_CPULOAD_EXTCLK
void stm32_cpuload_initialize(void);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_SPIKE_PRIME_HUB_SRC_SPIKE_PRIME_HUB_H */
