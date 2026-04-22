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
#include <arch/board/board_rgbled.h>

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

/* TLC5955 channel IDs (TLC5955_NUM_CHANNELS, TLC5955_CH_*) are defined in
 * <arch/board/board_rgbled.h>, included above so both kernel-space and
 * user-space code share the same layout.
 */

/* USB OTG FS
 *   PA9  = OTG_FS_VBUS
 */

#undef  GPIO_OTGFS_VBUS
#define GPIO_OTGFS_VBUS (GPIO_INPUT | GPIO_FLOAT | GPIO_SPEED_100MHz | \
                         GPIO_OPENDRAIN | GPIO_PORTA | GPIO_PIN9)

/* W25Q256 SPI NOR Flash chip select (active low, idle HIGH)
 *   PB12 = /CS (GPIO software NSS for SPI2)
 */

#define GPIO_W25_CS       (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_50MHz | \
                           GPIO_OUTPUT_SET | GPIO_PORTB | GPIO_PIN12)

/* CC2564C Bluetooth nSHUTD (chip enable, active HIGH)
 *   PA2 = BT_nSHUTD  (GPIO output, drive LOW at boot to keep chip in reset
 *                     until stm32_bluetooth_initialize() brings it up)
 *
 * Note: PA2 is also defined as GPIO_USART2_TX_1 in the pinmap, but this
 * board uses USART2_TX_2 (PD5) for the HCI UART to the BT controller, so
 * PA2 is free for nSHUTD control.
 */

#define GPIO_BT_NSHUTD    (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                           GPIO_OUTPUT_CLEAR | GPIO_PORTA | GPIO_PIN2)

/* procfs */

#ifdef CONFIG_FS_PROCFS
#  ifdef CONFIG_NSH_PROC_MOUNTPOINT
#    define STM32_PROCFS_MOUNTPOINT CONFIG_NSH_PROC_MOUNTPOINT
#  else
#    define STM32_PROCFS_MOUNTPOINT "/proc"
#  endif
#endif

/* Sound (DAC1 CH1 / amp enable)
 *   PA4  = DAC1 OUT1 (analog, upstream GPIO_DAC1_OUT1_0 from pinmap)
 *   PC10 = Amplifier enable (push-pull, active high, CLEAR at boot)
 */

#define GPIO_AMP_EN         (GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHz | \
                             GPIO_OUTPUT_CLEAR | GPIO_PORTC | GPIO_PIN10)

/* Resistor ladder decoder
 *
 * Two resistor ladders encode digital signals as analog voltages:
 *   DEV_0 (PC4, ADC rank 4): CH1 = center button, CH2 = /CHG (MP2639A)
 *   DEV_1 (PA1, ADC rank 5): CH0 = left, CH1 = right, CH2 = BT button
 */

#define RLAD_CH0  0x01
#define RLAD_CH1  0x02
#define RLAD_CH2  0x04

/* Center button (ADC resistor ladder)
 *   PC4 = ADC1_CH14 (not CH4! PC4 maps to CH14 on STM32F413)
 *   Unpressed: ADC ~3645, Pressed: ADC ~2872
 *   Threshold: below 3200 = pressed
 */

#define GPIO_ADC_CENTER_BTN       (GPIO_ANALOG | GPIO_PORTC | GPIO_PIN4)
#define CENTER_BTN_ADC_CH         14
#define CENTER_BTN_PRESS_THRESHOLD 3200

/* Battery charger ISET PWM (TIM5 CH1, PA0, AF2)
 *
 * Note: PA0 is also defined as GPIO_BTN_USER for the Bluetooth button,
 * but the physical BT button is on the resistor ladder (PA1).  When the
 * battery charger is enabled, PA0 is used for ISET PWM.
 */

#define GPIO_ISET_PWM  (GPIO_ALT | GPIO_AF2 | GPIO_SPEED_50MHz | \
                        GPIO_PUSHPULL | GPIO_PORTA | GPIO_PIN0)

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

int stm32_bringup(void);

#ifdef CONFIG_STM32_I2C2
int stm32_lsm6dsl_initialize(void);
#endif

void weak_function stm32_usbinitialize(void);

#ifdef CONFIG_STM32_SPI1
int tlc5955_initialize(void);
void tlc5955_set_duty(uint8_t ch, uint16_t value); /* Auto-schedules update */
int tlc5955_update_sync(void);  /* Immediate update (init/shutdown) */
int stm32_rgbled_register(void);  /* /dev/rgbled0 char device */
#endif

/* Resistor ladder decoder and threshold tables */

uint8_t resistor_ladder_decode(uint16_t adc_value, const uint16_t levels[8]);

extern const uint16_t g_ladder_dev0_levels[8];
extern const uint16_t g_ladder_dev1_levels[8];

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

#ifdef CONFIG_BATTERY_GAUGE
int stm32_battery_gauge_initialize(void);
#endif

#ifdef CONFIG_BATTERY_CHARGER
int stm32_battery_charger_initialize(void);
#endif

#ifdef CONFIG_SCHED_CPULOAD_EXTCLK
void stm32_cpuload_initialize(void);
#endif

int stm32_sound_initialize(void);
int stm32_tone_register(void);
int stm32_pcm_register(void);

#ifdef CONFIG_STM32_SPI2
int stm32_w25q256_initialize(void);
#endif

#ifdef CONFIG_STM32_TIM8
/* Start the 32.768 kHz TIM8 CH4 PWM on PC9 for the CC2564C SLOWCLK.  Must
 * be called before driving GPIO_BT_NSHUTD HIGH.
 */

int stm32_bt_slowclk_initialize(void);
#endif

#ifdef CONFIG_STM32_USART2
/* Initialise USART2 + DMA for the CC2564C HCI link and return a board-local
 * btuart_lowerhalf_s.  Call from the Bluetooth bring-up code after the slow
 * clock is stable but before nSHUTD HIGH so the UART is already accepting
 * traffic when the chip finishes its ROM boot.
 */

struct btuart_lowerhalf_s;
FAR struct btuart_lowerhalf_s *stm32_btuart_instantiate(void);

/* Return the number of bytes currently sitting in the RX ring.  Non-
 * destructive; safe to call from any context including poll() setup.
 */

size_t stm32_btuart_rx_available(FAR const struct btuart_lowerhalf_s *lower);

/* Power on the CC2564C (slow clock + nSHUTD toggle) and leave the USART2
 * lower-half instantiated.  HCI bring-up (reset / init script / baud
 * negotiation) is delegated to the higher-level host stack — see the
 * btstack port under apps/btsensor/ (Issue #52).
 */

int stm32_bluetooth_initialize(void);

/* Return the USART2 lower-half produced by stm32_bluetooth_initialize().
 * NULL until the bring-up has run successfully.
 */

FAR struct btuart_lowerhalf_s *stm32_btuart_lower(void);

/* Register the /dev/ttyBT character device that wraps the btuart lower-half
 * so user-mode apps can drive HCI over POSIX read/write/poll/ioctl.  See
 * boards/spike-prime-hub/include/board_btuart.h for the ABI.
 */

int stm32_btuart_chardev_register(FAR struct btuart_lowerhalf_s *lower);
#endif

#endif /* __ASSEMBLY__ */
#endif /* __BOARDS_SPIKE_PRIME_HUB_SRC_SPIKE_PRIME_HUB_H */
