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

#include <nuttx/arch.h>
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

#ifdef CONFIG_ARCH_IRQPRIO
  /* Issue #36 epsilon plan + Issue #50 refinement: board-wide NVIC
   * priority assignment.
   *
   * Every peripheral IRQ is compressed into the 0x80-0xE0 band so that
   * it stays at or below NuttX BASEPRI (0x80) and can therefore safely
   * call NuttX APIs (nxsem_post, etc.) from its ISR. 0x80 is reserved
   * for the OS tick (TIM9) so no peripheral shares a level with the
   * scheduler. Slot 0x90 stays reserved for Issue #43 (LUMP UART):
   *
   *   0x80  TIM9 tickless tick          (OS, kept at default)
   *   0x90  LUMP UART                   (future reservation, Issue #43)
   *   0xA0  Bluetooth UART              (USART2 + DMA1 S6/S7)
   *   0xB0  IMU I2C2 EV/ER + EXTI4
   *   0xC0  Sound DAC DMA1_S5
   *   0xD0  W25Q256 DMA1_S3/S4
   *         W25Q256 SPI2 IRQ
   *         USB OTG FS
   *         USB VBUS EXTI9_5            (Issue #49)
   *   0xE0  ADC DMA2_S0
   *         TLC5955 SPI1 + DMA2_S2/S3
   *         BUTTON_USER EXTI0
   *   0xF0  PendSV, SysTick
   *
   * See docs/{ja,en}/hardware/dma-irq.md for the full rationale.
   */

  /* step 1: system handlers */
  up_prioritize_irq(STM32_IRQ_PENDSV,  NVIC_SYSH_PRIORITY_MIN);
  up_prioritize_irq(STM32_IRQ_SYSTICK, NVIC_SYSH_PRIORITY_MIN);

  /* step 2: TLC5955 SPI1 + DMA (0xE0) */
  up_prioritize_irq(STM32_IRQ_SPI1,
                    NVIC_SYSH_PRIORITY_DEFAULT + 6 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_DMA2S2,
                    NVIC_SYSH_PRIORITY_DEFAULT + 6 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_DMA2S3,
                    NVIC_SYSH_PRIORITY_DEFAULT + 6 * NVIC_SYSH_PRIORITY_STEP);

  /* step 3: ADC DMA (0xE0) */
  up_prioritize_irq(STM32_IRQ_DMA2S0,
                    NVIC_SYSH_PRIORITY_DEFAULT + 6 * NVIC_SYSH_PRIORITY_STEP);

  /* step 4: USB OTG FS (0xD0) */
  up_prioritize_irq(STM32_IRQ_OTGFS,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);

  /* step 5: Sound DAC DMA (DMA1 Stream5, 0xC0) */
  up_prioritize_irq(STM32_IRQ_DMA1S5,
                    NVIC_SYSH_PRIORITY_DEFAULT + 4 * NVIC_SYSH_PRIORITY_STEP);

  /* step 6: IMU LSM6DS3TR-C — I2C2 event/error + EXTI4 (INT1 DRDY, 0xB0) */
  up_prioritize_irq(STM32_IRQ_I2C2EV,
                    NVIC_SYSH_PRIORITY_DEFAULT + 3 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_I2C2ER,
                    NVIC_SYSH_PRIORITY_DEFAULT + 3 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_EXTI4,
                    NVIC_SYSH_PRIORITY_DEFAULT + 3 * NVIC_SYSH_PRIORITY_STEP);

#ifdef CONFIG_STM32_SPI2
  /* step 7: W25Q256 Flash SPI2 + DMA1 Stream3/4 (0xD0, co-resident with
   * USB OTG FS).  DMA and SPI2 completion are kept at the same NVIC
   * level so they cannot mutually preempt: splitting them (DMA=0xD0 /
   * SPI2 IRQ=0xE0) surfaces SPI2 driver races under heavy concurrent
   * load (Sound DMA + IMU I2C + Flash dd) and causes USB CDC detaches.
   * pybricks groups them at the same preempt level (5 / 6) as well.
   */
  up_prioritize_irq(STM32_IRQ_DMA1S3,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_DMA1S4,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_SPI2,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);
#endif

#ifdef CONFIG_STM32_USART2
  /* step 8: CC2564C Bluetooth HCI UART — USART2 + DMA1 S6 (TX) + DMA1 S7
   * (RX) at 0xA0 (Issue #50 reserved slot).  Sits between LUMP (0x90,
   * Issue #43) and IMU (0xB0), matching the pybricks relative ordering
   * so BT RX DMA does not wait behind IMU / Sound / Flash traffic.
   */
  up_prioritize_irq(STM32_IRQ_USART2,
                    NVIC_SYSH_PRIORITY_DEFAULT + 2 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_DMA1S6,
                    NVIC_SYSH_PRIORITY_DEFAULT + 2 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_DMA1S7,
                    NVIC_SYSH_PRIORITY_DEFAULT + 2 * NVIC_SYSH_PRIORITY_STEP);
#endif

  /* step 9: BUTTON_USER (PA0 EXTI0, BT control button) at 0xE0.  Lives
   * with ADC + TLC5955 in the lowest peripheral band — button events are
   * rare and never time-critical, but the slot must stay below BASEPRI
   * so the IRQ handler can call NuttX work-queue APIs (Issue #56).
   */
  up_prioritize_irq(STM32_IRQ_EXTI0,
                    NVIC_SYSH_PRIORITY_DEFAULT + 6 * NVIC_SYSH_PRIORITY_STEP);

  /* step 10: USB VBUS detect (PA9 EXTI9_5) at 0xD0.  Co-resident with USB
   * OTG FS and W25Q256 SPI2/DMA so VBUS edge handling sits in the same
   * preempt level as the USB stack it ultimately drives, matching the
   * pybricks `platform.c:965` ordering (Issue #49).
   */
  up_prioritize_irq(STM32_IRQ_EXTI95,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);
#endif

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

#ifdef CONFIG_STM32_SPI2
  ret = stm32_w25q256_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_w25q256_initialize() failed: %d\n", ret);
      /* non-fatal: bringup continues, /mnt/flash will be absent */
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

  ret = stm32_rgbled_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_rgbled_register() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_ADC1
  stm32_adc_dma_initialize();
#endif

#ifdef CONFIG_BATTERY_GAUGE
  ret = stm32_battery_gauge_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_battery_gauge_initialize() failed: %d\n",
             ret);
    }
#endif

#ifdef CONFIG_BATTERY_CHARGER
  ret = stm32_battery_charger_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: stm32_battery_charger_initialize() failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_STM32_ADC1
  ret = stm32_power_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_power_initialize() failed: %d\n", ret);
    }

  ret = stm32_btbutton_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_btbutton_initialize() failed: %d\n",
             ret);
    }
#endif

  ret = stm32_sound_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_sound_initialize() failed: %d\n", ret);
    }

  ret = stm32_tone_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_tone_register() failed: %d\n", ret);
    }

  ret = stm32_pcm_register();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_pcm_register() failed: %d\n", ret);
    }

#ifdef CONFIG_STM32_USART2
  ret = stm32_bluetooth_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32_bluetooth_initialize() failed: %d\n",
             ret);
    }
#endif

  return ret;
}
