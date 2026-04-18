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
  /* Issue #36 epsilon plan: board-wide NVIC priority assignment.
   *
   * Every peripheral IRQ is compressed into the 0x80-0xF0 band so that
   * it stays at or below NuttX BASEPRI (0x80) and can therefore safely
   * call NuttX APIs (nxsem_post, etc.) from its ISR. The relative
   * ordering mirrors the pybricks design:
   *
   *   0x80  TIM9 tickless tick          (kept at default)
   *   0x90  IMU I2C2 ER/EV + EXTI4
   *   0xA0  Sound DAC DMA1_S5
   *   0xB0  USB OTG FS
   *   0xD0  ADC DMA2_S0
   *   0xD0  TLC5955 SPI1 + DMA2_S2/S3
   *   0xF0  PendSV, SysTick
   *
   * See docs/{ja,en}/hardware/dma-irq.md "Resolution: Issue #36" for
   * the full rationale and the staged rollout (steps 1-6) that verified
   * Issue #36 does not recur.
   */

  /* step 1: system handlers */
  up_prioritize_irq(STM32_IRQ_PENDSV,  NVIC_SYSH_PRIORITY_MIN);
  up_prioritize_irq(STM32_IRQ_SYSTICK, NVIC_SYSH_PRIORITY_MIN);

  /* step 2: TLC5955 SPI1 + DMA */
  up_prioritize_irq(STM32_IRQ_SPI1,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_DMA2S2,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_DMA2S3,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);

  /* step 3: ADC DMA */
  up_prioritize_irq(STM32_IRQ_DMA2S0,
                    NVIC_SYSH_PRIORITY_DEFAULT + 5 * NVIC_SYSH_PRIORITY_STEP);

  /* step 4: USB OTG FS */
  up_prioritize_irq(STM32_IRQ_OTGFS,
                    NVIC_SYSH_PRIORITY_DEFAULT + 3 * NVIC_SYSH_PRIORITY_STEP);

  /* step 5: Sound DAC DMA (DMA1 Stream5) */
  up_prioritize_irq(STM32_IRQ_DMA1S5,
                    NVIC_SYSH_PRIORITY_DEFAULT + 2 * NVIC_SYSH_PRIORITY_STEP);

  /* step 6: IMU LSM6DS3TR-C — I2C2 event/error + EXTI4 (INT1 DRDY) */
  up_prioritize_irq(STM32_IRQ_I2C2EV,
                    NVIC_SYSH_PRIORITY_DEFAULT + 1 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_I2C2ER,
                    NVIC_SYSH_PRIORITY_DEFAULT + 1 * NVIC_SYSH_PRIORITY_STEP);
  up_prioritize_irq(STM32_IRQ_EXTI4,
                    NVIC_SYSH_PRIORITY_DEFAULT + 1 * NVIC_SYSH_PRIORITY_STEP);
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

  return ret;
}
