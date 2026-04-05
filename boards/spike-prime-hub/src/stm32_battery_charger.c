/****************************************************************************
 * boards/spike-prime-hub/src/stm32_battery_charger.c
 *
 * MP2639A battery charger (lower-half) driver for SPIKE Prime Hub.
 * Ported from pybricks charger_mp2639a.c (MIT license).
 *
 * Registers as /dev/charge0 via NuttX battery charger framework.
 *
 * Hardware connections:
 *   MODE pin:  TLC5955 channel 14 (duty=0 enables, duty=0xFFFF disables)
 *   ISET pin:  TIM5 CH1, PA0 (96kHz PWM controls charge current limit)
 *   CHG pin:   Resistor ladder DEV_0 CH2 (shared with center button ADC)
 *   IB pin:    ADC rank 3 (CH3, PA3) — USB charger current measurement
 *   VBUS:      PA9 (GPIO read for USB presence detection)
 *
 * Charge current limits (ISET PWM duty):
 *   0%   = disabled
 *   2%   = 100mA  (USB standard min)
 *   15%  = 500mA  (USB standard max)
 *   100% = 1.5A   (dedicated charger)
 *
 * Battery LED indication (TLC5955 ch0-2: B/G/R):
 *   DISCHARGING:       OFF
 *   CHARGING (not full): Red solid
 *   CHARGING (full):   Green solid
 *   COMPLETE:          Green blink (2.75s on / 0.25s off)
 *   FAULT:             Yellow blink (0.5s on / 0.5s off)
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/wqueue.h>
#include <nuttx/power/battery_charger.h>
#include <nuttx/power/battery_ioctl.h>

#include "stm32.h"
#include "spike_prime_hub.h"

#ifdef CONFIG_BATTERY_CHARGER

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Polling interval: 250ms = 4Hz (matches pybricks) */

#define CHG_POLL_INTERVAL_MS      250

/* CHG signal sampling */

#define CHG_NUM_SAMPLES           7

/* Charge timeout: 60 min charge → 30 sec pause → restart */

#define CHARGE_TIMEOUT_TICKS      (60 * 60 * 1000 / CHG_POLL_INTERVAL_MS)
#define CHARGE_PAUSE_TICKS        (30 * 1000 / CHG_POLL_INTERVAL_MS)

/* ISET PWM duty cycle values (0-999, where 999 = 100%) */

#define ISET_DUTY_OFF             0
#define ISET_DUTY_100MA           20    /* 2% of 1000 */
#define ISET_DUTY_500MA           150   /* 15% of 1000 */
#define ISET_DUTY_1500MA          999   /* ~100% */

/* USB charger current scaling (from pybricks, empirically determined) */

#define IBUSBCH_SCALE_NUM         35116
#define IBUSBCH_SCALE_SHIFT       16
#define IBUSBCH_OFFSET            123

/* Battery full voltage threshold (mV) — 4.095V per cell × 2 */

#define BAT_FULL_MV               8190

/* Voltage-based average: 127/128 EMA coefficient */

#define AVG_SHIFT                 7     /* 2^7 = 128 */

/* TIM5 registers (APB1, 96 MHz after prescaler) */

#define TIM5_BASE                 0x40000c00
#define TIM_CR1_OFFSET            0x00
#define TIM_EGR_OFFSET            0x14
#define TIM_CCMR1_OFFSET          0x18
#define TIM_CCER_OFFSET           0x20
#define TIM_PSC_OFFSET            0x28
#define TIM_ARR_OFFSET            0x2c
#define TIM_CCR1_OFFSET           0x34

#define TIM_CR1_CEN               (1 << 0)
#define TIM_EGR_UG                (1 << 0)
#define TIM_CCMR1_OC1M_PWM1      (6 << 4)
#define TIM_CCMR1_OC1PE           (1 << 3)
#define TIM_CCER_CC1E             (1 << 0)

/* TIM5 period: 96 MHz / 1000 = 96 kHz PWM */

#define TIM5_ARR                  999

/* LED blink patterns (in poll ticks of 250ms) */

#define LED_COMPLETE_ON_TICKS     11    /* 2.75s */
#define LED_COMPLETE_PERIOD       12    /* 3.0s total */
#define LED_FAULT_ON_TICKS        2     /* 0.5s */
#define LED_FAULT_PERIOD          4     /* 1.0s total */

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spike_charger_s
{
  struct battery_charger_dev_s dev;
  struct work_s poll_work;
  bool chg_samples[CHG_NUM_SAMPLES];
  uint8_t chg_index;
  int charger_status;
  bool mode_enabled;
  uint32_t charge_count;
  uint32_t pause_count;
  bool charge_paused;
  uint8_t led_blink_count;
  int32_t avg_voltage;          /* EMA of battery voltage in mV */
  bool avg_initialized;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct spike_charger_s g_charger;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: iset_pwm_initialize
 *
 * Description:
 *   Initialize TIM5 CH1 on PA0 for ISET PWM output at 96 kHz.
 ****************************************************************************/

static void iset_pwm_initialize(void)
{
  /* Enable TIM5 clock (APB1) */

  modifyreg32(STM32_RCC_APB1ENR, 0, RCC_APB1ENR_TIM5EN);

  /* Configure PA0 as TIM5_CH1 (AF2) */

  stm32_configgpio(GPIO_ISET_PWM);

  /* TIM5: no prescaler, period = 999 → 96 MHz / 1000 = 96 kHz */

  putreg32(0, TIM5_BASE + TIM_PSC_OFFSET);
  putreg32(TIM5_ARR, TIM5_BASE + TIM_ARR_OFFSET);

  /* CH1: PWM mode 1, preload enable */

  putreg32(TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC1PE,
           TIM5_BASE + TIM_CCMR1_OFFSET);

  /* Enable CH1 output */

  putreg32(TIM_CCER_CC1E, TIM5_BASE + TIM_CCER_OFFSET);

  /* Initial duty = 0 (off) */

  putreg32(0, TIM5_BASE + TIM_CCR1_OFFSET);

  /* Force update, then enable counter */

  putreg32(TIM_EGR_UG, TIM5_BASE + TIM_EGR_OFFSET);
  putreg32(TIM_CR1_CEN, TIM5_BASE + TIM_CR1_OFFSET);
}

/****************************************************************************
 * Name: iset_pwm_set_duty
 *
 * Description:
 *   Set the ISET PWM duty cycle (0-999).
 ****************************************************************************/

static void iset_pwm_set_duty(uint32_t duty)
{
  if (duty > TIM5_ARR)
    {
      duty = TIM5_ARR;
    }

  putreg32(duty, TIM5_BASE + TIM_CCR1_OFFSET);
}

/****************************************************************************
 * Name: charger_set_mode
 *
 * Description:
 *   Enable or disable charging via the MODE pin (TLC5955 channel 14).
 *   MODE pin is active low: duty=0 enables charging.
 ****************************************************************************/

static void charger_set_mode(FAR struct spike_charger_s *priv, bool enable)
{
  tlc5955_set_duty(TLC5955_CH_CHARGER_MODE, enable ? 0 : 0xffff);
  priv->mode_enabled = enable;
}

/****************************************************************************
 * Name: vbus_present
 *
 * Description:
 *   Check if USB VBUS is present via PA9 GPIO.
 ****************************************************************************/

static bool vbus_present(void)
{
  return stm32_gpioread(GPIO_OTGFS_VBUS);
}

/****************************************************************************
 * Name: read_chg
 *
 * Description:
 *   Read the CHG signal from the resistor ladder.
 *   CHG is on DEV_0 CH2. When /CHG pin is low (charging), CH2 flag is set.
 ****************************************************************************/

static bool read_chg(void)
{
  uint16_t val = stm32_adc_read(ADC_RANK_BTN_CENTER);
  uint8_t flags = resistor_ladder_decode(val, g_ladder_dev0_levels);
  return (flags & RLAD_CH2) != 0;
}

/****************************************************************************
 * Name: charger_enable_if_usb
 *
 * Description:
 *   Enable charging if USB is connected, otherwise disable.
 *   Uses VBUS GPIO (PA9) detection.  Default limit: 500mA.
 *
 *   Note: USB BCD detection is not implemented.  Full BCD requires
 *   manipulating the OTG FS GCCFG register which conflicts with the
 *   NuttX USB driver.  The 500mA default is safe because the MP2639A
 *   hardware auto-limits current if VBUS voltage drops.
 ****************************************************************************/

static void charger_enable_if_usb(FAR struct spike_charger_s *priv)
{
  if (vbus_present())
    {
      iset_pwm_set_duty(ISET_DUTY_500MA);
      charger_set_mode(priv, true);
    }
  else
    {
      iset_pwm_set_duty(ISET_DUTY_OFF);
      charger_set_mode(priv, false);
    }
}

/****************************************************************************
 * Name: update_average_voltage
 *
 * Description:
 *   Update the exponential moving average of battery voltage.
 *   Uses 127/128 coefficient matching pybricks.
 ****************************************************************************/

static void update_average_voltage(FAR struct spike_charger_s *priv)
{
  uint16_t raw_v = stm32_adc_read(ADC_RANK_VBAT);
  uint16_t raw_i = stm32_adc_read(ADC_RANK_IBAT);
  int current_ma = (int)raw_i * 7300 / 4096;
  int voltage_mv = (int)raw_v * 9900 / 4096 + current_ma * 3 / 16;

  if (!priv->avg_initialized)
    {
      priv->avg_voltage = voltage_mv;
      priv->avg_initialized = true;
    }
  else
    {
      priv->avg_voltage = (priv->avg_voltage * 127 + voltage_mv) / 128;
    }
}

/****************************************************************************
 * Name: update_battery_led
 *
 * Description:
 *   Update the battery indicator LED based on charger status.
 ****************************************************************************/

static void update_battery_led(FAR struct spike_charger_s *priv)
{
  uint16_t r = 0;
  uint16_t g = 0;

  switch (priv->charger_status)
    {
      case BATTERY_CHARGING:
        if (priv->avg_voltage >= BAT_FULL_MV)
          {
            /* Full — green solid */

            g = 0xffff;
          }
        else
          {
            /* Charging — red solid */

            r = 0xffff;
          }
        break;

      case BATTERY_FULL:
        /* Complete — green blink */

        priv->led_blink_count = (priv->led_blink_count + 1)
                                % LED_COMPLETE_PERIOD;
        g = priv->led_blink_count < LED_COMPLETE_ON_TICKS ? 0xffff : 0;
        break;

      case BATTERY_FAULT:
        /* Fault — yellow blink */

        priv->led_blink_count = (priv->led_blink_count + 1)
                                % LED_FAULT_PERIOD;
        if (priv->led_blink_count < LED_FAULT_ON_TICKS)
          {
            r = 0xffff;
            g = 0xffff;
          }
        break;

      default:
        /* Discharging — LED off */

        break;
    }

  tlc5955_set_duty(TLC5955_CH_BATTERY_R, r);
  tlc5955_set_duty(TLC5955_CH_BATTERY_G, g);
  tlc5955_set_duty(TLC5955_CH_BATTERY_B, 0);
}

/****************************************************************************
 * Name: charger_poll_work
 *
 * Description:
 *   4Hz work queue handler for charger status monitoring.
 *   Ported from pybricks charger_mp2639a PROCESS_THREAD.
 ****************************************************************************/

static void charger_poll_work(FAR void *arg)
{
  FAR struct spike_charger_s *priv = (FAR struct spike_charger_s *)arg;

  /* Handle charge pause phase */

  if (priv->charge_paused)
    {
      if (++priv->pause_count >= CHARGE_PAUSE_TICKS)
        {
          priv->charge_paused = false;
          priv->charge_count = 0;
          priv->pause_count = 0;
        }
      goto reschedule;
    }


  /* Enable/disable charger based on USB presence */

  charger_enable_if_usb(priv);

  /* Sample CHG signal */

  priv->chg_samples[priv->chg_index] = read_chg();

  /* Update average voltage */

  update_average_voltage(priv);

  /* Determine charger status */

  if (priv->mode_enabled)
    {
      priv->charge_count++;

      /* Count transitions in the circular buffer to detect fault
       * (CHG blinking at ~1Hz).
       */

      int transitions = priv->chg_samples[0] !=
                        priv->chg_samples[CHG_NUM_SAMPLES - 1];
      int i;
      for (i = 1; i < CHG_NUM_SAMPLES; i++)
        {
          transitions += priv->chg_samples[i] != priv->chg_samples[i - 1];
        }

      if (transitions > 2)
        {
          /* CHG blinking → fault */

          priv->charger_status = BATTERY_FAULT;
        }
      else if (priv->chg_samples[priv->chg_index])
        {
          /* CHG signal on → charging */

          priv->charger_status = BATTERY_CHARGING;
        }
      else
        {
          /* CHG signal off.  After initial settling, this means complete. */

          priv->charger_status = priv->charge_count > 2
                               ? BATTERY_FULL
                               : BATTERY_DISCHARGING;
        }
    }
  else
    {
      priv->charger_status = BATTERY_DISCHARGING;
      priv->charge_count = 0;
    }

  /* Advance circular buffer index */

  if (++priv->chg_index >= CHG_NUM_SAMPLES)
    {
      priv->chg_index = 0;
    }

  /* Charge timeout: pause charging after 60 min */

  if (priv->charge_count >= CHARGE_TIMEOUT_TICKS)
    {
      priv->charger_status = BATTERY_DISCHARGING;
      charger_set_mode(priv, false);
      iset_pwm_set_duty(ISET_DUTY_OFF);
      priv->charge_paused = true;
      priv->pause_count = 0;
      priv->charge_count = 0;
    }

  /* Update battery indicator LED */

  update_battery_led(priv);

reschedule:
  work_queue(HPWORK, &priv->poll_work, charger_poll_work, priv,
             MSEC2TICK(CHG_POLL_INTERVAL_MS));
}

/****************************************************************************
 * Charger operations (NuttX lower-half interface)
 ****************************************************************************/

static int spike_charger_state(FAR struct battery_charger_dev_s *dev,
                               FAR int *status)
{
  FAR struct spike_charger_s *priv = (FAR struct spike_charger_s *)dev;
  *status = priv->charger_status;
  return OK;
}

static int spike_charger_health(FAR struct battery_charger_dev_s *dev,
                                FAR int *health)
{
  FAR struct spike_charger_s *priv = (FAR struct spike_charger_s *)dev;

  if (priv->charger_status == BATTERY_FAULT)
    {
      *health = BATTERY_HEALTH_UNSPEC_FAIL;
    }
  else
    {
      *health = BATTERY_HEALTH_GOOD;
    }

  return OK;
}

static int spike_charger_online(FAR struct battery_charger_dev_s *dev,
                                FAR bool *status)
{
  *status = vbus_present();
  return OK;
}

static int spike_charger_voltage(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  /* MP2639A has fixed charge voltage — not settable */

  return OK;
}

static int spike_charger_current(FAR struct battery_charger_dev_s *dev,
                                 int value)
{
  /* Set ISET PWM duty based on requested current (mA) */

  uint32_t duty;

  if (value <= 0)
    {
      duty = ISET_DUTY_OFF;
    }
  else if (value <= 100)
    {
      duty = ISET_DUTY_100MA;
    }
  else if (value <= 500)
    {
      duty = ISET_DUTY_500MA;
    }
  else
    {
      duty = ISET_DUTY_1500MA;
    }

  iset_pwm_set_duty(duty);
  return OK;
}

static int spike_charger_input_current(FAR struct battery_charger_dev_s *dev,
                                       int value)
{
  return -ENOSYS;
}

static int spike_charger_operate(FAR struct battery_charger_dev_s *dev,
                                 uintptr_t param)
{
  return -ENOSYS;
}

static int spike_charger_chipid(FAR struct battery_charger_dev_s *dev,
                                FAR unsigned int *value)
{
  *value = 0x2639;  /* MP2639A */
  return OK;
}

static int spike_charger_get_voltage(FAR struct battery_charger_dev_s *dev,
                                     FAR int *value)
{
  return -ENOSYS;
}

static int spike_charger_voltage_info(FAR struct battery_charger_dev_s *dev,
                                      FAR int *value)
{
  *value = 8400;  /* 2S Li-ion target charge voltage in mV */
  return OK;
}

static int spike_charger_get_protocol(FAR struct battery_charger_dev_s *dev,
                                      FAR int *value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_charger_operations_s g_spike_charger_ops =
{
  .state         = spike_charger_state,
  .health        = spike_charger_health,
  .online        = spike_charger_online,
  .voltage       = spike_charger_voltage,
  .current       = spike_charger_current,
  .input_current = spike_charger_input_current,
  .operate       = spike_charger_operate,
  .chipid        = spike_charger_chipid,
  .get_voltage   = spike_charger_get_voltage,
  .voltage_info  = spike_charger_voltage_info,
  .get_protocol  = spike_charger_get_protocol,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_battery_charger_initialize
 *
 * Description:
 *   Initialize the MP2639A battery charger driver and register /dev/charge0.
 *   Must be called after stm32_adc_dma_initialize() and
 *   tlc5955_initialize().
 ****************************************************************************/

int stm32_battery_charger_initialize(void)
{
  FAR struct spike_charger_s *priv = &g_charger;
  int ret;

  memset(priv, 0, sizeof(*priv));
  priv->dev.ops = &g_spike_charger_ops;
  priv->charger_status = BATTERY_DISCHARGING;

  /* Initialize ISET PWM (TIM5 CH1) */

  iset_pwm_initialize();

  /* Start with charging disabled */

  charger_set_mode(priv, false);
  iset_pwm_set_duty(ISET_DUTY_OFF);


  /* Register charger device */

  ret = battery_charger_register("/dev/charge0", &priv->dev);
  if (ret < 0)
    {
      return ret;
    }

  /* Start 4Hz polling */

  ret = work_queue(HPWORK, &priv->poll_work, charger_poll_work, priv,
                   MSEC2TICK(CHG_POLL_INTERVAL_MS));
  if (ret < 0)
    {
      return ret;
    }

  syslog(LOG_INFO, "CHG: MP2639A charger registered at /dev/charge0\n");
  return OK;
}

#endif /* CONFIG_BATTERY_CHARGER */
