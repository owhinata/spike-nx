/****************************************************************************
 * boards/spike-prime-hub/src/stm32_battery_gauge.c
 *
 * Battery gauge (lower-half) driver for SPIKE Prime Hub.
 * Ported from pybricks battery_adc.c (MIT license).
 *
 * Reads battery voltage, current, and temperature from the ADC DMA buffer.
 * Registers as /dev/bat0 via NuttX battery gauge framework.
 *
 * Hardware:
 *   Battery: 2S Li-ion (nominal 7.2V, full 8.4V, cutoff 6.0V)
 *   Voltage: ADC rank 1 (CH11, PC1) — max 9900mV at 4096 counts
 *   Current: ADC rank 0 (CH10, PC0) — max 7300mA at 4096 counts
 *     0.05 ohm shunt resistor + op amp
 *     Voltage correction: +current * 3/16 mV (0.1875 ohm path resistance)
 *   Temperature: ADC rank 2 (CH8, PB0) — 103AT NTC thermistor
 *     B=3435, R0=10k @ 25°C, voltage divider 2.4k/7.5k
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <stdbool.h>
#include <errno.h>
#include <math.h>
#include <debug.h>

#include <nuttx/clock.h>
#include <nuttx/power/battery_gauge.h>
#include <nuttx/power/battery_ioctl.h>

#include "spike_prime_hub.h"

#ifdef CONFIG_BATTERY_GAUGE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* ADC scaling constants (from pybricks pbdrvconfig.h for prime_hub) */

#define BAT_VOLTAGE_RAW_MAX       4096
#define BAT_VOLTAGE_SCALED_MAX    9900   /* mV at full-scale ADC */
#define BAT_CURRENT_RAW_MAX       4096
#define BAT_CURRENT_SCALED_MAX    7300   /* mA at full-scale ADC */
#define BAT_CURRENT_CORRECTION    3      /* 3/16 ohm path resistance */

/* Boot suppression: report 7000mV if real voltage is lower during first 1s */

#define BAT_BOOT_SUPPRESS_MS      1000
#define BAT_BOOT_SUPPRESS_MV      7000

/* Li-ion 2S voltage-to-SoC breakpoints (mV → %) */

#define BAT_FULL_MV               8400
#define BAT_CUTOFF_MV             6000

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct spike_gauge_s
{
  struct battery_gauge_dev_s dev;
  uint32_t boot_ticks;
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int spike_gauge_get_raw_current(void)
{
  uint16_t raw = stm32_adc_read(ADC_RANK_IBAT);
  return (int)raw * BAT_CURRENT_SCALED_MAX / BAT_CURRENT_RAW_MAX;
}

static int spike_gauge_get_raw_voltage(FAR struct spike_gauge_s *priv)
{
  uint16_t raw = stm32_adc_read(ADC_RANK_VBAT);
  int current_ma = spike_gauge_get_raw_current();
  int voltage_mv;

  voltage_mv = (int)raw * BAT_VOLTAGE_SCALED_MAX / BAT_VOLTAGE_RAW_MAX
             + current_ma * BAT_CURRENT_CORRECTION / 16;

  /* Suppress unreliable low voltage reading during first second after boot */

  if (voltage_mv < BAT_BOOT_SUPPRESS_MV)
    {
      uint32_t elapsed = TICK2MSEC(clock_systime_ticks() - priv->boot_ticks);
      if (elapsed < BAT_BOOT_SUPPRESS_MS)
        {
          voltage_mv = BAT_BOOT_SUPPRESS_MV;
        }
    }

  return voltage_mv;
}

/****************************************************************************
 * Name: spike_gauge_state
 ****************************************************************************/

static int spike_gauge_state(FAR struct battery_gauge_dev_s *dev,
                             FAR int *status)
{
  *status = BATTERY_DISCHARGING;
  return OK;
}

/****************************************************************************
 * Name: spike_gauge_online
 ****************************************************************************/

static int spike_gauge_online(FAR struct battery_gauge_dev_s *dev,
                              FAR bool *status)
{
  *status = true;
  return OK;
}

/****************************************************************************
 * Name: spike_gauge_voltage
 ****************************************************************************/

static int spike_gauge_voltage(FAR struct battery_gauge_dev_s *dev,
                               FAR int *value)
{
  FAR struct spike_gauge_s *priv =
    (FAR struct spike_gauge_s *)dev;

  *value = spike_gauge_get_raw_voltage(priv);
  return OK;
}

/****************************************************************************
 * Name: spike_gauge_current
 ****************************************************************************/

static int spike_gauge_current(FAR struct battery_gauge_dev_s *dev,
                               FAR int *value)
{
  *value = spike_gauge_get_raw_current();
  return OK;
}

/****************************************************************************
 * Name: spike_gauge_capacity
 *
 * Description:
 *   Estimate State of Charge (%) from battery voltage.
 *   Simple linear interpolation between cutoff and full voltage.
 ****************************************************************************/

static int spike_gauge_capacity(FAR struct battery_gauge_dev_s *dev,
                                FAR int *value)
{
  FAR struct spike_gauge_s *priv =
    (FAR struct spike_gauge_s *)dev;

  int mv = spike_gauge_get_raw_voltage(priv);

  if (mv >= BAT_FULL_MV)
    {
      *value = 100;
    }
  else if (mv <= BAT_CUTOFF_MV)
    {
      *value = 0;
    }
  else
    {
      *value = (mv - BAT_CUTOFF_MV) * 100 / (BAT_FULL_MV - BAT_CUTOFF_MV);
    }

  return OK;
}

/****************************************************************************
 * Name: spike_gauge_temp
 *
 * Description:
 *   Read battery temperature from NTC thermistor (103AT).
 *   Returns temperature in millidegrees Celsius.
 *
 *   Circuit: VREF — 2.4k — NTC_PIN — NTC — GND
 *                                   |
 *                                  7.5k — GND
 *
 *   R_NTC = 1 / ((4095/raw - 1) / 2400 - 1/7500)
 *   T(K)  = B / (ln(R_NTC / R0) + B / T0)
 *   where B=3435, R0=10k, T0=298.15K
 ****************************************************************************/

static int spike_gauge_temp(FAR struct battery_gauge_dev_s *dev,
                            FAR int *value)
{
  uint16_t raw = stm32_adc_read(ADC_RANK_NTC);

  if (raw == 0)
    {
      return -EIO;
    }

  float r_ntc = 1.0f / (((4095.0f / raw - 1.0f) / 2400.0f)
                         - 1.0f / 7500.0f);
  float t_kelvin = 3435.0f / (logf(r_ntc / 10000.0f) + 3435.0f / 298.15f);

  /* Convert Kelvin to millidegrees Celsius */

  *value = (int)((t_kelvin - 273.15f) * 1000.0f);
  return OK;
}

/****************************************************************************
 * Name: spike_gauge_chipid
 ****************************************************************************/

static int spike_gauge_chipid(FAR struct battery_gauge_dev_s *dev,
                              FAR unsigned int *value)
{
  return -ENOSYS;
}

/****************************************************************************
 * Name: spike_gauge_operate
 ****************************************************************************/

static int spike_gauge_operate(FAR struct battery_gauge_dev_s *dev,
                               FAR int *param)
{
  return -ENOSYS;
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct battery_gauge_operations_s g_spike_gauge_ops =
{
  .state    = spike_gauge_state,
  .online   = spike_gauge_online,
  .voltage  = spike_gauge_voltage,
  .capacity = spike_gauge_capacity,
  .current  = spike_gauge_current,
  .temp     = spike_gauge_temp,
  .chipid   = spike_gauge_chipid,
  .operate  = spike_gauge_operate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_battery_gauge_initialize
 *
 * Description:
 *   Initialize the battery gauge and register /dev/bat0.
 *   Must be called after stm32_adc_dma_initialize().
 ****************************************************************************/

int stm32_battery_gauge_initialize(void)
{
  static struct spike_gauge_s priv;
  int ret;

  priv.dev.ops = &g_spike_gauge_ops;
  priv.boot_ticks = clock_systime_ticks();

  ret = battery_gauge_register("/dev/bat0", &priv.dev);
  if (ret < 0)
    {
      return ret;
    }

  syslog(LOG_INFO, "BAT: gauge registered at /dev/bat0\n");
  return OK;
}

#endif /* CONFIG_BATTERY_GAUGE */
