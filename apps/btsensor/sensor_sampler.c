/****************************************************************************
 * apps/btsensor/sensor_sampler.c
 *
 * Issue A stub for the LEGO sensor uORB snapshot interface.  Provides
 * a working sensor_sampler_set_enabled() (so SENSOR ON / OFF round-trip
 * correctly) and a snapshot() implementation that always reports the
 * 6 classes as unbound — Issue B will plug in the real fd / dirty-
 * tracking logic.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <string.h>
#include <syslog.h>

#include "btsensor_wire.h"
#include "sensor_sampler.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static bool g_initialized;
static bool g_enabled;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int sensor_sampler_init(void)
{
  if (g_initialized)
    {
      return 0;
    }

  g_enabled     = false;
  g_initialized = true;
  return 0;
}

void sensor_sampler_deinit(void)
{
  if (!g_initialized)
    {
      return;
    }

  if (g_enabled)
    {
      sensor_sampler_set_enabled(false);
    }

  g_initialized = false;
}

int sensor_sampler_set_enabled(bool on)
{
  if (!g_initialized)
    {
      return -EINVAL;
    }

  if (on == g_enabled)
    {
      return 0;
    }

  g_enabled = on;
  syslog(LOG_INFO, "btsensor: SENSOR sampling %s (Issue A stub)\n",
         on ? "on" : "off");
  return 0;
}

bool sensor_sampler_is_enabled(void)
{
  return g_enabled;
}

void sensor_sampler_snapshot(struct sensor_class_state_s out[BTSENSOR_TLV_COUNT])
{
  /* Issue A: every class is reported as unbound with no payload.
   * bundle_emitter will write the class_id from the table index, which
   * matches enum legosensor_class_e (COLOR=0 .. MOTOR_L=5).
   */

  memset(out, 0, sizeof(struct sensor_class_state_s) * BTSENSOR_TLV_COUNT);
  for (uint8_t i = 0; i < BTSENSOR_TLV_COUNT; i++)
    {
      out[i].port_id   = 0xFF;
      out[i].age_10ms  = BTSENSOR_TLV_AGE_SATURATED;
    }
}
