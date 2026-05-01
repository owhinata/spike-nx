/****************************************************************************
 * apps/btsensor/btsensor_cmd.c
 *
 * PC -> Hub ASCII command parser for btsensor.
 *
 * Tiny line-based protocol (64-byte buffer, '\n' delimited, '\r'
 * ignored).  Mutations route through bundle_emitter / imu_sampler so
 * the BUNDLE wire flags and the underlying driver stay in sync, and
 * replies are queued via btsensor_tx so they have priority over
 * telemetry.
 *
 *   IMU ON | OFF             toggle IMU streaming
 *   SENSOR ON | OFF          toggle LEGO sensor streaming (Issue B fills
 *                            in TLV publish tracking; Issue A just
 *                            records the flag)
 *   SET ODR <hz>             ODR (only while IMU OFF, must be <=833)
 *   SET ACCEL_FSR <g>        accel FSR (only while IMU OFF)
 *   SET GYRO_FSR <dps>       gyro FSR (only while IMU OFF)
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "btsensor_cmd.h"
#include "btsensor_tx.h"
#include "bundle_emitter.h"
#include "imu_sampler.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static char     g_line[BTSENSOR_CMD_MAX_LINE];
static uint16_t g_line_len;
static bool     g_overflowed;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void reply(const char *line)
{
  (void)btsensor_tx_enqueue_response(line);
}

static void reply_rc(int rc, const char *what)
{
  if (rc == 0)
    {
      reply("OK\n");
      return;
    }

  char buf[BTSENSOR_CMD_MAX_LINE];
  switch (rc)
    {
      case -EBUSY:
        reply("ERR busy\n");
        return;
      case -EINVAL:
        snprintf(buf, sizeof(buf), "ERR invalid %s\n",
                 what != NULL ? what : "");
        reply(buf);
        return;
      default:
        snprintf(buf, sizeof(buf), "ERR errno=%d\n", -rc);
        reply(buf);
        return;
    }
}

static void cmd_imu(char *arg)
{
  if (arg == NULL)
    {
      reply("ERR invalid IMU\n");
      return;
    }

  if (strcmp(arg, "ON") == 0)
    {
      reply_rc(bundle_emitter_set_imu_enabled(true), "IMU ON");
      return;
    }

  if (strcmp(arg, "OFF") == 0)
    {
      reply_rc(bundle_emitter_set_imu_enabled(false), "IMU OFF");
      return;
    }

  char buf[BTSENSOR_CMD_MAX_LINE];
  snprintf(buf, sizeof(buf), "ERR invalid %s\n", arg);
  reply(buf);
}

static void cmd_sensor(char *arg)
{
  if (arg == NULL)
    {
      reply("ERR invalid SENSOR\n");
      return;
    }

  if (strcmp(arg, "ON") == 0)
    {
      reply_rc(bundle_emitter_set_sensor_enabled(true), "SENSOR ON");
      return;
    }

  if (strcmp(arg, "OFF") == 0)
    {
      reply_rc(bundle_emitter_set_sensor_enabled(false), "SENSOR OFF");
      return;
    }

  /* MODE / SEND / PWM tokens are reserved for Issue C; surface a
   * distinct error so the host UI can hide the corresponding controls
   * until that lands.
   */

  char buf[BTSENSOR_CMD_MAX_LINE];
  snprintf(buf, sizeof(buf), "ERR not_supported %s\n", arg);
  reply(buf);
}

static void cmd_set(char *what, char *value_str)
{
  if (what == NULL || value_str == NULL)
    {
      reply("ERR invalid SET\n");
      return;
    }

  long v = strtol(value_str, NULL, 10);
  if (v < 0)
    {
      reply("ERR invalid value\n");
      return;
    }

  if (strcmp(what, "ODR") == 0)
    {
      reply_rc(imu_sampler_set_odr_hz((uint32_t)v), "ODR");
      return;
    }

  if (strcmp(what, "ACCEL_FSR") == 0)
    {
      reply_rc(imu_sampler_set_accel_fsr((uint32_t)v), "ACCEL_FSR");
      return;
    }

  if (strcmp(what, "GYRO_FSR") == 0)
    {
      reply_rc(imu_sampler_set_gyro_fsr((uint32_t)v), "GYRO_FSR");
      return;
    }

  char buf[BTSENSOR_CMD_MAX_LINE];
  snprintf(buf, sizeof(buf), "ERR invalid %s\n", what);
  reply(buf);
}

static void process_line(char *line)
{
  syslog(LOG_INFO, "btsensor_cmd: %s\n", line);

  char *save;
  char *cmd = strtok_r(line, " ", &save);
  if (cmd == NULL)
    {
      return;             /* empty line — silent */
    }

  if (strcmp(cmd, "IMU") == 0)
    {
      cmd_imu(strtok_r(NULL, " ", &save));
      return;
    }

  if (strcmp(cmd, "SENSOR") == 0)
    {
      cmd_sensor(strtok_r(NULL, " ", &save));
      return;
    }

  if (strcmp(cmd, "SET") == 0)
    {
      char *what  = strtok_r(NULL, " ", &save);
      char *value = strtok_r(NULL, " ", &save);
      cmd_set(what, value);
      return;
    }

  char buf[BTSENSOR_CMD_MAX_LINE];
  snprintf(buf, sizeof(buf), "ERR unknown %s\n", cmd);
  reply(buf);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void btsensor_cmd_init(void)
{
  g_line_len   = 0;
  g_overflowed = false;
}

void btsensor_cmd_feed(const uint8_t *data, uint16_t len)
{
  for (uint16_t i = 0; i < len; i++)
    {
      char c = (char)data[i];

      if (c == '\r')
        {
          continue;
        }

      if (c == '\n')
        {
          if (g_overflowed)
            {
              reply("ERR overflow\n");
              g_overflowed = false;
            }
          else if (g_line_len > 0)
            {
              g_line[g_line_len] = '\0';
              process_line(g_line);
            }

          g_line_len = 0;
          continue;
        }

      if (g_line_len < BTSENSOR_CMD_MAX_LINE - 1)
        {
          g_line[g_line_len++] = c;
        }
      else
        {
          g_overflowed = true;
        }
    }
}
