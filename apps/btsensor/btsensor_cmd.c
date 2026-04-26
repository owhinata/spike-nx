/****************************************************************************
 * apps/btsensor/btsensor_cmd.c
 *
 * PC -> Hub ASCII command parser for btsensor (Issue #56 Commit D).
 *
 * The command set is intentionally tiny — toggle IMU sampling and
 * adjust ODR / FSR / batch — so the parser is just a 64-byte line
 * buffer fed character at a time, then strtok_r dispatch.  All
 * mutations route through imu_sampler_set_*() so the cache and the
 * driver stay in sync, and replies are queued via the btsensor_tx
 * arbiter so they have priority over IMU telemetry.
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
      reply_rc(imu_sampler_set_enabled(true), "IMU ON");
      return;
    }

  if (strcmp(arg, "OFF") == 0)
    {
      reply_rc(imu_sampler_set_enabled(false), "IMU OFF");
      return;
    }

  char buf[BTSENSOR_CMD_MAX_LINE];
  snprintf(buf, sizeof(buf), "ERR invalid %s\n", arg);
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

  if (strcmp(what, "BATCH") == 0)
    {
      reply_rc(imu_sampler_set_batch((uint8_t)v), "BATCH");
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
          /* Latch overflow until the next '\n' so we don't echo a
           * stream of ERRs while a long line is still arriving.
           */

          g_overflowed = true;
        }
    }
}
