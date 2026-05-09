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

#include <ctype.h>

#include <arch/board/board_legosensor.h>

#include "btsensor_cmd.h"
#include "btsensor_tx.h"
#include "btsensor_wire.h"
#include "bundle_emitter.h"
#include "imu_sampler.h"
#include "sensor_sampler.h"

#ifdef CONFIG_APP_BTSENSOR_SHELL_MODE
#  include "btsensor_shell.h"
#endif

#ifdef CONFIG_APP_CAPTURE
#  include "btsensor_capture_mode.h"
#endif

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

/* Parse a class identifier — accepts canonical names (color, ultrasonic,
 * force, motor_m, motor_r, motor_l) or numeric 0..5.  Returns 0 on
 * success, -EINVAL on parse failure.
 */

static int parse_class_id(const char *tok, uint8_t *out)
{
  if (tok == NULL)
    {
      return -EINVAL;
    }

  static const struct { const char *name; uint8_t id; } table[] =
  {
    { "color",      LEGOSENSOR_CLASS_COLOR      },
    { "ultrasonic", LEGOSENSOR_CLASS_ULTRASONIC },
    { "force",      LEGOSENSOR_CLASS_FORCE      },
    { "motor_m",    LEGOSENSOR_CLASS_MOTOR_M    },
    { "motor_r",    LEGOSENSOR_CLASS_MOTOR_R    },
    { "motor_l",    LEGOSENSOR_CLASS_MOTOR_L    },
  };

  for (size_t i = 0; i < sizeof(table) / sizeof(table[0]); i++)
    {
      if (strcasecmp(tok, table[i].name) == 0)
        {
          *out = table[i].id;
          return 0;
        }
    }

  /* Fallback: numeric 0..5. */

  char *end;
  long v = strtol(tok, &end, 10);
  if (*end != '\0' || v < 0 || v >= BTSENSOR_TLV_COUNT)
    {
      return -EINVAL;
    }

  *out = (uint8_t)v;
  return 0;
}

/* Hex decoder.  Tokens are read with strtok_r " ", so each token must
 * be even-length hex (no internal whitespace).  Returns total bytes on
 * success or a negated errno.  `out` must hold at least
 * BTSENSOR_TLV_PAYLOAD_MAX bytes.
 */

static int parse_hex_bytes(char **save, uint8_t *out, size_t cap)
{
  size_t total = 0;

  for (char *tok = strtok_r(NULL, " ", save);
       tok != NULL;
       tok = strtok_r(NULL, " ", save))
    {
      size_t toklen = strlen(tok);
      if ((toklen & 1U) != 0)
        {
          return -EINVAL;          /* odd hex digit count */
        }

      for (size_t i = 0; i < toklen; i += 2)
        {
          if (total >= cap)
            {
              return -E2BIG;
            }

          char c0 = tok[i];
          char c1 = tok[i + 1];
          if (!isxdigit((unsigned char)c0) || !isxdigit((unsigned char)c1))
            {
              return -EINVAL;
            }

          char pair[3] = { c0, c1, '\0' };
          out[total++] = (uint8_t)strtol(pair, NULL, 16);
        }
    }

  return total > 0 ? (int)total : -EINVAL;
}

/* Parse 1..4 signed integers in -10000..+10000 from successive
 * strtok tokens.  Values pass through verbatim into `int16_t` channel
 * slots — no scaling — matching `LEGOSENSOR_SET_PWM channels[]` and
 * `port pwm <P> set <duty>` directly.  COLOR / ULTRASONIC are
 * -ENOTSUP from SET_PWM (Issue #92) and use SEND mode=LIGHT instead;
 * MOTOR_* uses the full -10000..+10000 range; FORCE has no actuator.
 * Returns the count parsed, or a negated errno.
 */

static int parse_pwm_channels(char **save, int16_t *out, size_t cap)
{
  size_t n = 0;
  for (char *tok = strtok_r(NULL, " ", save);
       tok != NULL;
       tok = strtok_r(NULL, " ", save))
    {
      if (n >= cap)
        {
          return -E2BIG;
        }

      char *end;
      long v = strtol(tok, &end, 10);
      if (*end != '\0' || v < -10000 || v > 10000)
        {
          return -EINVAL;
        }

      /* BT-side PWM units are now raw kernel duty (-10000..10000,
       * .01 % units), matching `LEGOSENSOR_SET_PWM channels[]` and
       * `port pwm <P> set <duty>` directly.  The previous mapping
       * (-100..100 percent → ×100 scale here) was confusing across
       * the CLI / BT / kernel boundary; #92 unifies on raw units.
       */

      out[n++] = (int16_t)v;
    }

  return n > 0 ? (int)n : -EINVAL;
}

static void cmd_sensor_mode(char **save)
{
  char *cls_tok  = strtok_r(NULL, " ", save);
  char *mode_tok = strtok_r(NULL, " ", save);

  uint8_t class_id;
  if (parse_class_id(cls_tok, &class_id) < 0)
    {
      reply("ERR invalid class\n");
      return;
    }

  if (mode_tok == NULL)
    {
      reply("ERR invalid mode\n");
      return;
    }

  char *end;
  long mode = strtol(mode_tok, &end, 10);
  if (*end != '\0' || mode < 0 || mode > 7)
    {
      reply("ERR invalid mode\n");
      return;
    }

  reply_rc(sensor_sampler_select_mode(class_id, (uint8_t)mode),
           "SENSOR MODE");
}

static void cmd_sensor_send(char **save)
{
  char *cls_tok  = strtok_r(NULL, " ", save);
  char *mode_tok = strtok_r(NULL, " ", save);

  uint8_t class_id;
  if (parse_class_id(cls_tok, &class_id) < 0)
    {
      reply("ERR invalid class\n");
      return;
    }

  if (mode_tok == NULL)
    {
      reply("ERR invalid mode\n");
      return;
    }

  char *end;
  long mode = strtol(mode_tok, &end, 10);
  if (*end != '\0' || mode < 0 || mode > 7)
    {
      reply("ERR invalid mode\n");
      return;
    }

  uint8_t buf[BTSENSOR_TLV_PAYLOAD_MAX];
  int n = parse_hex_bytes(save, buf, sizeof(buf));
  if (n < 0)
    {
      reply(n == -E2BIG ? "ERR payload too long\n" : "ERR invalid hex\n");
      return;
    }

  reply_rc(sensor_sampler_send(class_id, (uint8_t)mode, buf, (size_t)n),
           "SENSOR SEND");
}

static void cmd_sensor_pwm(char **save)
{
  char *cls_tok = strtok_r(NULL, " ", save);

  uint8_t class_id;
  if (parse_class_id(cls_tok, &class_id) < 0)
    {
      reply("ERR invalid class\n");
      return;
    }

  int16_t channels[4];
  int n = parse_pwm_channels(save, channels, 4);
  if (n < 0)
    {
      reply(n == -E2BIG ? "ERR too many channels\n"
                        : "ERR invalid pwm\n");
      return;
    }

  reply_rc(sensor_sampler_set_pwm(class_id, channels, (size_t)n),
           "SENSOR PWM");
}

static void cmd_sensor(char *arg, char **save)
{
  if (arg == NULL)
    {
      reply("ERR invalid SENSOR\n");
      return;
    }

  if (strcasecmp(arg, "ON") == 0)
    {
      reply_rc(bundle_emitter_set_sensor_enabled(true), "SENSOR ON");
      return;
    }

  if (strcasecmp(arg, "OFF") == 0)
    {
      reply_rc(bundle_emitter_set_sensor_enabled(false), "SENSOR OFF");
      return;
    }

  if (strcasecmp(arg, "MODE") == 0)
    {
      cmd_sensor_mode(save);
      return;
    }

  if (strcasecmp(arg, "SEND") == 0)
    {
      cmd_sensor_send(save);
      return;
    }

  if (strcasecmp(arg, "PWM") == 0)
    {
      cmd_sensor_pwm(save);
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

#ifdef CONFIG_APP_BTSENSOR_SHELL_MODE
static void cmd_mode(char *arg)
{
  if (arg == NULL)
    {
      reply("ERR invalid MODE\n");
      return;
    }

  if (strcasecmp(arg, "SHELL") == 0)
    {
      if (btsensor_shell_is_active())
        {
          reply("ERR already_shell\n");
          return;
        }

      /* Pump auto-OFF (Codex 3rd review Q4 — does not auto-resume on
       * exit; user re-enables explicitly).  Both calls are idempotent.
       */

      (void)bundle_emitter_set_imu_enabled(false);
      (void)bundle_emitter_set_sensor_enabled(false);

      int rc = btsensor_shell_enter();
      if (rc < 0)
        {
          char buf[BTSENSOR_CMD_MAX_LINE];
          snprintf(buf, sizeof(buf), "ERR shell_enter %d\n", -rc);
          reply(buf);
          return;
        }

      /* Enqueue OK\n while still in MODE_SHELL_STARTING — the
       * post_drain_callback hook below flips into MODE_SHELL only
       * after the response has been physically dispatched.
       */

      int qrc = btsensor_tx_enqueue_response("OK\n");
      if (qrc != 0)
        {
          /* TX queue full (ENOSPC) — tear shell down so the system is
           * back in TELEMETRY before sending the failure reply.
           */

          btsensor_shell_exit_async(BTSENSOR_SHELL_REASON_DAEMON_STOP);
          reply("ERR shell_no_buffer\n");
          return;
        }

      int arc = btsensor_tx_arm_post_drain_callback(
                    btsensor_shell_transition_to_active,
                    btsensor_shell_drain_timeout,
                    NULL, 500);
      if (arc < 0)
        {
          syslog(LOG_WARNING,
                 "btsensor: arm_post_drain_callback rc=%d\n", arc);
        }

      /* Drop any remaining bytes from the same RFCOMM packet — the peer
       * is contractually not allowed to send anything after MODE SHELL
       * until OK\n is received (Codex 3rd review Q8).
       */

      btsensor_cmd_reset_rx_buffer();
      return;
    }

  if (strcasecmp(arg, "TELEMETRY") == 0)
    {
      /* If we got here we are by definition in TELEMETRY mode (cmd
       * parser is bypassed in SHELL_STARTING/SHELL).  No-op + OK.
       */

      reply("OK\n");
      return;
    }

#ifdef CONFIG_APP_CAPTURE
  if (strcasecmp(arg, "CAPTURE") == 0)
    {
      /* Drain whatever session apps/capture has staged on /dev/btcap.
       * The handler returns immediately after registering its data
       * source; the actual byte-stream forwarding happens off the
       * run-loop poll, framed by BTCS / BTCE on the BT side.  -ENOENT
       * means there was no writer in flight; -EBUSY means another
       * MODE switch (typically SHELL) is in progress.
       */

      int rc = btsensor_capture_mode_enter();
      if (rc == 0)
        {
          /* No "OK\n" reply on success.  mode_enter has already
           * queued the BTCS + 40-byte meta blob; appending an "OK\n"
           * here would sandwich those three bytes between the meta
           * and the .cap "CAPB" header, causing the host scanner's
           * `total_bytes` slice to misalign and reject the session
           * (Issue #122 follow-up).  The BTCS marker frame itself is
           * the implicit ack — host code keys off that.
           */
        }
      else if (rc == -ENOENT)
        {
          reply("ERR no_capture_session\n");
        }
      else if (rc == -EBUSY)
        {
          reply("ERR busy\n");
        }
      else
        {
          char rbuf[BTSENSOR_CMD_MAX_LINE];
          snprintf(rbuf, sizeof(rbuf), "ERR mode_capture %d\n", rc);
          reply(rbuf);
        }

      return;
    }
#endif

  char buf[BTSENSOR_CMD_MAX_LINE];
  snprintf(buf, sizeof(buf), "ERR invalid MODE %s\n", arg);
  reply(buf);
}
#endif /* CONFIG_APP_BTSENSOR_SHELL_MODE */

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
      cmd_sensor(strtok_r(NULL, " ", &save), &save);
      return;
    }

  if (strcmp(cmd, "SET") == 0)
    {
      char *what  = strtok_r(NULL, " ", &save);
      char *value = strtok_r(NULL, " ", &save);
      cmd_set(what, value);
      return;
    }

#ifdef CONFIG_APP_BTSENSOR_SHELL_MODE
  if (strcasecmp(cmd, "MODE") == 0)
    {
      cmd_mode(strtok_r(NULL, " ", &save));
      return;
    }
#endif

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

void btsensor_cmd_reset_rx_buffer(void)
{
  /* Used by the shell module on STARTING transition and on shell exit
   * cleanup to make sure no half-line bytes leak across mode changes
   * (Codex 3rd review Q5/Q8).
   */

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

#ifdef CONFIG_APP_BTSENSOR_SHELL_MODE
          /* If process_line() flipped us into shell-active state (e.g.
           * via MODE SHELL), discard any remaining bytes from the
           * current RFCOMM packet — the peer is contractually
           * forbidden from sending shell stdin until OK\n arrives
           * (Codex 3rd review Q8).
           */

          if (btsensor_shell_is_active())
            {
              return;
            }
#endif
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
