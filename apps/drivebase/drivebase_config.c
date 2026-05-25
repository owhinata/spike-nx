/****************************************************************************
 * apps/drivebase/drivebase_config.c
 *
 * Properties-format runtime config loader (Issue #143).  See header for
 * the wire-format contract.  Implementation notes:
 *
 *   - The parser is single-pass and stateless across lines; bad lines
 *     are skipped individually rather than rejecting the whole file
 *     (codex-review feedback #143).
 *
 *   - All values applied through db_settings_set_* setters, which
 *     enforce per-key range validation and reject writes after
 *     db_settings_freeze().  This module never touches the live
 *     structs directly.
 *
 *   - Logging goes through syslog: file-missing is silent (the
 *     overwhelmingly common case on first boot or fresh image);
 *     parse errors and unknown keys are LOG_WARNING; a successful
 *     load that applied >= 1 key emits a single LOG_INFO summary so
 *     dmesg makes it easy to confirm config actually took effect.
 ****************************************************************************/

#include <nuttx/config.h>

#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <syslog.h>

#include "drivebase_config.h"
#include "drivebase_settings.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Setter signature used by the dispatch table.  Returns 0 on success,
 * negative errno on failure; the caller logs and continues.
 */

typedef int (*db_config_setter_t)(const char *value);

struct db_config_entry_s
{
  const char         *key;
  db_config_setter_t  setter;
};

/****************************************************************************
 * File-scope state
 ****************************************************************************/

static struct db_config_start_defaults_s g_start_defaults;

/****************************************************************************
 * Value parsing helpers
 ****************************************************************************/

static int parse_i32(const char *s, int32_t *out)
{
  if (s == NULL || *s == '\0') return -EINVAL;
  errno = 0;
  char *end = NULL;
  long v = strtol(s, &end, 10);
  if (errno != 0) return -EINVAL;
  if (end == s) return -EINVAL;
  /* Reject trailing garbage after the integer */
  while (*end != '\0')
    {
      if (!isspace((unsigned char)*end)) return -EINVAL;
      end++;
    }
  if (v < INT32_MIN || v > INT32_MAX) return -EINVAL;
  *out = (int32_t)v;
  return 0;
}

static int parse_u32(const char *s, uint32_t *out)
{
  int32_t v;
  int rc = parse_i32(s, &v);
  if (rc < 0) return rc;
  if (v < 0) return -EINVAL;
  *out = (uint32_t)v;
  return 0;
}

/****************************************************************************
 * Per-key setters (thin wrappers that call into db_settings_set_*)
 ****************************************************************************/

#define DEF_PID_SETTER_I32(NAME, AXIS, FN)                              \
  static int set_##NAME(const char *value)                              \
  {                                                                     \
    int32_t v;                                                          \
    int rc = parse_i32(value, &v);                                      \
    if (rc < 0) return rc;                                              \
    return FN(AXIS, v);                                                 \
  }

#define DEF_COMP_SETTER_I32(NAME, AXIS, FN)                             \
  static int set_##NAME(const char *value)                              \
  {                                                                     \
    int32_t v;                                                          \
    int rc = parse_i32(value, &v);                                      \
    if (rc < 0) return rc;                                              \
    return FN(AXIS, v);                                                 \
  }

#define DEF_COMP_SETTER_U32(NAME, AXIS, FN)                             \
  static int set_##NAME(const char *value)                              \
  {                                                                     \
    uint32_t v;                                                         \
    int rc = parse_u32(value, &v);                                      \
    if (rc < 0) return rc;                                              \
    return FN(AXIS, v);                                                 \
  }

DEF_PID_SETTER_I32(pid_dist_kp_pos,         DB_AXIS_DISTANCE, db_settings_set_pid_kp_pos)
DEF_PID_SETTER_I32(pid_dist_ki_pos,         DB_AXIS_DISTANCE, db_settings_set_pid_ki_pos)
DEF_PID_SETTER_I32(pid_dist_kd_pos,         DB_AXIS_DISTANCE, db_settings_set_pid_kd_pos)
DEF_PID_SETTER_I32(pid_dist_kp_speed,       DB_AXIS_DISTANCE, db_settings_set_pid_kp_speed)
DEF_PID_SETTER_I32(pid_dist_ki_speed,       DB_AXIS_DISTANCE, db_settings_set_pid_ki_speed)
DEF_PID_SETTER_I32(pid_dist_deadband_mdeg,  DB_AXIS_DISTANCE, db_settings_set_pid_deadband_mdeg)
DEF_PID_SETTER_I32(pid_dist_out_max,        DB_AXIS_DISTANCE, db_settings_set_pid_out_max)

DEF_PID_SETTER_I32(pid_head_kp_pos,         DB_AXIS_HEADING,  db_settings_set_pid_kp_pos)
DEF_PID_SETTER_I32(pid_head_ki_pos,         DB_AXIS_HEADING,  db_settings_set_pid_ki_pos)
DEF_PID_SETTER_I32(pid_head_kd_pos,         DB_AXIS_HEADING,  db_settings_set_pid_kd_pos)
DEF_PID_SETTER_I32(pid_head_kp_speed,       DB_AXIS_HEADING,  db_settings_set_pid_kp_speed)
DEF_PID_SETTER_I32(pid_head_ki_speed,       DB_AXIS_HEADING,  db_settings_set_pid_ki_speed)
DEF_PID_SETTER_I32(pid_head_deadband_mdeg,  DB_AXIS_HEADING,  db_settings_set_pid_deadband_mdeg)
DEF_PID_SETTER_I32(pid_head_out_max,        DB_AXIS_HEADING,  db_settings_set_pid_out_max)

DEF_COMP_SETTER_I32(comp_dist_pos_tol_mdeg,        DB_AXIS_DISTANCE, db_settings_set_comp_pos_tol_mdeg)
DEF_COMP_SETTER_I32(comp_dist_speed_tol_mdegps,    DB_AXIS_DISTANCE, db_settings_set_comp_speed_tol_mdegps)
DEF_COMP_SETTER_I32(comp_dist_smart_continue_mdeg, DB_AXIS_DISTANCE, db_settings_set_comp_smart_continue_mdeg)
DEF_COMP_SETTER_U32(comp_dist_done_window_ms,      DB_AXIS_DISTANCE, db_settings_set_comp_done_window_ms)
DEF_COMP_SETTER_U32(comp_dist_smart_passive_hold_ms, DB_AXIS_DISTANCE, db_settings_set_comp_smart_passive_hold_ms)

DEF_COMP_SETTER_I32(comp_head_pos_tol_mdeg,        DB_AXIS_HEADING, db_settings_set_comp_pos_tol_mdeg)
DEF_COMP_SETTER_I32(comp_head_speed_tol_mdegps,    DB_AXIS_HEADING, db_settings_set_comp_speed_tol_mdegps)
DEF_COMP_SETTER_I32(comp_head_smart_continue_mdeg, DB_AXIS_HEADING, db_settings_set_comp_smart_continue_mdeg)
DEF_COMP_SETTER_U32(comp_head_done_window_ms,      DB_AXIS_HEADING, db_settings_set_comp_done_window_ms)
DEF_COMP_SETTER_U32(comp_head_smart_passive_hold_ms, DB_AXIS_HEADING, db_settings_set_comp_smart_passive_hold_ms)

/* Feed-forward gains (Issue #127 Phase 6 Step 6.1).  Reuse the same
 * (axis, setter) wrapper macro pattern as PID gains.
 */

DEF_PID_SETTER_I32(ff_dist_kV, DB_AXIS_DISTANCE, db_settings_set_ff_kV)
DEF_PID_SETTER_I32(ff_dist_kA, DB_AXIS_DISTANCE, db_settings_set_ff_kA)
DEF_PID_SETTER_I32(ff_head_kV, DB_AXIS_HEADING,  db_settings_set_ff_kV)
DEF_PID_SETTER_I32(ff_head_kA, DB_AXIS_HEADING,  db_settings_set_ff_kA)

/* Per-motor friction FF (Step 6.2).  Common to both motors, no axis
 * argument — the setter signature stays the simple one-value form.
 */

static int set_ff_motor_kS(const char *value)
{
  int32_t v;
  int rc = parse_i32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_ff_kS(v);
}

static int set_ff_v_hyst_enter_mdegps(const char *value)
{
  int32_t v;
  int rc = parse_i32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_ff_v_hyst_enter_mdegps(v);
}

static int set_ff_v_hyst_exit_mdegps(const char *value)
{
  int32_t v;
  int rc = parse_i32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_ff_v_hyst_exit_mdegps(v);
}

/* Battery sag correction (Step 6.3). */

static int set_battery_nominal_mv(const char *value)
{
  int32_t v;
  int rc = parse_i32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_battery_nominal_mv(v);
}

static int set_battery_min_mv(const char *value)
{
  int32_t v;
  int rc = parse_i32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_battery_min_mv(v);
}

static int set_stall_speed_mdegps(const char *value)
{
  int32_t v;
  int rc = parse_i32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_stall_speed_mdegps(v);
}

static int set_stall_duty_min(const char *value)
{
  int32_t v;
  int rc = parse_i32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_stall_duty_min(v);
}

static int set_stall_window_ms(const char *value)
{
  uint32_t v;
  int rc = parse_u32(value, &v);
  if (rc < 0) return rc;
  return db_settings_set_stall_window_ms(v);
}

static int set_wheel_d_um(const char *value)
{
  uint32_t v;
  int rc = parse_u32(value, &v);
  if (rc < 0) return rc;
  if (v == 0) return -EINVAL;
  g_start_defaults.wheel_d_um = v;
  return 0;
}

static int set_axle_t_um(const char *value)
{
  uint32_t v;
  int rc = parse_u32(value, &v);
  if (rc < 0) return rc;
  if (v == 0) return -EINVAL;
  g_start_defaults.axle_t_um = v;
  return 0;
}

static int set_tick_us(const char *value)
{
  uint32_t v;
  int rc = parse_u32(value, &v);
  if (rc < 0) return rc;
  if (v == 0) return -EINVAL;
  g_start_defaults.tick_us = v;
  return 0;
}

/* Phase 3b (#148): use_gyro_plus1 ∈ {0,1,3}.  See struct
 * db_config_start_defaults_s comment for the encoding.  Issue #157
 * removed 1D: a legacy value of 2 (old 1D) is aliased to 3 (3D) — the
 * fused-projection heading is identical — so existing cfg files keep
 * gyro-on at boot instead of silently dropping to encoder-only.
 */

static int set_use_gyro_plus1(const char *value)
{
  uint32_t v;
  int rc = parse_u32(value, &v);
  if (rc < 0) return rc;
  if (v > 3) return -EINVAL;
  if (v == 2)
    {
      syslog(LOG_WARNING,
             "drivebase cfg: use_gyro_plus1=2 (1D) is deprecated; "
             "treating as 3 (3D)\n");
      v = 3;
    }
  g_start_defaults.use_gyro_plus1 = (uint8_t)v;
  return 0;
}

/****************************************************************************
 * Dispatch table
 ****************************************************************************/

static const struct db_config_entry_s g_config_table[] =
{
  { "pid_dist_kp_pos",                set_pid_dist_kp_pos },
  { "pid_dist_ki_pos",                set_pid_dist_ki_pos },
  { "pid_dist_kd_pos",                set_pid_dist_kd_pos },
  { "pid_dist_kp_speed",              set_pid_dist_kp_speed },
  { "pid_dist_ki_speed",              set_pid_dist_ki_speed },
  { "pid_dist_deadband_mdeg",         set_pid_dist_deadband_mdeg },
  { "pid_dist_out_max",               set_pid_dist_out_max },

  { "pid_head_kp_pos",                set_pid_head_kp_pos },
  { "pid_head_ki_pos",                set_pid_head_ki_pos },
  { "pid_head_kd_pos",                set_pid_head_kd_pos },
  { "pid_head_kp_speed",              set_pid_head_kp_speed },
  { "pid_head_ki_speed",              set_pid_head_ki_speed },
  { "pid_head_deadband_mdeg",         set_pid_head_deadband_mdeg },
  { "pid_head_out_max",               set_pid_head_out_max },

  { "comp_dist_pos_tol_mdeg",         set_comp_dist_pos_tol_mdeg },
  { "comp_dist_speed_tol_mdegps",     set_comp_dist_speed_tol_mdegps },
  { "comp_dist_smart_continue_mdeg",  set_comp_dist_smart_continue_mdeg },
  { "comp_dist_done_window_ms",       set_comp_dist_done_window_ms },
  { "comp_dist_smart_passive_hold_ms", set_comp_dist_smart_passive_hold_ms },

  { "comp_head_pos_tol_mdeg",         set_comp_head_pos_tol_mdeg },
  { "comp_head_speed_tol_mdegps",     set_comp_head_speed_tol_mdegps },
  { "comp_head_smart_continue_mdeg",  set_comp_head_smart_continue_mdeg },
  { "comp_head_done_window_ms",       set_comp_head_done_window_ms },
  { "comp_head_smart_passive_hold_ms", set_comp_head_smart_passive_hold_ms },

  { "ff_dist_kV",                     set_ff_dist_kV },
  { "ff_dist_kA",                     set_ff_dist_kA },
  { "ff_head_kV",                     set_ff_head_kV },
  { "ff_head_kA",                     set_ff_head_kA },
  { "ff_motor_kS",                    set_ff_motor_kS },
  { "ff_v_hyst_enter_mdegps",         set_ff_v_hyst_enter_mdegps },
  { "ff_v_hyst_exit_mdegps",          set_ff_v_hyst_exit_mdegps },

  { "battery_nominal_mv",             set_battery_nominal_mv },
  { "battery_min_mv",                 set_battery_min_mv },

  { "stall_speed_mdegps",             set_stall_speed_mdegps },
  { "stall_duty_min",                 set_stall_duty_min },
  { "stall_window_ms",                set_stall_window_ms },

  { "wheel_d_um",                     set_wheel_d_um },
  { "axle_t_um",                      set_axle_t_um },
  { "tick_us",                        set_tick_us },

  { "use_gyro_plus1",                 set_use_gyro_plus1 },
};

#define DB_CONFIG_TABLE_SIZE \
  (sizeof(g_config_table) / sizeof(g_config_table[0]))

/****************************************************************************
 * Line processing
 ****************************************************************************/

static char *lstrip(char *s)
{
  while (*s != '\0' && isspace((unsigned char)*s)) s++;
  return s;
}

static void rstrip(char *s)
{
  size_t n = strlen(s);
  while (n > 0 && isspace((unsigned char)s[n - 1])) s[--n] = '\0';
}

static const struct db_config_entry_s *find_entry(const char *key)
{
  for (size_t i = 0; i < DB_CONFIG_TABLE_SIZE; i++)
    {
      if (strcmp(g_config_table[i].key, key) == 0)
        {
          return &g_config_table[i];
        }
    }
  return NULL;
}

/* Returns 1 if the line resulted in a successful key application,
 * 0 if it was a no-op (blank / comment), -1 if it was logged as an error.
 */

static int process_line(char *line, unsigned line_no)
{
  /* Strip leading whitespace and the optional UTF-8 BOM on line 1. */

  if (line_no == 1 &&
      (unsigned char)line[0] == 0xEF &&
      (unsigned char)line[1] == 0xBB &&
      (unsigned char)line[2] == 0xBF)
    {
      line += 3;
    }

  char *p = lstrip(line);

  /* Skip blank lines and full-line comments. */

  if (*p == '\0' || *p == '#') return 0;

  /* Split on the first '='. */

  char *eq = strchr(p, '=');
  if (eq == NULL)
    {
      syslog(LOG_WARNING,
             "drivebase: config line %u: missing '='\n", line_no);
      return -1;
    }

  *eq = '\0';
  char *key = p;
  char *val = eq + 1;

  /* Strip an inline `#` comment from the value: anything from the first
   * '#' to end-of-line is dropped before rstrip / number parsing.  Keys
   * holding integers can never legitimately contain '#', so this is
   * safe; if the schema ever grows string-valued keys, revisit.
   */

  char *hash = strchr(val, '#');
  if (hash != NULL) *hash = '\0';

  rstrip(key);
  val = lstrip(val);
  rstrip(val);

  if (*key == '\0')
    {
      syslog(LOG_WARNING,
             "drivebase: config line %u: empty key\n", line_no);
      return -1;
    }

  const struct db_config_entry_s *entry = find_entry(key);
  if (entry == NULL)
    {
      syslog(LOG_WARNING,
             "drivebase: config line %u: unknown key '%s'\n",
             line_no, key);
      return -1;
    }

  int rc = entry->setter(val);
  if (rc < 0)
    {
      syslog(LOG_WARNING,
             "drivebase: config line %u: key '%s' value '%s' rejected "
             "(%d)\n", line_no, key, val, rc);
      return -1;
    }
  return 1;
}

/****************************************************************************
 * Public API
 ****************************************************************************/

int db_config_load(const char *path)
{
  memset(&g_start_defaults, 0, sizeof(g_start_defaults));

  /* Phase 6 production default: gyro-locked heading at boot.  Phase
   * 2.5/3a/3b shipped the Madgwick fusion + heading-PID injection so
   * the typical drivebase use case wants gyro heading from the first
   * command.  cfg can still override to 1 (NONE) explicitly if the
   * IMU is intentionally disabled; this just changes the unedited-
   * cfg default.  See drivebase_daemon.c for the `use_gyro_plus1 - 1`
   * decode and the latched-on-first-command flow.  Issue #157: 3D is
   * now the only gyro mode (1D removed).
   */

  g_start_defaults.use_gyro_plus1 = 3;   /* +1 encoding: 3 = 3D    */

  if (path == NULL) path = DB_CONFIG_DEFAULT_PATH;

  FILE *fp = fopen(path, "r");
  if (fp == NULL)
    {
      /* ENOENT is the overwhelmingly common case (no config installed
       * yet) — silent fallback per Issue #143 spec.  Any other errno is
       * worth a warning so a mount issue or path typo is visible.
       */

      if (errno == ENOENT) return 0;
      syslog(LOG_WARNING,
             "drivebase: cannot open config '%s': %d\n", path, errno);
      return 0;
    }

  unsigned line_no = 0;
  unsigned applied = 0;
  unsigned bad     = 0;
  char buf[DB_CONFIG_LINE_MAX + 2];   /* +1 for '\n', +1 for terminator */

  while (fgets(buf, sizeof(buf), fp) != NULL)
    {
      line_no++;

      /* Detect overlong lines: fgets reads until newline OR buffer-1.
       * If we filled the buffer and the last char isn't a newline, the
       * physical line extends beyond DB_CONFIG_LINE_MAX.  Skip the rest
       * of the physical line and warn.
       */

      size_t n = strlen(buf);
      bool overlong = (n == sizeof(buf) - 1 && buf[n - 1] != '\n');
      if (overlong)
        {
          syslog(LOG_WARNING,
                 "drivebase: config line %u: exceeds %d bytes, skipped\n",
                 line_no, DB_CONFIG_LINE_MAX);
          int c;
          while ((c = fgetc(fp)) != EOF && c != '\n') { }
          bad++;
          continue;
        }

      /* Strip CR (CRLF endings) and the trailing LF.  rstrip() in
       * process_line handles any other trailing whitespace.
       */

      if (n > 0 && buf[n - 1] == '\n') buf[--n] = '\0';
      if (n > 0 && buf[n - 1] == '\r') buf[--n] = '\0';

      int r = process_line(buf, line_no);
      if (r > 0) applied++;
      else if (r < 0) bad++;
    }

  fclose(fp);

  if (applied > 0)
    {
      syslog(LOG_INFO,
             "drivebase: loaded %u config key(s) from %s (%u rejected)\n",
             applied, path, bad);
    }
  else if (bad > 0)
    {
      /* File exists, has content, but every line was bad — strongly
       * suggests a typo or schema misunderstanding, surface it.
       */

      syslog(LOG_WARNING,
             "drivebase: config '%s' yielded 0 applied keys (%u rejected)\n",
             path, bad);
    }

  return 0;
}

const struct db_config_start_defaults_s *db_config_get_start_defaults(void)
{
  return &g_start_defaults;
}
