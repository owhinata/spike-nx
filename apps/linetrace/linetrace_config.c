/****************************************************************************
 * apps/linetrace/linetrace_config.c
 *
 * Properties-format runtime config loader (Issue #181).  See header for
 * the wire-format contract.  Implementation notes:
 *
 *   - The parser is single-pass and stateless across lines; bad lines
 *     are skipped individually rather than rejecting the whole file
 *     (matches drivebase_config / codex-review feedback #143).
 *
 *   - Unlike drivebase, linetrace has no settings module: each setter
 *     range-validates the value, writes it into the file-scope defaults
 *     struct, and sets the corresponding present bit.  The present bit
 *     is set ONLY on a fully successful parse+validate, so an absent or
 *     rejected key leaves the bit clear and the consumer falls back to
 *     the compiled default.
 *
 *   - Logging goes through syslog: file-missing is silent (the common
 *     case on first boot); parse errors and unknown keys are LOG_WARNING;
 *     a load that applied >= 1 key emits a single LOG_INFO summary.
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

#include "linetrace_config.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* Setter signature used by the dispatch table.  Returns 0 on success,
 * negative errno on failure; the caller logs and continues.
 */

typedef int (*lt_config_setter_t)(const char *value);

struct lt_config_entry_s
{
  const char         *key;
  lt_config_setter_t  setter;
};

/****************************************************************************
 * File-scope state
 ****************************************************************************/

static struct lt_config_defaults_s g_defaults;

/****************************************************************************
 * Value parsing helper
 ****************************************************************************/

static int parse_i32(const char *s, int32_t *out)
{
  if (s == NULL || *s == '\0') return -EINVAL;
  errno = 0;
  char *end = NULL;
  long v = strtol(s, &end, 10);
  if (errno != 0) return -EINVAL;
  if (end == s) return -EINVAL;

  /* Reject trailing garbage after the integer (e.g. "123abc"). */

  while (*end != '\0')
    {
      if (!isspace((unsigned char)*end)) return -EINVAL;
      end++;
    }
  if (v < INT32_MIN || v > INT32_MAX) return -EINVAL;
  *out = (int32_t)v;
  return 0;
}

/****************************************************************************
 * Per-key setters
 *
 * Each parses an int, range-validates against the same bounds the
 * `linetrace run` CLI enforces, stores it, and marks the present bit.
 ****************************************************************************/

#define DEF_SETTER(NAME, FIELD, LO, HI, BIT)                            \
  static int set_##NAME(const char *value)                              \
  {                                                                     \
    int32_t v;                                                          \
    int rc = parse_i32(value, &v);                                      \
    if (rc < 0) return rc;                                              \
    if (v < (LO) || v > (HI)) return -EINVAL;                           \
    g_defaults.FIELD    = (int)v;                                       \
    g_defaults.present |= (BIT);                                        \
    return 0;                                                           \
  }

/* Gain bounds match parse_gain()'s [-100.00, 100.00] -> x100. */

DEF_SETTER(kp_x100,      kp_x100,      -10000, 10000,     LT_CFG_KP)
DEF_SETTER(ki_x100,      ki_x100,      -10000, 10000,     LT_CFG_KI)
DEF_SETTER(kd_x100,      kd_x100,      -10000, 10000,     LT_CFG_KD)
DEF_SETTER(target,       target,       0,      1024,      LT_CFG_TARGET)
DEF_SETTER(hz,           hz,           10,     200,       LT_CFG_HZ)
DEF_SETTER(speed_mmps,   speed_mmps,   0,      INT32_MAX, LT_CFG_SPEED)

/* v_min_mmps lower bound is 1: do_run rejects v_min=0 when speed>0, and
 * the present mask removes any need for a 0 sentinel.  The effective
 * value is clamped to the run's speed at engage time (see do_run).
 */

DEF_SETTER(v_min_mmps,   v_min_mmps,   1,      INT32_MAX, LT_CFG_VMIN)
DEF_SETTER(v_alpha_x100, v_alpha_x100, 0,      10000,     LT_CFG_VALPHA)
DEF_SETTER(v_beta_x100,  v_beta_x100,  0,      10000,     LT_CFG_VBETA)

/* edge: 1 = LEFT, 2 = RIGHT (matches g_params / capture schema). */

DEF_SETTER(edge,         edge,         1,      2,         LT_CFG_EDGE)

/****************************************************************************
 * Dispatch table
 ****************************************************************************/

static const struct lt_config_entry_s g_config_table[] =
{
  { "kp_x100",      set_kp_x100 },
  { "ki_x100",      set_ki_x100 },
  { "kd_x100",      set_kd_x100 },
  { "target",       set_target },
  { "hz",           set_hz },
  { "speed_mmps",   set_speed_mmps },
  { "v_min_mmps",   set_v_min_mmps },
  { "v_alpha_x100", set_v_alpha_x100 },
  { "v_beta_x100",  set_v_beta_x100 },
  { "edge",         set_edge },
};

#define LT_CONFIG_TABLE_SIZE \
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

static const struct lt_config_entry_s *find_entry(const char *key)
{
  for (size_t i = 0; i < LT_CONFIG_TABLE_SIZE; i++)
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
  /* Strip the optional UTF-8 BOM on line 1. */

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
             "linetrace: config line %u: missing '='\n", line_no);
      return -1;
    }

  *eq = '\0';
  char *key = p;
  char *val = eq + 1;

  /* Strip an inline `#` comment from the value.  All keys are integers,
   * so a '#' on the value side is always a comment.
   */

  char *hash = strchr(val, '#');
  if (hash != NULL) *hash = '\0';

  rstrip(key);
  val = lstrip(val);
  rstrip(val);

  if (*key == '\0')
    {
      syslog(LOG_WARNING,
             "linetrace: config line %u: empty key\n", line_no);
      return -1;
    }

  const struct lt_config_entry_s *entry = find_entry(key);
  if (entry == NULL)
    {
      syslog(LOG_WARNING,
             "linetrace: config line %u: unknown key '%s'\n",
             line_no, key);
      return -1;
    }

  int rc = entry->setter(val);
  if (rc < 0)
    {
      syslog(LOG_WARNING,
             "linetrace: config line %u: key '%s' value '%s' rejected "
             "(%d)\n", line_no, key, val, rc);
      return -1;
    }
  return 1;
}

/****************************************************************************
 * Public API
 ****************************************************************************/

int lt_config_load(const char *path)
{
  memset(&g_defaults, 0, sizeof(g_defaults));

  if (path == NULL) path = LT_CONFIG_DEFAULT_PATH;

  FILE *fp = fopen(path, "r");
  if (fp == NULL)
    {
      /* ENOENT is the common case (no config installed yet) — silent
       * fallback.  Any other errno is worth a warning so a mount issue
       * or path typo is visible.
       */

      if (errno == ENOENT) return 0;
      syslog(LOG_WARNING,
             "linetrace: cannot open config '%s': %d\n", path, errno);
      return 0;
    }

  unsigned line_no = 0;
  unsigned applied = 0;
  unsigned bad     = 0;
  char buf[LT_CONFIG_LINE_MAX + 2];   /* +1 for '\n', +1 for terminator */

  while (fgets(buf, sizeof(buf), fp) != NULL)
    {
      line_no++;

      /* Detect overlong lines: if the buffer filled and the last char
       * isn't a newline, the physical line exceeds LT_CONFIG_LINE_MAX.
       * Skip the rest of the physical line and warn.
       */

      size_t n = strlen(buf);
      bool overlong = (n == sizeof(buf) - 1 && buf[n - 1] != '\n');
      if (overlong)
        {
          syslog(LOG_WARNING,
                 "linetrace: config line %u: exceeds %d bytes, skipped\n",
                 line_no, LT_CONFIG_LINE_MAX);
          int c;
          while ((c = fgetc(fp)) != EOF && c != '\n') { }
          bad++;
          continue;
        }

      /* Strip CR (CRLF endings) and the trailing LF. */

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
             "linetrace: loaded %u config key(s) from %s (%u rejected)\n",
             applied, path, bad);
    }
  else if (bad > 0)
    {
      syslog(LOG_WARNING,
             "linetrace: config '%s' yielded 0 applied keys (%u rejected)\n",
             path, bad);
    }

  return 0;
}

const struct lt_config_defaults_s *lt_config_get_defaults(void)
{
  return &g_defaults;
}
