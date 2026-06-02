/****************************************************************************
 * apps/linetrace/linetrace_config.h
 *
 * Properties-format runtime config loader (Issue #181).  Reads
 * `/mnt/flash/linetrace.cfg` and reports recognised key=value pairs to
 * the caller (linetrace_main.c do_start) so the PID/sensor tuning can be
 * overlaid onto g_params before the daemon task is spawned.  Modelled on
 * drivebase_config (Issue #143); linetrace has no settings module, so the
 * loader fills a plain defaults struct rather than calling setters.
 *
 * Wire format (identical to drivebase.cfg):
 *   - One key=value per line, whitespace-trimmed on both sides of '='
 *   - '#' starts a comment: either the whole line (full-line comment) or
 *     trailing on a value line (`key = 42  # explanation`).  Anything from
 *     the first '#' on the right-hand side of '=' is stripped before
 *     integer parsing.
 *   - Blank lines ignored
 *   - LF and CRLF line endings both accepted
 *   - Optional UTF-8 BOM (0xEF 0xBB 0xBF) at file start is skipped
 *   - Line length limit LT_CONFIG_LINE_MAX bytes; overlong lines logged
 *     and skipped
 *   - Unknown keys logged at LOG_WARNING and skipped (forward compat)
 *   - Out-of-range or unparsable values logged at LOG_WARNING and skipped;
 *     the rest of the file is processed
 *
 * Precedence (resolved in do_start / do_run): CLI > config > compiled.
 ****************************************************************************/

#ifndef __APPS_LINETRACE_LINETRACE_CONFIG_H
#define __APPS_LINETRACE_LINETRACE_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LT_CONFIG_DEFAULT_PATH   "/mnt/flash/linetrace.cfg"
#define LT_CONFIG_LINE_MAX       256

/* Bits in lt_config_defaults_s.present.  A bit is set ONLY when the key
 * was parsed, range-validated, and applied successfully.  An explicit
 * mask is required because 0 is a legitimate value for most keys
 * (kp=0 / ki=0 / target=0 etc.), so the drivebase "0 == unset" trick
 * does not work here.  The consumer treats an absent bit as "fall back
 * to the compiled-in default".
 */

enum lt_config_present_e
{
  LT_CFG_KP     = 1u << 0,
  LT_CFG_KI     = 1u << 1,
  LT_CFG_KD     = 1u << 2,
  LT_CFG_TARGET = 1u << 3,
  LT_CFG_HZ     = 1u << 4,
  LT_CFG_SPEED  = 1u << 5,
  LT_CFG_VMIN   = 1u << 6,
  LT_CFG_VALPHA = 1u << 7,
  LT_CFG_VBETA  = 1u << 8,
  LT_CFG_EDGE   = 1u << 9,
};

/* Persisted linetrace tuning.  Field names and units mirror the
 * linetrace_main.c g_params struct one-to-one; gains are x100 scaled
 * ints (kp_x100 = 36 -> 0.36), edge is 1 = LEFT / 2 = RIGHT.
 */

struct lt_config_defaults_s
{
  int      kp_x100;
  int      ki_x100;
  int      kd_x100;
  int      target;
  int      hz;
  int      speed_mmps;
  int      v_min_mmps;
  int      v_alpha_x100;
  int      v_beta_x100;
  int      edge;

  uint16_t present;   /* OR of lt_config_present_e bits */
};

/* Load and parse a config file.  Returns 0 on success (including the
 * "file missing" case, treated as a successful no-op), <0 on fatal I/O
 * errors.  Per-line parse failures are logged via syslog but do not fail
 * the load.  On return, lt_config_get_defaults() reflects the keys that
 * were applied (or an all-zero / present==0 struct if the file was
 * absent).
 */

int lt_config_load(const char *path);

/* Report the parsed config.  Fields whose present bit is clear hold
 * indeterminate values and must not be used; the caller keys all reads
 * off the present mask.
 */

const struct lt_config_defaults_s *lt_config_get_defaults(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_LINETRACE_LINETRACE_CONFIG_H */
