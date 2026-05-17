/****************************************************************************
 * apps/drivebase/drivebase_config.h
 *
 * Properties-format runtime config loader (Issue #143).  Reads
 * `/mnt/flash/drivebase.cfg` and applies recognised key=value pairs to
 * the drivebase_settings module before db_settings_freeze() is called.
 *
 * Wire format:
 *   - One key=value per line, whitespace-trimmed on both sides of '='
 *   - '#' starts a comment: either the whole line (full-line comment)
 *     or trailing on a value line (`key = 42  # explanation`).  Anything
 *     from the first '#' on the right-hand side of '=' is stripped
 *     before integer parsing.
 *   - Blank lines ignored
 *   - LF and CRLF line endings both accepted
 *   - Optional UTF-8 BOM (0xEF 0xBB 0xBF) at file start is skipped
 *   - Line length limit DB_CONFIG_LINE_MAX bytes; overlong lines logged
 *     and skipped
 *   - Unknown keys logged at LOG_WARNING and skipped (forward compat)
 *   - Out-of-range or unparsable values logged at LOG_WARNING and skipped;
 *     the rest of the file is processed
 *
 * Wheel/axle/tick defaults loaded from the file are reported to the
 * caller via db_config_get_start_defaults() so the daemon can pick them
 * up when the CLI omits the corresponding positional argument.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_CONFIG_H
#define __APPS_DRIVEBASE_DRIVEBASE_CONFIG_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define DB_CONFIG_DEFAULT_PATH   "/mnt/flash/drivebase.cfg"
#define DB_CONFIG_LINE_MAX       256

/* Optional overrides for drivebase_daemon_start() arguments.  Each
 * value is "unset" (0) until the parser sees the corresponding key.
 * Daemon picks these up only when the user did not pass the positional
 * CLI argument; CLI always wins.
 */

struct db_config_start_defaults_s
{
  uint32_t wheel_d_um;
  uint32_t axle_t_um;
  uint32_t tick_us;
};

/* Load and apply a config file.  Returns 0 on success (including the
 * "file missing" case which is treated as a successful no-op), <0 on
 * fatal I/O errors that should be surfaced to the caller.  Per-line
 * parse failures are logged via syslog but do not fail the load.
 *
 * On return, db_config_get_start_defaults() reflects the latest values
 * seen in the file (or all-zero if the file was absent).
 */

int db_config_load(const char *path);

/* Report the start-default overrides loaded from the config file.
 * Fields that the file did not specify are 0; the daemon must treat 0
 * as "fall back to the compiled-in default".
 */

const struct db_config_start_defaults_s *db_config_get_start_defaults(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_CONFIG_H */
