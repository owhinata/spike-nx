/****************************************************************************
 * apps/drivebase/drivebase_imu_cal.c
 *
 * Self-contained minimal properties parser for /mnt/flash/imu_cal.txt.
 * See drivebase_imu_cal.h for the data model and failure semantics.
 *
 * Format (line-based, `# ... \n` comments, blank lines ignored):
 *
 *   schema_version = 1
 *   nominal_gyro_radps_per_lsb = 6.109e-4    # informational only
 *   nominal_accel_ms2_per_lsb  = 5.985e-4    # informational only
 *   fsr_gy_dps     = 1000
 *   fsr_xl_g       = 2
 *   odr_hz         = 104
 *   ambient_temp_c = 23
 *
 *   gyro_bias_lsb_x1000  = b0 b1 b2
 *   accel_bias_lsb_x1000 = b0 b1 b2
 *
 *   gyro_M_x1000  = m00 m01 m02 m10 m11 m12 m20 m21 m22    # row-major
 *   accel_M_x1000 = m00 m01 m02 m10 m11 m12 m20 m21 m22
 *
 * Unknown keys are tolerated (forward compat for future schemas that
 * stay within v1 by adding new informational fields).  The reader is
 * deliberately *not* hooked into drivebase_config.c — that helper
 * carries a much larger set of tunables and the cal payload here is
 * a one-shot startup load with stricter validation needs.
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

#include "drivebase_imu.h"
#include "drivebase_imu_cal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LINE_MAX_LEN     256

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static char *trim(char *s)
{
  while (*s && isspace((unsigned char)*s))
    {
      s++;
    }

  if (*s == '\0')
    {
      return s;
    }

  char *end = s + strlen(s) - 1;
  while (end > s && isspace((unsigned char)*end))
    {
      *end-- = '\0';
    }

  return s;
}

/* Parse `count` whitespace-separated decimal integers into `out`.
 * Returns 0 on success, -EINVAL if fewer than `count` valid tokens
 * are present or any token is not a clean decimal integer.
 */

static int parse_int_array(const char *value, int32_t *out, size_t count)
{
  char  buf[LINE_MAX_LEN];
  char *save = NULL;
  size_t n = 0;

  /* strtok_r mutates the string, so copy into a writable scratch.
   * LINE_MAX_LEN is enough for any of our value lines (9 int32s plus
   * separators, comfortably under 200 bytes).
   */

  strncpy(buf, value, sizeof(buf) - 1);
  buf[sizeof(buf) - 1] = '\0';

  for (char *tok = strtok_r(buf, " \t", &save);
       tok != NULL;
       tok = strtok_r(NULL, " \t", &save))
    {
      if (n >= count)
        {
          break;
        }

      char *end;
      long v = strtol(tok, &end, 10);
      if (*end != '\0' || end == tok)
        {
          return -EINVAL;
        }

      out[n++] = (int32_t)v;
    }

  return (n == count) ? 0 : -EINVAL;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_imu_cal_set_identity(struct db_imu_cal_s *cal)
{
  memset(cal, 0, sizeof(*cal));
  for (int i = 0; i < 3; i++)
    {
      cal->gyro_M_x1000[i][i]  = 1000;
      cal->accel_M_x1000[i][i] = 1000;
    }

  cal->schema_version = DB_IMU_CAL_SCHEMA_VERSION;
  cal->loaded         = false;
}

int db_imu_cal_load(struct db_imu_cal_s *cal)
{
  db_imu_cal_set_identity(cal);

  FILE *f = fopen(DB_IMU_CAL_PATH, "r");
  if (f == NULL)
    {
      syslog(LOG_INFO,
             "drivebase: imu_cal: %s absent, running uncalibrated\n",
             DB_IMU_CAL_PATH);
      return -ENOENT;
    }

  char line[LINE_MAX_LEN];
  int  line_no = 0;
  bool got_schema = false;
  int  rc = 0;
  int32_t scratch9[9];

  while (fgets(line, sizeof(line), f) != NULL)
    {
      line_no++;
      char *p = trim(line);
      if (*p == '\0' || *p == '#')
        {
          continue;
        }

      char *eq = strchr(p, '=');
      if (eq == NULL)
        {
          syslog(LOG_WARNING,
                 "drivebase: imu_cal: line %d malformed (no '=')\n",
                 line_no);
          continue;
        }

      *eq = '\0';
      char *key = trim(p);
      char *value = trim(eq + 1);

      if (strcmp(key, "schema_version") == 0)
        {
          char *end;
          long v = strtol(value, &end, 10);
          if (*end != '\0' || end == value)
            {
              rc = -EINVAL;
              break;
            }

          cal->schema_version = (uint8_t)v;
          got_schema = true;
          if (cal->schema_version != DB_IMU_CAL_SCHEMA_VERSION)
            {
              syslog(LOG_WARNING,
                     "drivebase: imu_cal: schema=%u != %u, reject\n",
                     (unsigned)cal->schema_version,
                     (unsigned)DB_IMU_CAL_SCHEMA_VERSION);
              rc = -EPROTO;
              break;
            }
        }
      else if (strcmp(key, "fsr_gy_dps") == 0)
        {
          cal->fsr_gy_dps = (uint16_t)strtol(value, NULL, 10);
        }
      else if (strcmp(key, "fsr_xl_g") == 0)
        {
          cal->fsr_xl_g = (uint8_t)strtol(value, NULL, 10);
        }
      else if (strcmp(key, "odr_hz") == 0)
        {
          cal->odr_hz = (uint16_t)strtol(value, NULL, 10);
        }
      else if (strcmp(key, "ambient_temp_c") == 0)
        {
          cal->ambient_temp_c = (int8_t)strtol(value, NULL, 10);
        }
      else if (strcmp(key, "gyro_bias_lsb_x1000") == 0)
        {
          rc = parse_int_array(value, cal->gyro_bias_lsb_x1000, 3);
          if (rc < 0)
            {
              syslog(LOG_WARNING,
                     "drivebase: imu_cal: line %d bad gyro_bias\n",
                     line_no);
              break;
            }
        }
      else if (strcmp(key, "accel_bias_lsb_x1000") == 0)
        {
          rc = parse_int_array(value, cal->accel_bias_lsb_x1000, 3);
          if (rc < 0)
            {
              syslog(LOG_WARNING,
                     "drivebase: imu_cal: line %d bad accel_bias\n",
                     line_no);
              break;
            }
        }
      else if (strcmp(key, "gyro_M_x1000") == 0)
        {
          rc = parse_int_array(value, scratch9, 9);
          if (rc < 0)
            {
              syslog(LOG_WARNING,
                     "drivebase: imu_cal: line %d bad gyro_M\n",
                     line_no);
              break;
            }

          memcpy(cal->gyro_M_x1000, scratch9, sizeof(scratch9));
        }
      else if (strcmp(key, "accel_M_x1000") == 0)
        {
          rc = parse_int_array(value, scratch9, 9);
          if (rc < 0)
            {
              syslog(LOG_WARNING,
                     "drivebase: imu_cal: line %d bad accel_M\n",
                     line_no);
              break;
            }

          memcpy(cal->accel_M_x1000, scratch9, sizeof(scratch9));
        }
      /* informational keys (nominal_*_per_lsb, etc.): accept silently
       * — the on-device math derives them from fsr_*_dps anyway.
       */
    }

  fclose(f);

  if (rc < 0)
    {
      db_imu_cal_set_identity(cal);
      return rc;
    }

  if (!got_schema)
    {
      syslog(LOG_WARNING,
             "drivebase: imu_cal: schema_version missing, reject\n");
      db_imu_cal_set_identity(cal);
      return -EINVAL;
    }

  /* FSR mismatch.  Dynamic rescale of the cal-time bias / M to the
   * runtime FSR is future work (Phase 2.5b).  For now reject; the
   * apply_fsr_change() path still handles live SET ODR / FSR by
   * rescaling the runtime bias estimate, but starting from a
   * mismatched cal would mean rescaling Matrix entries too, which
   * the current pipeline does not implement.
   */

  if (cal->fsr_gy_dps != DB_IMU_DEFAULT_FSR_GY_DPS)
    {
      syslog(LOG_WARNING,
             "drivebase: imu_cal: gyro FSR mismatch "
             "(cal=%u dps, runtime default=%u dps), reject\n",
             (unsigned)cal->fsr_gy_dps,
             (unsigned)DB_IMU_DEFAULT_FSR_GY_DPS);
      db_imu_cal_set_identity(cal);
      return -EINVAL;
    }

  cal->loaded = true;
  syslog(LOG_INFO,
         "drivebase: imu_cal: loaded %s (FSR=±%u dps, ODR=%u Hz, T=%d°C)\n",
         DB_IMU_CAL_PATH,
         (unsigned)cal->fsr_gy_dps,
         (unsigned)cal->odr_hz,
         (int)cal->ambient_temp_c);
  return 0;
}
