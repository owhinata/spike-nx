/****************************************************************************
 * apps/drivebase/drivebase_battery.c
 *
 * Battery-sag duty correction polling + atomic snapshot
 * (Issue #152 Phase 6 Step 6.3).  See drivebase_battery.h for the
 * architecture overview.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <nuttx/power/battery_ioctl.h>

#include "drivebase_battery.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_BATTERY_DEVPATH    "/dev/bat0"

/* EMA shift (Plan D3): prev = (prev * 7 + now) / 8 → τ ≈ 8 × poll
 * period = 8 × 200 ms = 1.6 s.  At 7200 mV, the per-step truncation
 * loss is <1 mV (0.014 %), well below the duty quantization granularity
 * the correction can express (.01 % per mdeg-per-mV at nominal).
 */

#define DB_BATTERY_EMA_ALPHA_NUM   7
#define DB_BATTERY_EMA_ALPHA_DEN   8

/****************************************************************************
 * File-scope State
 *
 * Single-instance daemon — no struct wrapper needed.  The atomic is the
 * sole inter-thread surface; everything else is touched by the daemon
 * idle thread only.
 ****************************************************************************/

static _Atomic int32_t g_vbat_mv      = 7200;  /* overwritten by init */
static int             g_bat_fd       = -1;
static int32_t         g_prev_ema_mv  = 0;     /* daemon-local EMA   */
static bool            g_warn_emitted = false; /* throttle bad-ioctl  */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_battery_init(int32_t nominal_mv)
{
  if (nominal_mv <= 0)
    {
      nominal_mv = 7200;
    }

  /* Seed the atomic so the first RT tick (which fires before the first
   * poll completes) sees a sensible value → correction reduces to ×1.
   * memory_order_relaxed: this store happens once before the RT thread
   * is launched, so there is no concurrent reader yet.
   */

  atomic_store_explicit(&g_vbat_mv, nominal_mv, memory_order_relaxed);

  /* Seed the EMA so the first poll's value blends with nominal rather
   * than 0 (which would briefly drop the published mV to ~nominal/8 and
   * trigger the low-V cap on the RT side).
   */

  g_prev_ema_mv  = nominal_mv;
  g_warn_emitted = false;
}

int db_battery_open(void)
{
  if (g_bat_fd >= 0)
    {
      return 0;
    }

  g_bat_fd = open(DB_BATTERY_DEVPATH, O_RDONLY);
  if (g_bat_fd < 0)
    {
      int err = errno;
      syslog(LOG_WARNING,
             "drivebase: battery open %s failed errno %d\n",
             DB_BATTERY_DEVPATH, err);
      return -err;
    }
  return 0;
}

void db_battery_close(void)
{
  if (g_bat_fd >= 0)
    {
      close(g_bat_fd);
      g_bat_fd = -1;
    }
}

int db_battery_poll(void)
{
  if (g_bat_fd < 0)
    {
      return -EBADF;
    }

  int sample_mv = 0;
  if (ioctl(g_bat_fd, BATIOC_VOLTAGE, (unsigned long)&sample_mv) < 0)
    {
      int err = errno;
      if (!g_warn_emitted)
        {
          syslog(LOG_WARNING,
                 "drivebase: battery ioctl errno %d (warning suppressed)\n",
                 err);
          g_warn_emitted = true;
        }
      return -err;
    }
  g_warn_emitted = false;

  /* Reject implausible samples — the duty correction divides by vbat
   * later, and a 0 or negative reading would either crash (DIV/0) or
   * invert the correction.  The RT side also clamps to battery_min_mv,
   * but skipping the EMA update here keeps a single transient glitch
   * from poisoning the published average for ~1.6 s.
   */

  if (sample_mv <= 0)
    {
      return -EINVAL;
    }

  /* EMA: prev = (prev * 7 + sample) / 8.  All int32; max intermediate
   * is `prev * 7 + sample` < ~50000 * 7 = 350000, well inside int32.
   */

  g_prev_ema_mv = (g_prev_ema_mv * DB_BATTERY_EMA_ALPHA_NUM + sample_mv) /
                  DB_BATTERY_EMA_ALPHA_DEN;

  atomic_store_explicit(&g_vbat_mv, g_prev_ema_mv, memory_order_relaxed);
  return 0;
}

int32_t db_battery_get_mv(void)
{
  return atomic_load_explicit(&g_vbat_mv, memory_order_relaxed);
}
