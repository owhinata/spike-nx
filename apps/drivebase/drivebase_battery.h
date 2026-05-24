/****************************************************************************
 * apps/drivebase/drivebase_battery.h
 *
 * Battery-sag duty correction (Issue #152 Phase 6 Step 6.3).
 *
 * SPIKE Prime Hub's 6S Li-Ion pack sags from ~8.3 V (full) to ~6.0 V
 * (empty), giving the same commanded duty up to a 28 % mechanical
 * output spread.  Phase 6 corrects for this with a voltage model
 * `duty_corrected = duty * V_NOM_mv / V_bat_mv` applied per-motor at
 * the compose stage (drivebase_drivebase.c), after PID + kV + kS, so
 * the closed loop sees a constant control plant regardless of battery
 * state.
 *
 * Architecture (Plan D3):
 *   - Polling owner: daemon idle thread (drivebase_daemon.c, already
 *     wakes every 50 ms).  Every 4th wake (= ~200 ms) it calls
 *     db_battery_poll(), which issues BATIOC_VOLTAGE on /dev/bat0 and
 *     updates a daemon-local EMA (τ ≈ 1.5 s).  The RT thread NEVER
 *     issues the ioctl — its mutex-protected upper-half would steal
 *     2 ms of tick budget.
 *
 *   - Transfer: a single `_Atomic int32_t` carries the latest EMA mV
 *     from daemon to RT.  Cortex-M4 32-bit aligned loads are HW
 *     atomic, `memory_order_relaxed` is sufficient (the value is one
 *     scalar — no inter-field consistency to worry about).
 *
 *   - Cold start: db_battery_init() seeds the atomic to the configured
 *     `battery_nominal_mv` BEFORE the RT thread launches.  This way
 *     the first few RT ticks (before the first poll fires) see ×1 no-
 *     op correction.
 *
 *   - Low-V cap: callers receiving the snapshot via db_battery_get_mv()
 *     should clamp at `battery_min_mv` (default 6000 mV) before
 *     dividing.  At 6.0 V the correction factor is 7200/6000 = 1.2× —
 *     enough headroom for nominal motion at end-of-pack without the
 *     correction blowing up if the gauge briefly reads sub-6 V.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_BATTERY_H
#define __APPS_DRIVEBASE_DRIVEBASE_BATTERY_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Cold-start seed.  Stores `nominal_mv` into the atomic snapshot so the
 * first RT tick (before the daemon's first poll fires) reads a sensible
 * mV and ends up at ×1 correction.  Must be called AFTER db_settings_
 * freeze() but BEFORE db_rt_start().
 *
 * Idempotent — safe to call from db_daemon_run on every cycle restart.
 */

void db_battery_init(int32_t nominal_mv);

/* Open /dev/bat0 for subsequent polling.  Returns 0 on success, -errno
 * on failure (battery driver missing, etc.).  Failure is non-fatal:
 * db_battery_poll() becomes a no-op and the atomic stays at its seed
 * value, so correction reduces to ×1.  Call after db_battery_init().
 */

int  db_battery_open(void);

/* Close the polling fd.  Safe to call when not open.  Used by the
 * daemon teardown path.
 */

void db_battery_close(void);

/* Issue BATIOC_VOLTAGE on the cached fd, update the EMA, store the new
 * value into the atomic snapshot.  Called by the daemon idle thread at
 * ~200 ms cadence (1 out of every 4 idle wakes).  Daemon-local EMA
 * lives in the .c — this function is the only writer.
 *
 * Returns 0 on success, -errno on ioctl failure (atomic untouched).
 * Failure logs a single LOG_WARNING per consecutive run so a missing /
 * misbehaving driver does not flood dmesg.
 */

int  db_battery_poll(void);

/* RT-side accessor — reads the atomic snapshot via memory_order_relaxed.
 * Returns the EMA mV at the last successful poll, or the cold-start
 * seed if no poll has succeeded yet.  Never returns 0 (atomic starts
 * at nominal_mv from db_battery_init).
 */

int32_t db_battery_get_mv(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_BATTERY_H */
