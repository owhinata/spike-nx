/****************************************************************************
 * apps/drivebase/drivebase_daemon.h
 *
 * Lifecycle FSM for the SPIKE Prime drivebase daemon (Issue #77
 * commit #11).  Owns the long-lived daemon task that spawns the RT
 * loop, attaches to /dev/drivebase, and tears down cleanly on
 * `drivebase stop` or on a stall watchdog trigger.  The user-facing
 * CLI verbs (drive_straight / turn / forever / stop_motion /
 * get_state / set_use_gyro / jitter) travel through the kernel
 * chardev's user-facing ioctls — never directly into this module.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_DAEMON_H
#define __APPS_DRIVEBASE_DRIVEBASE_DAEMON_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

enum db_daemon_state_e
{
  DB_DAEMON_STOPPED      = 0,
  DB_DAEMON_INITIALISING = 1,
  DB_DAEMON_RUNNING      = 2,
  DB_DAEMON_TEARDOWN     = 3,
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Spawn the daemon task with a configured wheel diameter / axle
 * track.  Returns the new daemon's pid (> 0) on success or a
 * negated errno.  Idempotent — second call while running returns
 * -EALREADY.
 */

int  drivebase_daemon_start(uint32_t wheel_d_mm, uint32_t axle_t_mm);

/* Request graceful teardown.  Sets running=false and waits up to
 * `timeout_ms` for the daemon task to exit.  Returns 0 on clean
 * stop, -ETIMEDOUT if the daemon is still alive after the wait, or
 * -EAGAIN if no daemon was running.
 */

int  drivebase_daemon_stop(uint32_t timeout_ms);

enum db_daemon_state_e drivebase_daemon_state(void);
int  drivebase_daemon_get_pid(void);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_DAEMON_H */
