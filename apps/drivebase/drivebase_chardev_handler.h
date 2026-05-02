/****************************************************************************
 * apps/drivebase/drivebase_chardev_handler.h
 *
 * Daemon-side handler for the /dev/drivebase kernel chardev IPC
 * (Issue #77 commit #9).  Owns the fd that the kernel records as
 * `attach_filep`, drains command envelopes from the kernel's
 * lock-free cmd_ring, dispatches each into db_drivebase_*, and
 * publishes the daemon's state back through the chardev's double-
 * buffer slot.
 *
 * The same handler functions are reused by the Linux/FUSE port —
 * the only thing that changes between NuttX and FUSE is the bridge
 * layer that delivers envelopes to `db_chardev_handler_dispatch_*`.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_CHARDEV_HANDLER_H
#define __APPS_DRIVEBASE_DRIVEBASE_CHARDEV_HANDLER_H

#include <stdbool.h>
#include <stdint.h>

#include <arch/board/board_drivebase.h>

#include "drivebase_drivebase.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_chardev_handler_s
{
  int                       fd;                /* /dev/drivebase fd     */
  bool                      attached;
  uint32_t                  output_epoch_seen; /* highest STOP epoch    */
                                               /* the daemon has acted  */
                                               /* on                    */
  bool                      configured;
  uint32_t                  wheel_d_mm;
  uint32_t                  axle_t_mm;

  struct db_drivebase_s    *db;                /* injected by attach()  */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Open /dev/drivebase, register the daemon with the kernel chardev
 * (DAEMON_ATTACH), and bind the drivebase aggregator the dispatcher
 * runs against.  `motor_l_port_idx` / `motor_r_port_idx` are the
 * legoport indices the kernel needs for the STOP fast path; the
 * daemon got them from drivebase_motor_port_idx() right after
 * drivebase_motor_init().  Returns 0 or a negated errno.
 */

int  db_chardev_handler_attach(struct db_chardev_handler_s *h,
                               struct db_drivebase_s *db,
                               int motor_l_port_idx,
                               int motor_r_port_idx,
                               uint8_t default_on_completion);

/* Reverse of attach.  Issues DAEMON_DETACH and closes the fd.  Safe
 * to call from the chardev handler's own teardown path or from an
 * atexit() hook; the kernel cleanup also fires on close so a daemon
 * crash still leaves the chardev in a clean state for the next
 * attach.
 */

void db_chardev_handler_detach(struct db_chardev_handler_s *h);

/* Per-tick:
 *   1. drain cmd_ring envelopes via DAEMON_PICKUP_CMD until empty
 *   2. for each envelope, dispatch into db_drivebase_*; STOP envelopes
 *      with epoch <= output_epoch_seen are silently dropped (the
 *      kernel already coast/braked the motors in the ioctl context)
 *   3. publish drivebase state via DAEMON_PUBLISH_STATE
 *
 * Returns 0 or a negated errno.  Safe to call from the RT tick
 * thread; only does ioctls (no stdio, malloc, mutex contention).
 */

int  db_chardev_handler_tick(struct db_chardev_handler_s *h,
                             uint64_t now_us);

/* Periodic — call once per ~50 ms (10 ticks) to refresh status
 * counters that don't change every tick (motor_bound flags, gyro
 * presence, etc).  Cheap; just a memcpy + ioctl.
 */

int  db_chardev_handler_publish_status(struct db_chardev_handler_s *h);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_CHARDEV_HANDLER_H */
