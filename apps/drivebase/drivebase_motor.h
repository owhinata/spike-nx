/****************************************************************************
 * apps/drivebase/drivebase_motor.h
 *
 * sensor_motor_l / sensor_motor_r abstraction for the drivebase daemon
 * (Issue #77).  Owns the per-side fd, the LEGOSENSOR_CLAIM lock for the
 * daemon's lifetime, and the per-tick encoder drain + actuation API.
 *
 * Wrap detection / continuous angle accumulation is *not* done here —
 * it ships in commit #5 (drivebase_angle.c), which consumes the raw
 * `db_motor_sample_s` produced by drivebase_motor_drain().
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_MOTOR_H
#define __APPS_DRIVEBASE_DRIVEBASE_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "drivebase_internal.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

/* Snapshot of the latest LUMP frame on a motor topic.  Mirrors the
 * fields drivebase needs from `struct lump_sample_s`; the ABI struct is
 * 56 bytes which is overkill once the relevant first int32 is extracted.
 */

struct db_motor_sample_s
{
  uint64_t timestamp_us;       /* lump_sample_s.timestamp                 */
  uint32_t seq;                /* lump_sample_s.seq                       */
  uint32_t generation;         /* lump_sample_s.generation                */
  int32_t  raw_value;          /* data.i32[0] of the active mode          */
  uint8_t  mode_id;
  uint8_t  data_type;
  uint8_t  num_values;
  uint8_t  port_idx;           /* 0..5 = A..F                             */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Open both /dev/uorb/sensor_motor_{l,r}, take LEGOSENSOR_CLAIM, and
 * verify both topics are bound to LPF2 type 48 (SPIKE Medium Motor) on
 * physically-present ports.  Returns 0 on success or a negated errno:
 *
 *   -ENODEV   one or both topics are not bound to a SPIKE Medium Motor
 *   -EBUSY    LEGOSENSOR_CLAIM rejected (another writer holds it)
 *
 * The fds stay open for the entire daemon lifetime; close on teardown
 * auto-RELEASEs the CLAIM and the legoport chardev's close-cleanup
 * coasts the H-bridge, so a process exit is always safe.
 */

int  drivebase_motor_init(void);
void drivebase_motor_deinit(void);

bool drivebase_motor_is_initialised(void);

/* Per-side accessors (return -ENODEV until init succeeds). */

int  drivebase_motor_port_idx(enum db_side_e side);

/* Drain the latest LUMP frames from one side.  Returns 0 on success
 * (out filled with the freshest sample), -EAGAIN if the topic has no
 * fresh sample since the previous drain, or a negated errno on a real
 * read error.  Non-blocking — safe to call from the 5 ms control tick.
 */

int  drivebase_motor_drain(enum db_side_e side,
                           struct db_motor_sample_s *out);

/* Actuation.  duty is the .01-% signed PWM duty (-10000..10000) routed
 * through LEGOSENSOR_SET_PWM; coast / brake go through the per-class
 * LEGOSENSOR_MOTOR_*_{COAST,BRAKE} ioctls landed in this Issue's prep
 * commit.  All three return 0 on success or a negated errno.
 */

int  drivebase_motor_set_duty(enum db_side_e side, int16_t duty);
int  drivebase_motor_coast(enum db_side_e side);
int  drivebase_motor_brake(enum db_side_e side);

/* Switch the LUMP reporting mode on one side.  Used at daemon startup
 * to lock the device into the mode whose first int32 holds the encoder
 * angle the observer expects.  The ABI is `LEGOSENSOR_SELECT` so the
 * actual mode change is asynchronous — the next drain after the device
 * acks will report the new mode_id.
 */

int  drivebase_motor_select_mode(enum db_side_e side, uint8_t mode);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_MOTOR_H */
