/****************************************************************************
 * apps/drivebase/drivebase_chardev_handler.c
 *
 * Daemon-side handler for the kernel /dev/drivebase chardev IPC.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "drivebase_chardev_handler.h"
#include "drivebase_motor.h"

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int dispatch_envelope(struct db_chardev_handler_s *h,
                             const struct drivebase_cmd_envelope_s *env,
                             uint64_t now_us)
{
  /* STOP epochs that are stale (kernel already physically stopped the
   * motors at a later epoch) are dropped — we still update local
   * trajectory bookkeeping below so the daemon doesn't try to drive
   * the motors after the user told it to stop.
   */

  bool epoch_active = (env->epoch >= h->output_epoch_seen);

  switch (env->cmd_kind)
    {
      case DRIVEBASE_CONFIG:
        {
          if (env->epoch < h->output_epoch_seen) return 0;
          const struct drivebase_config_s *c =
              (const struct drivebase_config_s *)env->payload;
          if (h->configured)
            {
              return -EBUSY;
            }
          h->wheel_d_mm = c->wheel_diameter_mm;
          h->axle_t_mm  = c->axle_track_mm;
          int rc = db_drivebase_init(h->db, h->wheel_d_mm, h->axle_t_mm);
          if (rc < 0) return rc;
          rc = db_drivebase_reset(h->db, now_us);
          if (rc == 0) h->configured = true;
          return rc;
        }

      case DRIVEBASE_RESET:
        {
          const struct drivebase_reset_s *r =
              (const struct drivebase_reset_s *)env->payload;
          db_drivebase_set_origin(h->db, r->distance_mm, r->angle_mdeg);
          return 0;
        }

      case DRIVEBASE_DRIVE_STRAIGHT:
        if (!epoch_active) return 0;
        if (!h->configured) return -ENOTCONN;
        {
          const struct drivebase_drive_straight_s *a =
              (const struct drivebase_drive_straight_s *)env->payload;
          return db_drivebase_drive_straight(h->db, now_us,
                                             a->distance_mm,
                                             a->on_completion);
        }

      case DRIVEBASE_TURN:
        if (!epoch_active) return 0;
        if (!h->configured) return -ENOTCONN;
        {
          const struct drivebase_turn_s *a =
              (const struct drivebase_turn_s *)env->payload;
          return db_drivebase_turn(h->db, now_us, a->angle_deg,
                                   a->on_completion);
        }

      case DRIVEBASE_DRIVE_CURVE:
      case DRIVEBASE_DRIVE_ARC_ANGLE:
        if (!epoch_active) return 0;
        if (!h->configured) return -ENOTCONN;
        {
          const struct drivebase_drive_curve_s *a =
              (const struct drivebase_drive_curve_s *)env->payload;
          return db_drivebase_drive_curve(h->db, now_us, a->radius_mm,
                                          a->angle_deg,
                                          a->on_completion);
        }

      case DRIVEBASE_DRIVE_ARC_DISTANCE:
        if (!epoch_active) return 0;
        if (!h->configured) return -ENOTCONN;
        {
          const struct drivebase_drive_arc_s *a =
              (const struct drivebase_drive_arc_s *)env->payload;
          return db_drivebase_drive_arc_distance(h->db, now_us,
                                                 a->radius_mm,
                                                 a->arg /* distance_mm */,
                                                 a->on_completion);
        }

      case DRIVEBASE_DRIVE_FOREVER:
        if (!epoch_active) return 0;
        if (!h->configured) return -ENOTCONN;
        {
          const struct drivebase_drive_forever_s *a =
              (const struct drivebase_drive_forever_s *)env->payload;
          return db_drivebase_drive_forever(h->db, now_us,
                                            a->speed_mmps,
                                            a->turn_rate_dps);
        }

      case DRIVEBASE_STOP:
        {
          /* The kernel already coast/brake/zeroed the H-bridge in the
           * ioctl context.  We just need to sync trajectory state so
           * subsequent ticks don't re-drive the motors.
           */

          const struct drivebase_stop_s *a =
              (const struct drivebase_stop_s *)env->payload;
          h->output_epoch_seen = env->epoch;
          if (h->configured)
            {
              db_drivebase_stop(h->db, now_us, a->on_completion);
            }
          return 0;
        }

      case DRIVEBASE_SET_DRIVE_SETTINGS:
      case DRIVEBASE_SET_USE_GYRO:
      case DRIVEBASE_SPIKE_DRIVE_FOREVER:
      case DRIVEBASE_SPIKE_DRIVE_TIME:
      case DRIVEBASE_SPIKE_DRIVE_ANGLE:
        /* commit #11 wires these — for commit #9 we silently drop
         * envelopes the daemon doesn't yet act on so the kernel can
         * still pop them from the ring (no permanent stuck envelope).
         */
        return 0;

      default:
        return -ENOSYS;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int db_chardev_handler_attach(struct db_chardev_handler_s *h,
                              struct db_drivebase_s *db,
                              int motor_l_port_idx,
                              int motor_r_port_idx,
                              uint8_t default_on_completion)
{
  memset(h, 0, sizeof(*h));
  h->fd = -1;
  h->db = db;

  if (db == NULL ||
      motor_l_port_idx < 0 || motor_l_port_idx > 5 ||
      motor_r_port_idx < 0 || motor_r_port_idx > 5)
    {
      return -EINVAL;
    }

  h->fd = open(DRIVEBASE_DEVPATH, O_RDWR);
  if (h->fd < 0)
    {
      return -errno;
    }

  struct drivebase_attach_s att;
  memset(&att, 0, sizeof(att));
  att.motor_l_port_idx     = (uint8_t)motor_l_port_idx;
  att.motor_r_port_idx     = (uint8_t)motor_r_port_idx;
  att.default_on_completion = default_on_completion;

  int rc = ioctl(h->fd, DRIVEBASE_DAEMON_ATTACH, (unsigned long)&att);
  if (rc < 0)
    {
      int err = -errno;
      close(h->fd);
      h->fd = -1;
      return err;
    }

  h->attached = true;
  return 0;
}

void db_chardev_handler_detach(struct db_chardev_handler_s *h)
{
  if (h->fd < 0) return;
  if (h->attached)
    {
      ioctl(h->fd, DRIVEBASE_DAEMON_DETACH, 0);
      h->attached = false;
    }
  close(h->fd);
  h->fd = -1;
}

int db_chardev_handler_tick(struct db_chardev_handler_s *h,
                            uint64_t now_us)
{
  if (!h->attached)
    {
      return -ENOTCONN;
    }

  /* 1. Drain command envelopes.  PICKUP_CMD returns -EAGAIN when the
   *    ring is empty; loop until that happens.
   */

  for (uint32_t i = 0; i < 16; i++)
    {
      struct drivebase_cmd_envelope_s env;
      int rc = ioctl(h->fd, DRIVEBASE_DAEMON_PICKUP_CMD,
                     (unsigned long)&env);
      if (rc < 0)
        {
          if (errno == EAGAIN) break;
          return -errno;
        }
      (void)dispatch_envelope(h, &env, now_us);
    }

  /* 2. Publish drivebase aggregate state. */

  if (h->configured)
    {
      struct drivebase_state_s st;
      db_drivebase_get_state(h->db, &st);
      st.tick_seq = (uint32_t)(now_us & 0xffffffff);
      int rc = ioctl(h->fd, DRIVEBASE_DAEMON_PUBLISH_STATE,
                     (unsigned long)&st);
      if (rc < 0)
        {
          return -errno;
        }
    }

  return 0;
}

int db_chardev_handler_publish_status(struct db_chardev_handler_s *h)
{
  if (!h->attached) return -ENOTCONN;

  struct drivebase_status_s s;
  memset(&s, 0, sizeof(s));
  s.configured       = h->configured;
  s.motor_l_bound    = drivebase_motor_is_initialised();
  s.motor_r_bound    = drivebase_motor_is_initialised();
  s.imu_present      = 0;          /* commit #10 wires this  */
  s.use_gyro         = 0;
  s.daemon_attached  = 1;
  s.tick_count       = 0;          /* RT task tracks this    */
  s.tick_overrun_count = 0;
  s.tick_max_lag_us  = 0;

  int rc = ioctl(h->fd, DRIVEBASE_DAEMON_PUBLISH_STATUS,
                 (unsigned long)&s);
  return rc < 0 ? -errno : 0;
}
