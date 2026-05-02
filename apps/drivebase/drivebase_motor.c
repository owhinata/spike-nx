/****************************************************************************
 * apps/drivebase/drivebase_motor.c
 *
 * sensor_motor_l / sensor_motor_r abstraction.  Owns the per-side fd
 * and the LEGOSENSOR_CLAIM lock for the daemon's lifetime; exposes
 * non-blocking encoder drain + actuation primitives for the 5 ms
 * control tick.  See drivebase_motor.h for the API contract.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <arch/board/board_legosensor.h>

#include "drivebase_motor.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DB_LPF2_TYPE_SPIKE_MEDIUM_MOTOR  48

/* sizeof(lump_sample_s) is 56 B (board_legosensor.h _Static_assert).
 * NBUFFER=16 ⇒ 16 ms of 1 kHz samples.  We drain the whole ring in one
 * read() and keep only the freshest entry.
 */

#define DB_DRAIN_BATCH                   16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct db_motor_side_s
{
  int                       fd;
  bool                      claimed;
  uint8_t                   port_idx;
  uint32_t                  last_consumed_seq;
  bool                      have_last_seq;
  const char               *path;
  enum legosensor_class_e   expected_class;
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct db_motor_side_s g_motor[DB_SIDE_NUM] =
{
  [DB_SIDE_LEFT] =
  {
    .fd             = -1,
    .port_idx       = 0xff,
    .path           = "/dev/uorb/sensor_motor_l",
    .expected_class = LEGOSENSOR_CLASS_MOTOR_L,
  },
  [DB_SIDE_RIGHT] =
  {
    .fd             = -1,
    .port_idx       = 0xff,
    .path           = "/dev/uorb/sensor_motor_r",
    .expected_class = LEGOSENSOR_CLASS_MOTOR_R,
  },
};

static bool g_initialised;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void close_one(struct db_motor_side_s *m)
{
  if (m->claimed)
    {
      /* Best-effort RELEASE — close() will auto-RELEASE anyway. */
      ioctl(m->fd, LEGOSENSOR_RELEASE, 0);
      m->claimed = false;
    }
  if (m->fd >= 0)
    {
      close(m->fd);
      m->fd = -1;
    }
  m->port_idx       = 0xff;
  m->have_last_seq  = false;
  m->last_consumed_seq = 0;
}

static int open_one(struct db_motor_side_s *m)
{
  /* O_RDONLY only — the NuttX upper-half sensor framework rejects
   * O_RDWR for SENSOR_TYPE_CUSTOM topics.  All ioctls (CLAIM, SELECT,
   * SET_PWM, MOTOR_*_{COAST,BRAKE}) work fine on a read-only fd.
   */

  m->fd = open(m->path, O_RDONLY | O_NONBLOCK);
  if (m->fd < 0)
    {
      return -errno;
    }

  int ret = ioctl(m->fd, LEGOSENSOR_CLAIM, 0);
  if (ret < 0)
    {
      int err = -errno;
      close_one(m);
      return err;
    }
  m->claimed = true;

  /* Verify the topic is currently bound to a SPIKE Medium Motor.  The
   * LEGOSENSOR class topic only binds when a matching device is present
   * on a port whose parity matches the class — so this query both
   * checks "is something plugged in?" and "is it the right device?".
   */

  struct legosensor_info_arg_s info;
  memset(&info, 0, sizeof(info));
  ret = ioctl(m->fd, LEGOSENSOR_GET_INFO, (unsigned long)&info);
  if (ret < 0)
    {
      int err = -errno;
      close_one(m);
      return err;
    }

  if (info.info.type_id != DB_LPF2_TYPE_SPIKE_MEDIUM_MOTOR)
    {
      close_one(m);
      return -ENODEV;
    }

  m->port_idx = info.port;
  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int drivebase_motor_init(void)
{
  if (g_initialised)
    {
      return -EALREADY;
    }

  int ret = open_one(&g_motor[DB_SIDE_LEFT]);
  if (ret < 0)
    {
      return ret;
    }

  ret = open_one(&g_motor[DB_SIDE_RIGHT]);
  if (ret < 0)
    {
      close_one(&g_motor[DB_SIDE_LEFT]);
      return ret;
    }

  g_initialised = true;
  return 0;
}

void drivebase_motor_deinit(void)
{
  if (!g_initialised)
    {
      return;
    }

  /* Do not force COAST here — that would override an explicit BRAKE
   * the daemon may have just issued (BRAKE-and-deinit is a legitimate
   * stop policy).  The legoport chardev's close-cleanup auto-coasts
   * any port we left in PWM state when the fd is closed, which covers
   * the runaway-PWM safety case without trampling on BRAKE.
   */

  close_one(&g_motor[DB_SIDE_LEFT]);
  close_one(&g_motor[DB_SIDE_RIGHT]);
  g_initialised = false;
}

bool drivebase_motor_is_initialised(void)
{
  return g_initialised;
}

int drivebase_motor_port_idx(enum db_side_e side)
{
  if (!g_initialised || (unsigned)side >= DB_SIDE_NUM)
    {
      return -ENODEV;
    }
  return g_motor[side].port_idx;
}

int drivebase_motor_drain(enum db_side_e side,
                          struct db_motor_sample_s *out)
{
  if (!g_initialised || (unsigned)side >= DB_SIDE_NUM || out == NULL)
    {
      return -EINVAL;
    }

  struct db_motor_side_s *m = &g_motor[side];
  struct lump_sample_s    batch[DB_DRAIN_BATCH];

  ssize_t n = read(m->fd, batch, sizeof(batch));
  if (n < 0)
    {
      if (errno == EAGAIN || errno == ENODATA)
        {
          return -EAGAIN;
        }
      return -errno;
    }

  size_t count = (size_t)n / sizeof(batch[0]);
  if (count == 0)
    {
      return -EAGAIN;
    }

  /* Newest sample is the last entry — single most-recent latch. */

  const struct lump_sample_s *latest = &batch[count - 1];

  out->timestamp_us = latest->timestamp;
  out->seq          = latest->seq;
  out->generation   = latest->generation;
  out->mode_id      = latest->mode_id;
  out->data_type    = latest->data_type;
  out->num_values   = latest->num_values;
  out->port_idx     = latest->port;

  /* For commit #4 the consumer treats the first int32 of the active
   * mode's payload as the encoder reading.  Wider modes (multi-int32 or
   * float) and mode-specific scaling land in commit #5 (drivebase_angle).
   * If the mode reports something narrower, sign-extend to int32 so the
   * upper layer never sees a stale upper byte from a previous read.
   */

  switch (latest->data_type)
    {
      case 0:  /* INT8  */
        out->raw_value = (int32_t)latest->data.i8[0];
        break;
      case 1:  /* INT16 */
        out->raw_value = (int32_t)latest->data.i16[0];
        break;
      case 2:  /* INT32 */
        out->raw_value = latest->data.i32[0];
        break;
      case 3:  /* FLOAT */
        out->raw_value = (int32_t)latest->data.f32[0];
        break;
      default:
        out->raw_value = 0;
        break;
    }

  m->last_consumed_seq = latest->seq;
  m->have_last_seq     = true;
  return 0;
}

int drivebase_motor_set_duty(enum db_side_e side, int16_t duty)
{
  if (!g_initialised || (unsigned)side >= DB_SIDE_NUM)
    {
      return -ENODEV;
    }

  struct legosensor_pwm_arg_s arg;
  memset(&arg, 0, sizeof(arg));
  arg.num_channels = 1;
  arg.channels[0]  = duty;

  int ret = ioctl(g_motor[side].fd, LEGOSENSOR_SET_PWM, (unsigned long)&arg);
  return ret < 0 ? -errno : 0;
}

int drivebase_motor_coast(enum db_side_e side)
{
  if (!g_initialised || (unsigned)side >= DB_SIDE_NUM)
    {
      return -ENODEV;
    }

  int cmd = (side == DB_SIDE_LEFT) ? LEGOSENSOR_MOTOR_L_COAST
                                   : LEGOSENSOR_MOTOR_R_COAST;
  int ret = ioctl(g_motor[side].fd, cmd, 0);
  return ret < 0 ? -errno : 0;
}

int drivebase_motor_brake(enum db_side_e side)
{
  if (!g_initialised || (unsigned)side >= DB_SIDE_NUM)
    {
      return -ENODEV;
    }

  int cmd = (side == DB_SIDE_LEFT) ? LEGOSENSOR_MOTOR_L_BRAKE
                                   : LEGOSENSOR_MOTOR_R_BRAKE;
  int ret = ioctl(g_motor[side].fd, cmd, 0);
  return ret < 0 ? -errno : 0;
}

int drivebase_motor_select_mode(enum db_side_e side, uint8_t mode)
{
  if (!g_initialised || (unsigned)side >= DB_SIDE_NUM)
    {
      return -ENODEV;
    }

  struct legosensor_select_arg_s arg = { .mode = mode };
  int ret = ioctl(g_motor[side].fd, LEGOSENSOR_SELECT, (unsigned long)&arg);
  return ret < 0 ? -errno : 0;
}
