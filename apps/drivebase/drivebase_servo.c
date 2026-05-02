/****************************************************************************
 * apps/drivebase/drivebase_servo.c
 *
 * Per-motor closed loop.  Stitches encoder drain, observer, trajectory,
 * PID, and motor actuation into one update step.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include "drivebase_servo.h"
#include "drivebase_motor.h"
#include "drivebase_settings.h"
#include "drivebase_angle.h"

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }
static int64_t abs64(int64_t v) { return v < 0 ? -v : v; }

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_servo_init(struct db_servo_s *s, enum db_side_e side)
{
  s->side                  = side;
  s->trajectory_active     = false;
  s->prev_endpoint_valid   = false;
  s->prev_endpoint_mdeg    = 0;
  s->x_actual_mdeg         = 0;
  s->t_last_us             = 0;
  s->on_completion         = DRIVEBASE_ON_COMPLETION_COAST;
  s->last_applied_duty     = 0;
  s->last_actuation        = DRIVEBASE_ON_COMPLETION_COAST;
  s->gains                 = db_settings_servo_gains();
  s->completion            = db_settings_completion();

  const struct db_stall_settings_s *st = db_settings_stall();
  db_observer_init(&s->observer,
                   (uint32_t)st->stall_speed_mdegps,
                   (uint32_t)st->stall_duty_min,
                   st->stall_window_ms,
                   30 /* slope window in ms */);

  db_pid_init(&s->pid);
}

int db_servo_reset(struct db_servo_s *s, uint64_t now_us)
{
  if (!drivebase_motor_is_initialised())
    {
      return -ENODEV;
    }

  /* Try a single non-blocking drain to seed the observer.  If the
   * topic has no fresh sample yet (freshly opened fd, no publish
   * since CLAIM), reset to (x=0, t=now) and let the first
   * db_servo_update tick prime the observer when an encoder sample
   * arrives.  This keeps reset() callable from the RT tick context
   * without pulling in a multi-ms busy-wait.
   */

  struct db_motor_sample_s sm;
  int rc = drivebase_motor_drain(s->side, &sm);
  if (rc == 0)
    {
      s->x_actual_mdeg = db_angle_deg_to_mdeg(sm.raw_value);
      s->t_last_us     = sm.timestamp_us > now_us ?
                         now_us : sm.timestamp_us;
      db_observer_reset(&s->observer, s->x_actual_mdeg, s->t_last_us);
    }
  else if (rc == -EAGAIN)
    {
      s->x_actual_mdeg = 0;
      s->t_last_us     = now_us;
      db_observer_reset(&s->observer, 0, now_us);
    }
  else
    {
      return rc;
    }

  db_pid_reset(&s->pid);
  s->trajectory_active   = false;
  s->prev_endpoint_valid = false;
  s->last_applied_duty   = 0;
  s->last_actuation      = DRIVEBASE_ON_COMPLETION_COAST;
  return 0;
}

int db_servo_position_relative(struct db_servo_s *s,
                               uint64_t now_us,
                               int64_t delta_mdeg,
                               int32_t v_max_mdegps,
                               int32_t accel_mdegps2,
                               int32_t decel_mdegps2,
                               uint8_t on_completion)
{
  /* Decide the trajectory origin: continue-from-endpoint applies when
   * the previous endpoint is still within `pos_tolerance × 2` of the
   * current encoder reading.  This is the SMART continuation behaviour
   * the plan describes — applies after any completion that was hit
   * cleanly, regardless of on_completion (matching pybricks).
   */

  int64_t origin = s->x_actual_mdeg;
  if (s->prev_endpoint_valid)
    {
      int64_t err = abs64(s->prev_endpoint_mdeg - s->x_actual_mdeg);
      if (err <= 2 * (int64_t)s->completion->pos_tolerance_mdeg)
        {
          origin = s->prev_endpoint_mdeg;
        }
    }

  int64_t target = origin + delta_mdeg;

  db_trajectory_init_position(&s->trajectory, now_us,
                              origin, target,
                              v_max_mdegps,
                              accel_mdegps2, decel_mdegps2);

  s->trajectory_active   = true;
  s->prev_endpoint_valid = true;
  s->prev_endpoint_mdeg  = target;
  s->on_completion       = on_completion;
  db_pid_pause(&s->pid, false);
  db_pid_reset(&s->pid);
  return 0;
}

int db_servo_forever(struct db_servo_s *s,
                     uint64_t now_us,
                     int32_t v_target_mdegps,
                     int32_t accel_mdegps2)
{
  db_trajectory_init_forever(&s->trajectory, now_us,
                             s->x_actual_mdeg,
                             v_target_mdegps, accel_mdegps2);
  s->trajectory_active   = true;
  s->prev_endpoint_valid = false;
  s->on_completion       = DRIVEBASE_ON_COMPLETION_COAST;
  db_pid_pause(&s->pid, false);
  db_pid_reset(&s->pid);
  return 0;
}

int db_servo_stop(struct db_servo_s *s, uint64_t now_us,
                  uint8_t on_completion)
{
  s->trajectory_active   = false;
  s->prev_endpoint_valid = false;
  s->on_completion       = on_completion;

  /* Apply the actuation immediately — the daemon's user surface
   * (commit #11) will issue STOP through the kernel chardev, which
   * already takes care of the kernel-side fast path.  When called
   * standalone (CLI), this is the only place the motor stops.
   */

  switch (on_completion)
    {
      case DRIVEBASE_ON_COMPLETION_COAST:
      case DRIVEBASE_ON_COMPLETION_COAST_SMART:
        s->last_applied_duty = 0;
        s->last_actuation    = DRIVEBASE_ON_COMPLETION_COAST;
        return drivebase_motor_coast(s->side);

      case DRIVEBASE_ON_COMPLETION_BRAKE:
      case DRIVEBASE_ON_COMPLETION_BRAKE_SMART:
        s->last_applied_duty = 0;
        s->last_actuation    = DRIVEBASE_ON_COMPLETION_BRAKE;
        return drivebase_motor_brake(s->side);

      case DRIVEBASE_ON_COMPLETION_HOLD:
      default:
        /* HOLD-after-STOP keeps the controller alive at the current
         * position — re-arm trajectory as a zero-distance move so the
         * PID has a fixed reference.
         */

        db_trajectory_init_position(&s->trajectory, now_us,
                                    s->x_actual_mdeg,
                                    s->x_actual_mdeg,
                                    1, 1, 1);
        s->trajectory_active = true;
        s->on_completion     = DRIVEBASE_ON_COMPLETION_HOLD;
        return 0;
    }
}

int db_servo_update(struct db_servo_s *s, uint64_t now_us)
{
  if (!drivebase_motor_is_initialised())
    {
      return -ENODEV;
    }

  /* 1. Drain encoder.  -EAGAIN means no fresh sample this tick — keep
   *    the previously-cached x_actual and feed the observer through
   *    the idle path so stall-streak still ticks.
   */

  struct db_motor_sample_s sm;
  int dr = drivebase_motor_drain(s->side, &sm);
  uint32_t duty_abs = (uint32_t)abs32(s->last_applied_duty);

  if (dr == 0)
    {
      s->x_actual_mdeg = db_angle_deg_to_mdeg(sm.raw_value);
      db_observer_update_sample(&s->observer,
                                s->x_actual_mdeg,
                                sm.timestamp_us,
                                duty_abs);
    }
  else if (dr == -EAGAIN)
    {
      uint32_t dt_ms = (now_us > s->t_last_us) ?
                       (uint32_t)((now_us - s->t_last_us) / 1000) : 0;
      db_observer_idle_tick(&s->observer, dt_ms, duty_abs);
    }
  else
    {
      return dr;
    }

  s->t_last_us = now_us;

  /* 2. Reference from trajectory (if any). */

  struct db_trajectory_ref_s ref = { 0 };
  if (s->trajectory_active)
    {
      db_trajectory_get_reference(&s->trajectory, now_us, &ref);
    }
  else
    {
      ref.x_mdeg    = s->x_actual_mdeg;
      ref.v_mdegps  = 0;
      ref.a_mdegps2 = 0;
      ref.done      = true;
    }

  /* 3. PID update. */

  uint32_t dt_ms_pid = (now_us > s->t_last_us) ? 5 :
                       (uint32_t)((now_us - s->t_last_us) / 1000);
  if (dt_ms_pid == 0) dt_ms_pid = 5;  /* assume nominal tick on first call */

  struct db_pid_input_s in =
  {
    .ref_x_mdeg      = ref.x_mdeg,
    .ref_v_mdegps    = ref.v_mdegps,
    .act_x_mdeg      = s->x_actual_mdeg,
    .act_v_mdegps    = db_observer_v(&s->observer),
    .dt_ms           = dt_ms_pid,
    .gains           = s->gains,
    .completion      = s->completion,
    .trajectory_done = ref.done,
    .on_completion   = s->on_completion,
  };
  struct db_pid_output_s out;
  db_pid_update(&s->pid, &in, &out);

  /* 4. Apply actuation. */

  int rc = 0;
  switch (out.actuation)
    {
      case DRIVEBASE_ON_COMPLETION_COAST:
        rc = drivebase_motor_coast(s->side);
        s->last_applied_duty = 0;
        break;

      case DRIVEBASE_ON_COMPLETION_BRAKE:
        rc = drivebase_motor_brake(s->side);
        s->last_applied_duty = 0;
        break;

      case DRIVEBASE_ON_COMPLETION_HOLD:
      default:
        rc = drivebase_motor_set_duty(s->side, (int16_t)out.duty);
        s->last_applied_duty = out.duty;
        break;
    }

  s->last_actuation = out.actuation;
  return rc;
}

void db_servo_get_status(const struct db_servo_s *s,
                         struct db_servo_status_s *out)
{
  struct db_trajectory_ref_s ref = { 0 };
  if (s->trajectory_active)
    {
      db_trajectory_get_reference(&s->trajectory, s->t_last_us, &ref);
    }
  else
    {
      ref.x_mdeg   = s->x_actual_mdeg;
      ref.v_mdegps = 0;
      ref.done     = true;
    }
  out->ref_x_mdeg        = ref.x_mdeg;
  out->ref_v_mdegps      = ref.v_mdegps;
  out->act_x_mdeg        = s->x_actual_mdeg;
  out->act_v_mdegps      = db_observer_v(&s->observer);
  out->applied_duty      = s->last_applied_duty;
  out->actuation         = s->last_actuation;
  out->done              = s->pid.done;
  out->stalled           = db_observer_is_stalled(&s->observer);
  out->trajectory_active = s->trajectory_active;
}

bool db_servo_is_done(const struct db_servo_s *s)
{
  return s->pid.done;
}
