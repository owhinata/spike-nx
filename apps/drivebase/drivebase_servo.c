/****************************************************************************
 * apps/drivebase/drivebase_servo.c
 *
 * Per-motor encoder I/O + observer + duty application.  Phase 2 (#141)
 * removed the trajectory + PID + completion latch from this module; all
 * closed-loop control now lives in drivebase_aggregate.c, and this file
 * only handles the per-motor concerns (encoder drain, slope observer,
 * applying a compose result to the H-bridge).
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>

#include <arch/board/board_drivebase.h>  /* enum drivebase_on_completion_e */

#include "drivebase_servo.h"
#include "drivebase_motor.h"
#include "drivebase_settings.h"
#include "drivebase_angle.h"
#include "drivebase_rt.h"      /* DB_RT_TICK_MS_DEFAULT                  */

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int32_t clamp_duty(int32_t v)
{
  if (v >  10000) return  10000;
  if (v < -10000) return -10000;
  return v;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_servo_init(struct db_servo_s *s, enum db_side_e side,
                   uint32_t tick_ms)
{
  s->side                  = side;
  s->tick_ms               = (tick_ms == 0) ? DB_RT_TICK_MS_DEFAULT
                                            : tick_ms;
  s->x_actual_mdeg         = 0;
  s->v_estimate_mdegps     = 0;
  s->t_last_us             = 0;
  s->last_applied_duty     = 0;
  s->last_actuation        = DRIVEBASE_ON_COMPLETION_COAST;

  const struct db_stall_settings_s *st = db_settings_stall();
  db_observer_init(&s->observer,
                   (uint32_t)st->stall_speed_mdegps,
                   (uint32_t)st->stall_duty_min,
                   st->stall_window_ms,
                   30 /* slope window in ms */);
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
   * arrives.
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

  s->v_estimate_mdegps = 0;
  s->last_applied_duty = 0;
  s->last_actuation    = DRIVEBASE_ON_COMPLETION_COAST;
  return 0;
}

int db_servo_update(struct db_servo_s *s, uint64_t now_us)
{
  if (!drivebase_motor_is_initialised())
    {
      return -ENODEV;
    }

  /* Drain any pending encoder samples + update the slope observer.
   * Unlike pre-#141 we do NOT call any PID / trajectory code — that
   * happens in drivebase_aggregate from the aggregator (which uses
   * the state read here).
   */

  struct db_motor_sample_s sm;
  int dr = drivebase_motor_drain(s->side, &sm);
  uint32_t duty_abs = (uint32_t)(s->last_applied_duty < 0 ?
                                 -s->last_applied_duty :
                                  s->last_applied_duty);

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

  s->t_last_us         = now_us;
  s->v_estimate_mdegps = db_observer_v(&s->observer);
  return 0;
}

int db_servo_apply(struct db_servo_s *s, int32_t duty, uint8_t actuation)
{
  if (!drivebase_motor_is_initialised())
    {
      return -ENODEV;
    }

  int rc;
  switch (actuation)
    {
      case DRIVEBASE_ON_COMPLETION_COAST:
      case DRIVEBASE_ON_COMPLETION_COAST_SMART:
        rc = drivebase_motor_coast(s->side);
        s->last_applied_duty = 0;
        s->last_actuation    = DRIVEBASE_ON_COMPLETION_COAST;
        return rc;

      case DRIVEBASE_ON_COMPLETION_BRAKE:
      case DRIVEBASE_ON_COMPLETION_BRAKE_SMART:
        rc = drivebase_motor_brake(s->side);
        s->last_applied_duty = 0;
        s->last_actuation    = DRIVEBASE_ON_COMPLETION_BRAKE;
        return rc;

      case DRIVEBASE_ON_COMPLETION_HOLD:
      case DRIVEBASE_ON_COMPLETION_CONTINUE:
      default:
        {
          int32_t d = clamp_duty(duty);
          rc = drivebase_motor_set_duty(s->side, d);
          s->last_applied_duty = d;
          s->last_actuation    = DRIVEBASE_ON_COMPLETION_HOLD;
          return rc;
        }
    }
}

void db_servo_get_status(const struct db_servo_s *s,
                         struct db_servo_status_s *out)
{
  out->act_x_mdeg   = s->x_actual_mdeg;
  out->act_v_mdegps = s->v_estimate_mdegps;
  out->applied_duty = s->last_applied_duty;
  out->actuation    = s->last_actuation;
  out->stalled      = db_observer_is_stalled(&s->observer);
}
