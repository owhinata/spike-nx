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
 * Pre-processor Definitions
 ****************************************************************************/

/* #154 glitch sanity gate.  A fresh encoder sample more than
 * DB_SERVO_GLITCH_MDEG (~5000 deg) from the trusted position is physically
 * impossible in one tick (max wheel motion is ~1 deg/tick).  It is either a
 * transient garbage value the motor emits intermittently in mode 2 (the
 * recurring raw=53038, seen even at idle) or — rarely — a real epoch shift
 * the reset path missed.  Drop it UNLESS the SAME far level repeats for
 * DB_SERVO_REBASE_RUN consecutive samples (agreeing within
 * DB_SERVO_AGREE_MDEG), which a short garbage burst never does; only then is
 * it re-baselined (observer re-seeded, no slope spike).
 */

#define DB_SERVO_GLITCH_MDEG        5000000   /* 5000 deg: impossible jump   */
#define DB_SERVO_AGREE_MDEG         100000    /* 100 deg: same "far level"   */
#define DB_SERVO_REBASE_RUN         10        /* consecutive to re-baseline  */

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
  s->reject_candidate_mdeg = 0;
  s->candidate_run         = 0;

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
  else
    {
      /* No fresh continuous sample: -EAGAIN (incl. a SYNC sentinel),
       * DB_MOTOR_DRAIN_DISCONNECTED (positive), or a read error.  Seed a
       * neutral (x=0, t=now) origin and let the first db_servo_update tick
       * prime the observer once a real encoder sample arrives.  This is
       * also the post-reclaim epoch-reset path (#154), so it must not fail
       * on a still-draining sentinel.
       */

      s->x_actual_mdeg = 0;
      s->t_last_us     = now_us;
      db_observer_reset(&s->observer, 0, now_us);
    }

  s->v_estimate_mdegps     = 0;
  s->last_applied_duty     = 0;
  s->last_actuation        = DRIVEBASE_ON_COMPLETION_COAST;
  s->reject_candidate_mdeg = 0;
  s->candidate_run         = 0;
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
      /* #154 glitch sanity gate.  drivebase_motor_drain already drops
       * non-POS-mode frames; this catches a transient garbage value that
       * arrives IN mode 2 right after a reclaim (e.g. raw=53038 deg between
       * real ~210).  A single >5000 deg jump is physically impossible, so
       * drop it; after a few consecutive drops accept the far value as a
       * real epoch shift (re-seed, no slope spike).
       */

      int64_t x_new   = db_angle_deg_to_mdeg(sm.raw_value);
      int64_t d_track = x_new - s->x_actual_mdeg;
      if (d_track < 0) d_track = -d_track;

      if (d_track <= DB_SERVO_GLITCH_MDEG)
        {
          /* Continuous with the trusted position — accept. */

          s->candidate_run = 0;
          s->x_actual_mdeg = x_new;
          db_observer_update_sample(&s->observer,
                                    s->x_actual_mdeg,
                                    sm.timestamp_us,
                                    duty_abs);
        }
      else
        {
          /* Far from the trusted position.  Count consecutive samples that
           * agree on the SAME far level: a real epoch shift holds steady, a
           * transient garbage burst (raw=53038) does not.  Only a sustained
           * run re-baselines; otherwise drop the sample.
           */

          int64_t d_cand = x_new - s->reject_candidate_mdeg;
          if (d_cand < 0) d_cand = -d_cand;
          if (s->candidate_run > 0 && d_cand <= DB_SERVO_AGREE_MDEG)
            {
              if (s->candidate_run < 255) s->candidate_run++;
            }
          else
            {
              s->candidate_run = 1;
            }
          s->reject_candidate_mdeg = x_new;

          if (s->candidate_run >= DB_SERVO_REBASE_RUN)
            {
              s->candidate_run = 0;
              s->x_actual_mdeg = x_new;
              db_observer_reset(&s->observer, x_new, sm.timestamp_us);
            }
          else
            {
              /* drop the (likely garbage) sample — keep the trusted x. */
            }
        }
    }
  else if (dr == -EAGAIN)
    {
      uint32_t dt_ms = (now_us > s->t_last_us) ?
                       (uint32_t)((now_us - s->t_last_us) / 1000) : 0;
      db_observer_idle_tick(&s->observer, dt_ms, duty_abs);
    }
  else
    {
      /* #154: DB_MOTOR_DRAIN_DISCONNECTED (positive sentinel) or a read
       * error (-EBADF after a reclaim closed the fd, -ENODEV).  The port
       * is lost — arm recovery and skip the observer update (keep the last
       * estimate rather than ingesting a garbage epoch).  Crucially do NOT
       * propagate the code: a <0 here would bubble up through
       * db_drivebase_update into rt_tick_cb and stop the RT thread, and a
       * positive code would be misread by callers.  The apply gate coasts
       * both wheels while the request is pending.
       */

      drivebase_motor_request_reclaim(s->side);
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

          /* #154 honest duty: only record the commanded duty if it was
           * actually applied.  A stale-CLAIM set_duty returns -ENODEV (no
           * torque); recording `d` anyway would feed the stall observer a
           * high applied-duty against zero speed → a FALSE stall=1.  On
           * failure record 0 so the observer sees "not driving".
           * drivebase_motor_set_duty already arms the reclaim request.
           */

          s->last_applied_duty = (rc == 0) ? d : 0;
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
