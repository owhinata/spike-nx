/****************************************************************************
 * apps/drivebase/drivebase_aggregate.c
 *
 * Aggregate distance/heading PID controller (Issue #141 Phase 2 A).
 * See drivebase_aggregate.h for the architecture overview.
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>
#include <string.h>

#include <arch/board/board_drivebase.h>  /* enum drivebase_on_completion_e */

#include "drivebase_aggregate.h"
#include "drivebase_settings.h"

/****************************************************************************
 * Private Helpers
 ****************************************************************************/

static int64_t abs64(int64_t v)
{
  return v < 0 ? -v : v;
}

/* Refresh cached gain / completion pointers — db_settings_completion_axis
 * recomputes the derived tolerance on every call, so re-fetching keeps
 * the controller in sync with the latest kp_pos.
 */

static void aggregate_refresh_settings(struct db_aggregate_control_s *ctl)
{
  ctl->gains      = db_settings_pid_gains(ctl->axis);
  ctl->completion = db_settings_completion_axis(ctl->axis);
  ctl->ff_axis    = db_settings_ff_axis_gains(ctl->axis);
}

/* Reference-time pause helpers (Issue #142, Phase 5 D).  Ports pybricks
 * `pbio_position_integrator_{get_ref_time,pause,resume,reset}` (lib/pbio/
 * src/integrator.c L142-218) to spike-nx's µs-based clock.  Trajectory
 * functions are called with the effective ref-time returned by
 * aggregate_get_ref_time(), so the trajectory module itself is unaware
 * of pauses.
 */

static uint64_t aggregate_get_ref_time(const struct db_aggregate_control_s *ctl,
                                       uint64_t now_us)
{
  /* While paused, the effective ref-time is frozen at pause_begin minus
   * the previously-accumulated total.  On resume, total_us absorbs the
   * pause duration, so (now - total) is continuous across the resume.
   */

  if (ctl->traj_paused)
    {
      return ctl->traj_pause_begin_us - ctl->traj_pause_total_us;
    }
  return now_us - ctl->traj_pause_total_us;
}

static void aggregate_set_pause(struct db_aggregate_control_s *ctl,
                                bool pause, uint64_t now_us)
{
  if (pause && !ctl->traj_paused)
    {
      ctl->traj_pause_begin_us = now_us;
      ctl->traj_paused         = true;
    }
  else if (!pause && ctl->traj_paused)
    {
      ctl->traj_pause_total_us += (now_us - ctl->traj_pause_begin_us);
      ctl->traj_paused          = false;
    }
}

static void aggregate_reset_pause(struct db_aggregate_control_s *ctl)
{
  ctl->traj_paused         = false;
  ctl->traj_pause_begin_us = 0;
  ctl->traj_pause_total_us = 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void db_aggregate_control_init(struct db_aggregate_control_s *ctl,
                               enum db_axis_e axis,
                               uint32_t tick_ms)
{
  memset(ctl, 0, sizeof(*ctl));
  ctl->axis           = axis;
  ctl->tick_ms        = tick_ms ? tick_ms : 2;
  ctl->on_completion  = DRIVEBASE_ON_COMPLETION_COAST;
  aggregate_refresh_settings(ctl);
  db_pid_init(&ctl->pid);
}

void db_aggregate_control_reset(struct db_aggregate_control_s *ctl)
{
  aggregate_refresh_settings(ctl);
  ctl->trajectory_active   = false;
  ctl->prev_endpoint_valid = false;
  ctl->prev_endpoint_mdeg  = 0;
  ctl->on_completion       = DRIVEBASE_ON_COMPLETION_COAST;
  aggregate_reset_pause(ctl);
  db_pid_reset(&ctl->pid);
}

int db_aggregate_control_drive_position(struct db_aggregate_control_s *ctl,
                                        uint64_t now_us,
                                        int64_t origin_mdeg,
                                        int64_t delta_mdeg,
                                        int32_t v_max_mdegps,
                                        int32_t accel_mdegps2,
                                        int32_t decel_mdegps2,
                                        uint8_t on_completion)
{
  aggregate_refresh_settings(ctl);

  /* SMART continue-from-endpoint: if the previous finite trajectory's
   * endpoint is still within `smart_continue_window_mdeg` of the
   * requested origin, start the new move from the recorded endpoint
   * (pybricks "continue from endpoint" behaviour, #140 decoupled the
   * window from pos_tolerance so retuning tolerance does not widen
   * this window).
   */

  int64_t actual_origin = origin_mdeg;
  if (ctl->prev_endpoint_valid)
    {
      int64_t err = abs64(ctl->prev_endpoint_mdeg - origin_mdeg);
      if (err <= (int64_t)ctl->completion->smart_continue_window_mdeg)
        {
          actual_origin = ctl->prev_endpoint_mdeg;
        }
    }

  int64_t target = actual_origin + delta_mdeg;
  aggregate_reset_pause(ctl);
  db_trajectory_init_position(&ctl->trajectory, now_us,
                              actual_origin, target,
                              v_max_mdegps,
                              accel_mdegps2, decel_mdegps2);

  ctl->trajectory_active   = true;
  ctl->prev_endpoint_valid = true;
  ctl->prev_endpoint_mdeg  = target;
  ctl->on_completion       = on_completion;
  db_pid_pause(&ctl->pid, false);
  db_pid_reset(&ctl->pid);
  return 0;
}

int db_aggregate_control_drive_forever(struct db_aggregate_control_s *ctl,
                                       uint64_t now_us,
                                       int64_t origin_mdeg,
                                       int32_t v_target_mdegps,
                                       int32_t accel_mdegps2)
{
  aggregate_refresh_settings(ctl);

  /* In-flight retarget — keep PID accumulators / trajectory phase so a
   * 100 Hz re-issue stream does not pin v=0 forever.  Pause state is
   * preserved across the retarget; pass the effective ref-time so the
   * trajectory module sees a consistent clock.
   */

  if (ctl->trajectory_active && ctl->trajectory.infinite)
    {
      uint64_t t_ref_us = aggregate_get_ref_time(ctl, now_us);
      db_trajectory_retarget_forever(&ctl->trajectory, t_ref_us,
                                     v_target_mdegps, accel_mdegps2);
      ctl->prev_endpoint_valid = false;
      ctl->on_completion       = DRIVEBASE_ON_COMPLETION_COAST;
      return 0;
    }

  aggregate_reset_pause(ctl);
  db_trajectory_init_forever(&ctl->trajectory, now_us, origin_mdeg,
                             v_target_mdegps, accel_mdegps2);
  ctl->trajectory_active   = true;
  ctl->prev_endpoint_valid = false;
  ctl->on_completion       = DRIVEBASE_ON_COMPLETION_COAST;
  db_pid_pause(&ctl->pid, false);
  db_pid_reset(&ctl->pid);
  return 0;
}

int db_aggregate_control_drive_hold(struct db_aggregate_control_s *ctl,
                                    uint64_t now_us,
                                    int64_t origin_mdeg)
{
  aggregate_refresh_settings(ctl);

  /* Zero-length trajectory at the current state position.  The PID
   * actively tracks origin_mdeg as the reference; small disturbances
   * pull the controller back to origin.  No SMART continuation —
   * HOLD is not a finite move with a meaningful endpoint.
   */

  aggregate_reset_pause(ctl);
  db_trajectory_init_position(&ctl->trajectory, now_us,
                              origin_mdeg, origin_mdeg,
                              1, 1, 1);
  ctl->trajectory_active   = true;
  ctl->prev_endpoint_valid = false;
  ctl->on_completion       = DRIVEBASE_ON_COMPLETION_HOLD;
  db_pid_pause(&ctl->pid, false);
  db_pid_reset(&ctl->pid);
  return 0;
}

int db_aggregate_control_stop(struct db_aggregate_control_s *ctl,
                              uint64_t now_us,
                              int64_t  current_state_mdeg,
                              uint8_t  on_completion)
{
  aggregate_reset_pause(ctl);
  ctl->trajectory_active   = false;
  ctl->prev_endpoint_valid = false;
  ctl->on_completion       = on_completion;

  switch (on_completion)
    {
      case DRIVEBASE_ON_COMPLETION_COAST:
      case DRIVEBASE_ON_COMPLETION_COAST_SMART:
        db_pid_stop_passive(&ctl->pid);
        ctl->pid.latched_passive_done = true;
        ctl->pid.latched_actuation    = DRIVEBASE_ON_COMPLETION_COAST;
        return 0;

      case DRIVEBASE_ON_COMPLETION_BRAKE:
      case DRIVEBASE_ON_COMPLETION_BRAKE_SMART:
        db_pid_stop_passive(&ctl->pid);
        ctl->pid.latched_passive_done = true;
        ctl->pid.latched_actuation    = DRIVEBASE_ON_COMPLETION_BRAKE;
        return 0;

      case DRIVEBASE_ON_COMPLETION_HOLD:
      default:
        /* HOLD-on-stop = active hold at the current state position.
         * Re-arm with a zero-length trajectory so the PID has a fixed
         * reference and small disturbances get pulled back.
         */

        return db_aggregate_control_drive_hold(ctl, now_us,
                                               current_state_mdeg);
    }
}

void db_aggregate_control_update(struct db_aggregate_control_s *ctl,
                                 uint64_t now_us,
                                 int64_t  state_x_mdeg,
                                 int32_t  state_v_mdegps,
                                 uint32_t dt_ms,
                                 struct db_aggregate_output_s *out)
{
  /* Invariant safety-net: trajectory_active == false ⇒ traj_paused must
   * be false.  Explicit reset/stop/start paths already call
   * aggregate_reset_pause(), but a trailing path with latched_passive_
   * done == true and trajectory_active == false (e.g. stop with
   * COAST/BRAKE) would otherwise leak pause state into the next command.
   */

  if (!ctl->trajectory_active)
    {
      aggregate_reset_pause(ctl);
    }

  /* Inactive AND not latched = no command staged.  Safe-default
   * COAST output keeps motors free until someone arms a trajectory.
   */

  if (!ctl->trajectory_active && !ctl->pid.latched_passive_done)
    {
      out->duty         = 0;
      out->ref_v_mdegps = 0;
      out->actuation    = DRIVEBASE_ON_COMPLETION_COAST;
      out->done         = ctl->pid.done;
      return;
    }

  /* If we are inactive but latched, db_pid_update's early-return path
   * will surface the latched actuation/duty.  The trajectory reference
   * values it ignores can be left at "hold here" so the input struct
   * is well-formed.
   */

  struct db_trajectory_ref_s ref;
  if (ctl->trajectory_active)
    {
      uint64_t t_ref_us = aggregate_get_ref_time(ctl, now_us);
      db_trajectory_get_reference(&ctl->trajectory, t_ref_us, &ref);
    }
  else
    {
      ref.x_mdeg    = state_x_mdeg;
      ref.v_mdegps  = 0;
      ref.a_mdegps2 = 0;
      ref.done      = true;
    }

  /* Feed-forward (Issue #127 Phase 6 Step 6.1, Plan D7).  kV/kA are
   * per-axis here; the (non-linear) per-motor kS friction term is added
   * by drivebase_drivebase.c after the L/R compose.
   *
   * Unit-scaling note: trajectory ref is in mdeg/s and mdeg/s^2; the
   * settings gains are in .01% per (deg/s) and .01% per (deg/s^2).  To
   * avoid a 1000x scaling error AND to keep the entire calculation in
   * int32 (no __aeabi_ldivmod in the RT path — Plan D7 Codex Round 3
   * BLOCKING), do the /1000 conversion FIRST in int32, then multiply.
   *
   *   gain * (mdeg/s / 1000) -> .01% * deg/s = .01% duty
   *
   * Bounds (with DB_FF_GAIN_LIMIT=1000, v_max~=1110, a_max~=800):
   *   |kV * v_dps|  <= 1000 * 1110 = 1.11e6  (int32 OK)
   *   |kA * a_dps2| <= 1000 *  800 = 8.0e5   (int32 OK)
   *   sum <= ~2e6 << 2^31, no int64 needed.
   */

  int32_t duty_ff = 0;
  if (ctl->ff_axis != NULL && ctl->trajectory_active)
    {
      int32_t v_dps  = ref.v_mdegps  / 1000;
      int32_t a_dps2 = ref.a_mdegps2 / 1000;
      duty_ff        = ctl->ff_axis->kV * v_dps
                     + ctl->ff_axis->kA * a_dps2;
    }

  struct db_pid_input_s in =
    {
      .ref_x_mdeg      = ref.x_mdeg,
      .ref_v_mdegps    = ref.v_mdegps,
      .ref_a_mdegps2   = ref.a_mdegps2,
      .act_x_mdeg      = state_x_mdeg,
      .act_v_mdegps    = state_v_mdegps,
      .dt_ms           = dt_ms ? dt_ms : ctl->tick_ms,
      .gains           = ctl->gains,
      .completion      = ctl->completion,
      .duty_ff         = duty_ff,
      .trajectory_done = ref.done,
      .on_completion   = ctl->on_completion,
    };

  struct db_pid_output_s pout;
  db_pid_update(&ctl->pid, &in, &pout);
  out->duty         = pout.duty;
  out->ref_v_mdegps = ref.v_mdegps;
  out->actuation    = pout.actuation;
  out->done         = pout.done;

  /* Reference-time pause decision (Issue #142, Phase 5 D).  Mirrors
   * pybricks `lib/pbio/src/control.c:302-319`.  Pause if and only if:
   *   (1) An active trajectory is running and the PID is not latched off
   *   (2) Total output is rail-clamped IN THE SAME DIRECTION as the
   *       proportional term (sign(P) ≡ sign(pos_err) because kp_pos > 0).
   *       This is a V1 approximation of pybricks' `|torque_proportional|
   *       >= max_windup_torque`; see Issue #142 for the known false-
   *       negative/false-positive edges
   *   (3) Proportional sign is NOT opposite the velocity error sign
   *       (otherwise the controller is correctly braking against the
   *       reference, not winding up)
   *   (4) Proportional sign is NOT opposite the reference acceleration
   *       sign (otherwise we are decelerating intentionally, e.g. the
   *       trapezoidal decel segment).
   */

  if (ctl->trajectory_active)
    {
      int64_t pos_err_64 = ref.x_mdeg - state_x_mdeg;
      int32_t speed_err  = ref.v_mdegps - state_v_mdegps;

      int32_t sign_p = (pos_err_64 > 0) - (pos_err_64 < 0);
      int32_t sign_v = (speed_err   > 0) - (speed_err   < 0);
      int32_t sign_a = (ref.a_mdegps2 > 0) - (ref.a_mdegps2 < 0);

      bool p_limited =
          (sign_p > 0 && ctl->pid.output_saturated_high) ||
          (sign_p < 0 && ctl->pid.output_saturated_low);

      bool want_pause =
          !ctl->pid.latched_passive_done &&
          p_limited &&
          !(sign_p != 0 && sign_v != 0 && sign_p == -sign_v) &&
          !(sign_p != 0 && sign_a != 0 && sign_p == -sign_a);

      aggregate_set_pause(ctl, want_pause, now_us);
    }
}

bool db_aggregate_control_is_done(const struct db_aggregate_control_s *ctl)
{
  return ctl->pid.done;
}

bool db_aggregate_control_is_active(const struct db_aggregate_control_s *ctl)
{
  return ctl->trajectory_active;
}
