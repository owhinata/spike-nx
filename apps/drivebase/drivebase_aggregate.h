/****************************************************************************
 * apps/drivebase/drivebase_aggregate.h
 *
 * Aggregate distance/heading PID controller for the drivebase daemon
 * (Issue #141 Phase 2 A).  Replaces the per-motor closed-loop servo
 * with two top-level controllers — one for the average wheel travel
 * (state_distance = (sL + sR) / 2) and one for the L-R differential
 * (state_heading = (sR - sL) / 2 in motor-mdeg, spike-nx convention:
 * positive = CCW = right wheel ahead).
 *
 * Per pybricks `lib/pbio/src/drivebase.c:55-525` but adapted to:
 *
 *   - Duty output (SPIKE H-bridge), not torque.  Output unit is the
 *     ABI-standard .01% duty, range [-out_max, +out_max].  Final L/R
 *     compose happens outside this module (drivebase_drivebase.c) and
 *     uses the spike-nx-native sign convention:
 *
 *        left_duty  = duty_distance - duty_heading
 *        right_duty = duty_distance + duty_heading
 *
 *     (Positive heading PID output → right wheel faster → CCW yaw,
 *     matching the publicly exposed `angle_mdeg` semantics.)
 *
 *   - State + speed are passed in by the aggregator each tick — the
 *     per-motor `db_observer_s` instances still own the encoder slope
 *     estimation, and the aggregator builds state_distance/state_
 *     heading from their outputs.  This module never touches motor I/O.
 *
 *   - The pybricks "coast/brake propagation" rule (if EITHER axis
 *     decides PBIO_DCMOTOR_ACTUATION_COAST or BRAKE, both axes stop)
 *     is implemented at the aggregator level.  Each axis reports its
 *     own actuation hint per tick (DRIVE_DUTY / COAST / BRAKE / HOLD);
 *     the aggregator inspects both and propagates.
 *
 *   - SMART continuation (`prev_endpoint_*`) is per-axis and only
 *     meaningful for the "primary" axis of each user-level command
 *     (distance for STRAIGHT, heading for TURN).  The aggregator
 *     decides which axis is primary; this module just records the
 *     prev endpoint when a finite trajectory completes cleanly.
 *
 *   - The #132 passive-completion latch (COAST/BRAKE/SMART_expired
 *     only) lives inside `db_pid_state_s` and is reused here.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_AGGREGATE_H
#define __APPS_DRIVEBASE_DRIVEBASE_AGGREGATE_H

#include <stdbool.h>
#include <stdint.h>

#include "drivebase_internal.h"
#include "drivebase_control.h"
#include "drivebase_trajectory.h"

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Types
 ****************************************************************************/

struct db_aggregate_control_s
{
  enum db_axis_e            axis;

  /* Nominal control-loop period in ms (= drivebase RT tick).  Used as
   * the PID dt fallback when measured Δt is unavailable.
   */

  uint32_t                  tick_ms;

  /* Per-axis PID + trajectory state.  trajectory operates in the
   * axis's state space (motor mdeg for both distance and heading).
   */

  struct db_pid_state_s     pid;
  struct db_trajectory_s    trajectory;

  bool                      trajectory_active;
  bool                      prev_endpoint_valid;
  int64_t                   prev_endpoint_mdeg;
  uint8_t                   on_completion;

  /* Cached gain pointers (axis-specific, fetched from
   * db_settings_pid_gains(axis) at init).  Stored as a pointer so the
   * PID input can hand them straight to db_pid_update without copying.
   */

  const struct db_servo_gains_s        *gains;
  const struct db_completion_settings_s *completion;

  /* Per-axis feed-forward gain pointer (Issue #127 Phase 6 Step 6.1).
   * Same lifetime contract as `gains`: the settings module is frozen
   * before the RT thread starts, after which the dereference is racing
   * only with itself.  Per-motor kS friction (Step 6.2) lives on the
   * drivebase top-level, not here, because the L/R compose makes the
   * kS distribution non-linear in axis space (Plan D1 Codex BLOCKING).
   */

  const struct db_ff_axis_gains_s      *ff_axis;

  /* Reference-time pause for trajectory anti-windup (Issue #142, Phase 5
   * D).  Mirrors pybricks `pbio_position_integrator` (lib/pbio/src/
   * integrator.c L142-218): when the proportional output is pinned at
   * its rail in the direction of the position error and ref is not
   * decelerating, freeze trajectory time so the reference does not run
   * away from the state.  Each axis owns its own pause clock; resume
   * adds (now - pause_begin) to total_us so phase is continuous.
   *
   * Invariant: trajectory_active == false ⇒ traj_paused == false.
   * Enforced by aggregate_reset_pause() on every command/reset/stop and
   * by a safety-net check at the top of db_aggregate_control_update().
   */

  bool                      traj_paused;
  uint64_t                  traj_pause_begin_us;
  uint64_t                  traj_pause_total_us;
};

struct db_aggregate_output_s
{
  int32_t  duty;                 /* signed PWM duty in .01 % units     */
  int32_t  ref_v_mdegps;         /* trajectory reference speed this    */
                                 /* tick, exported for Step 6.2 per-   */
                                 /* motor kS friction FF (the L/R      */
                                 /* compose needs both axes' v_ref to  */
                                 /* form sign(vL_ref) / sign(vR_ref)). */
  uint8_t  actuation;            /* enum drivebase_on_completion_e —    */
                                 /* what the aggregator should do      */
                                 /* (DRIVE_DUTY / COAST / BRAKE / HOLD) */
  bool     done;                 /* current done state of this axis    */
  bool     trajectory_done;      /* trajectory finished (ref.done).    */
                                 /* Issue #158 Phase 7: lets the L/R   */
                                 /* compose gate the terminal breakaway*/
                                 /* on "trajectory ended" vs the post- */
                                 /* tolerance `done` above.            */
  int32_t  pos_err_mdeg;         /* ref_x - act_x this tick (clamped   */
                                 /* to int32).  Phase 7: per-wheel err */
                                 /* = D.pos_err ∓ H.pos_err drives the */
                                 /* breakaway direction.               */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* Initialise an aggregate axis controller.  Idempotent — re-init keeps
 * the gain/completion pointers fresh and zeroes PID + trajectory state.
 */

void db_aggregate_control_init(struct db_aggregate_control_s *ctl,
                               enum db_axis_e axis,
                               uint32_t tick_ms);

/* Reset internal state (PID + trajectory + SMART memory) to "stopped at
 * the given state position".  Called after a fresh encoder origin (e.g.
 * db_drivebase_reset).
 */

void db_aggregate_control_reset(struct db_aggregate_control_s *ctl);

/* Plan a finite relative move from `origin_mdeg` of `delta_mdeg`,
 * tracked by a trapezoidal trajectory bounded by `v_max_mdegps` /
 * `accel_mdegps2` / `decel_mdegps2`.  on_completion ∈ enum
 * drivebase_on_completion_e.  SMART continuation uses
 * `smart_continue_window_mdeg` (Phase 1 F decoupled from pos_tolerance):
 * if `|prev_endpoint - origin| < smart_continue_window_mdeg`, the new
 * move starts from `prev_endpoint` instead of `origin` (pybricks
 * "continue from endpoint").
 */

int  db_aggregate_control_drive_position(struct db_aggregate_control_s *ctl,
                                         uint64_t now_us,
                                         int64_t origin_mdeg,
                                         int64_t delta_mdeg,
                                         int32_t v_max_mdegps,
                                         int32_t accel_mdegps2,
                                         int32_t decel_mdegps2,
                                         uint8_t on_completion);

/* Plan an infinite drive at signed `v_target_mdegps`.  No completion.
 * Re-arming an already-active infinite trajectory retargets in flight
 * (no PID accumulator reset, no reference jump).
 */

int  db_aggregate_control_drive_forever(struct db_aggregate_control_s *ctl,
                                        uint64_t now_us,
                                        int64_t origin_mdeg,
                                        int32_t v_target_mdegps,
                                        int32_t accel_mdegps2);

/* Plan a zero-length "hold" trajectory at the given position.  Used
 * for the auxiliary axis of asymmetric user commands (e.g. STRAIGHT
 * holds heading at current, TURN holds distance at current).  The PID
 * stays active; on_completion is forced to HOLD so the actuation hint
 * is HOLD regardless of completion timing.
 */

int  db_aggregate_control_drive_hold(struct db_aggregate_control_s *ctl,
                                     uint64_t now_us,
                                     int64_t origin_mdeg);

/* Stop trajectory immediately and apply the requested actuation
 * passively (PID enters latched_passive_done for COAST/BRAKE/SMART_
 * expired; for HOLD, the controller is re-armed as a zero-length
 * trajectory at the current state position).  Mirrors db_servo_stop()
 * semantics from pre-#141.
 */

int  db_aggregate_control_stop(struct db_aggregate_control_s *ctl,
                               uint64_t now_us,
                               int64_t  current_state_mdeg,
                               uint8_t  on_completion);

/* Per-tick update: take current state (position + speed), produce a
 * duty + actuation hint.  Caller is responsible for clamping the
 * compose result to per-side duty limits and applying it via the
 * (slim) servo I/O.  Mirrors db_servo_update() but with state
 * provided externally — no observer touch.
 */

void db_aggregate_control_update(struct db_aggregate_control_s *ctl,
                                 uint64_t now_us,
                                 int64_t  state_x_mdeg,
                                 int32_t  state_v_mdegps,
                                 uint32_t dt_ms,
                                 struct db_aggregate_output_s *out);

/* Convenience: returns true if the controller's PID done bit is set.
 * Aggregate done at the drivebase level = is_done(distance) &&
 * is_done(heading).
 */

bool db_aggregate_control_is_done(const struct db_aggregate_control_s *ctl);

/* Convenience: returns true while a non-stopped trajectory is active. */

bool db_aggregate_control_is_active(const struct db_aggregate_control_s *ctl);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_AGGREGATE_H */
