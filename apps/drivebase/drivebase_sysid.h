/****************************************************************************
 * apps/drivebase/drivebase_sysid.h
 *
 * System-identification CLI verbs (Issue #152 Phase 6 Step 6.4).
 *
 * Measures the SPIKE Medium Motor + drivebase plant parameters needed
 * to seed the Phase 6 feed-forward (`u = sign(v_ref)·kS + kV·v_ref +
 * kA·a_ref`) without depending on the closed-loop PID stack.  All
 * measurement loops drive duty directly via drivebase_motor_set_duty
 * (PID + aggregate bypass) and read motor speed via a side-local
 * observer instance built from the raw encoder samples drained on the
 * CLI task.
 *
 * Entry contract (Plan D8):
 *
 *   - The drivebase daemon MUST be stopped (`daemon_attached==0`).
 *     If a daemon is running, every verb returns -EBUSY immediately;
 *     the RT thread would otherwise overwrite our duty commands on
 *     its 2 ms tick.
 *
 *   - The robot MUST be on the ground, on a flat surface, with clear
 *     room to drive a few meters.  Free-spin ("wheels up") measurement
 *     underestimates kS (no carpet / floor friction), and kA cannot be
 *     measured at all without the inertia of the actual chassis +
 *     wheels.  The help string spells this out; the CLI does not
 *     enforce it (we have no sensor for it).
 *
 *   - On exit (normal or Ctrl-C), every verb leaves both motors in
 *     COAST so the robot does not roll into a wall.
 *
 *   - Verbs are ordered:  ramp-ks  →  ramp-kv  →  ramp-ka.  ramp-kv
 *     needs the kS measured by ramp-ks to subtract friction from the
 *     residual; ramp-ka needs the kV from ramp-kv to subtract back-
 *     EMF from the residual.  Running them out of order produces
 *     biased gains; the dispatch does not enforce the ordering but
 *     the verb signatures require the caller to pass in the prior
 *     result.
 *
 *   - Output is print-only — there is no `set` command.  The operator
 *     copies the measured values into /mnt/flash/drivebase.cfg (or
 *     into source defaults for a permanent change).  Auto-write is a
 *     separate Issue's responsibility.
 *
 * vbat normalisation (Plan D8 Codex Round 2 BLOCKING):
 *   Both kS and kV scale inversely with vbat — a measurement taken at
 *   low vbat OVERESTIMATES the gain because more duty was required to
 *   reach the same physical effect.  To normalise to the
 *   `battery_nominal_mv` reference (the voltage at which Phase 6
 *   correction targets ×1):
 *       gain_nominal = gain_measured * vbat_mv / battery_nominal_mv
 *   i.e. MULTIPLY by `vbat/nominal` (a number < 1 when below nominal).
 *   Every verb prints both the raw measurement and the nominal-
 *   normalised value with vbat recorded inline.
 ****************************************************************************/

#ifndef __APPS_DRIVEBASE_DRIVEBASE_SYSID_H
#define __APPS_DRIVEBASE_DRIVEBASE_SYSID_H

#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* `drivebase _sysid <subverb> ...` entry point.  Called from the CLI
 * dispatch in drivebase_main.c after the `_sysid` verb is recognised.
 * argc / argv are the post-`_sysid` tokens (subverb is argv[0]).
 *
 * Returns 0 on success, non-zero on usage error / measurement failure.
 * Every exit path coasts both motors before returning.
 */

int drivebase_sysid_cli(int argc, char *argv[]);

#ifdef __cplusplus
}
#endif

#endif /* __APPS_DRIVEBASE_DRIVEBASE_SYSID_H */
