"""Category D: drivebase daemon tests (Issue #77).

Non-interactive cases exercise the kernel /dev/drivebase chardev and
the userspace daemon's lifecycle FSM without requiring physical
motors:

    - DRIVEBASE_GET_STATUS works pre-attach (no daemon)
    - drivebase_daemon_start spawns task + RT pthread
    - GET_STATUS / GET_STATE round-trip while the daemon is alive
      (regression for the TLS_ALIGNED heap-fragmentation fix that
      otherwise made NSH report ``command not found`` for any
      drivebase verb after `drivebase start`)
    - drivebase_daemon_stop tears down cleanly and bumps
      attach_generation
    - Idempotency: second start while running returns -EALREADY,
      stop while stopped returns -EAGAIN

Interactive cases require a motor pair on odd/even ports — they're
behind the `interactive` mark so CI runs the smoke set without them.
"""

import re
import time

import pytest


# ---------------------------------------------------------------------------
# Helpers — small parsers for the CLI's printf surface
# ---------------------------------------------------------------------------

_STATUS_RE = re.compile(r"^\s*([A-Za-z_]+)\s*=\s*(\d+)\s*$", re.MULTILINE)


def _parse_status(text):
    """Parse `drivebase status` lines into a {field: int} dict."""
    return {m.group(1): int(m.group(2)) for m in _STATUS_RE.finditer(text)}


_STATE_HEADER = (
    "time_ms", "dist_mm", "v_mmps", "angle_mdeg", "tr_dps",
    "done", "stall", "cmd", "tick",
)
# CLI exposes columns under names that don't 1:1 match the legacy
# key=value surface; this map keeps existing assertions working.
_STATE_ALIASES = {"dist": "dist_mm", "v": "v_mmps", "angle": "angle_mdeg"}


def _parse_state(text):
    """Parse `drivebase get-state` table output.

    Returns the *first* data row as a dict keyed by header name.  Both
    the modern column names (``dist_mm``) and the legacy short names
    (``dist``) are accessible so older assertions keep working.
    """
    rows = []
    for line in text.splitlines():
        toks = line.split()
        if len(toks) != len(_STATE_HEADER):
            continue
        if toks == list(_STATE_HEADER):
            continue
        try:
            rows.append({k: int(v) for k, v in zip(_STATE_HEADER, toks)})
        except ValueError:
            continue
    if not rows:
        return {}
    row = rows[0]
    for short, full in _STATE_ALIASES.items():
        row[short] = row[full]
    return row


def _ensure_stopped(p, timeout=15.0):
    """Make sure no daemon is alive entering a test.

    Issue #120 (9134953) added rcS auto-start of drivebase at boot, so
    after reboot the daemon is already attached.  Two factors stretch
    the time it takes for daemon_attached to actually clear after the
    test issues `drivebase stop`:

      1. drivebase_daemon_stop() only waits 2 s internally for
         teardown_done; if the daemon is still in INITIALISING (motor
         init / mode-2 select / IMU open), it does not poll `running`
         until reaching the RUNNING idle loop — the CLI returns
         -ETIMEDOUT but the daemon continues teardown asynchronously.
      2. test_crash reboots the Hub four times immediately before the
         drivebase tests, so D-1 hits the Hub mid-init right after the
         last watchdog reset, when motor / IMU bring-up has not
         finished.

    Retry `drivebase stop` periodically and poll daemon_attached until
    it clears or the (generous) outer deadline expires.
    """
    deadline = time.time() + timeout
    last_stop = 0.0
    while time.time() < deadline:
        # Re-issue stop every 3 s — covers the case where the first
        # call hit -ETIMEDOUT because the daemon was still finishing
        # INITIALISING, but the next attempt finds the daemon in
        # RUNNING and the synchronous teardown wait succeeds.
        if time.time() - last_stop > 3.0:
            p.sendCommand("drivebase stop", timeout=5)
            last_stop = time.time()
        time.sleep(0.3)
        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        if st.get("daemon_attached") == 0:
            return
    # Fall through with whatever state we have; the caller's assertion
    # will surface the failure with the real status snapshot.


# ---------------------------------------------------------------------------
# Non-interactive: chardev ABI + lifecycle
# ---------------------------------------------------------------------------


def test_drivebase_status_no_daemon(p):
    """D-1: GET_STATUS returns a zeroed snapshot when no daemon attached.

    The chardev is registered at board bringup so `/dev/drivebase` is
    always openable; the user-facing GET_STATUS path explicitly does
    not require ATTACH so monitoring tools work even before the daemon
    starts.
    """

    _ensure_stopped(p)
    out = p.sendCommand("drivebase status", timeout=5)
    st = _parse_status(out)

    assert st["daemon_attached"] == 0
    assert st["configured"] == 0
    assert st["motor_l_bound"] == 0
    assert st["motor_r_bound"] == 0
    assert st["use_gyro"] == 0
    # tick counters / publish timestamps are always 0 pre-attach
    assert st["tick_count"] == 0
    assert st["last_publish_us"] == 0


def test_drivebase_stop_when_not_running(p):
    """D-2: `drivebase stop` against a stopped daemon returns EAGAIN."""

    _ensure_stopped(p)
    out = p.sendCommand("drivebase stop", timeout=5)
    # The CLI maps -EAGAIN to "not running" so the human-readable
    # surface stays stable even if errno renaming happens upstream.
    assert "not running" in out, f"unexpected stop output: {out!r}"


def test_drivebase_start_status_stop_cycle(p):
    """D-3: start → status (under live daemon) → stop → status round-trip.

    This is the regression for the Issue #77 KNOWN LIMITATION that the
    TLS_ALIGNED=y default in BUILD_PROTECTED was forcing every user
    task stack into 8 KB-aligned slots; daemon allocation fragmented
    Umem so the next CLI task_spawn returned -ENOMEM and NSH reported
    "command not found" for every drivebase verb.  With TLS_ALIGNED
    off (commit fix: #77), the CLI must spawn cleanly while the
    daemon is alive.
    """

    _ensure_stopped(p)
    start = p.sendCommand("drivebase start", timeout=8)
    assert "started" in start, f"start failed: {start!r}"
    # Daemon needs ~30 ms to motor_select_mode + open IMU + spawn RT
    # pthread before its first PUBLISH_STATE; give it a comfortable
    # margin so we are not racing the lifecycle FSM.
    time.sleep(2.0)

    # The KNOWN LIMITATION: this exact ioctl used to return
    # "command not found" because exec_builtin was ENOMEM-failing.
    out = p.sendCommand("drivebase status", timeout=8)
    assert "command not found" not in out, (
        "regression: drivebase CLI cannot spawn while daemon is alive"
    )
    st = _parse_status(out)
    assert st["daemon_attached"] == 1
    assert st["configured"] == 1
    # `last_publish_us` increments every RT tick (5 ms cadence) — a
    # nonzero reading proves the publish path is round-tripping.
    assert st["last_publish_us"] > 0

    stop = p.sendCommand("drivebase stop", timeout=8)
    assert "stopped" in stop, f"stop failed: {stop!r}"
    time.sleep(0.3)

    # attach_generation must have advanced so a re-ATTACH would not
    # collide with the previous session's state.
    out = p.sendCommand("drivebase status", timeout=5)
    st = _parse_status(out)
    assert st["daemon_attached"] == 0
    assert st["attach_generation"] >= 1


def test_drivebase_start_with_tick_arg(p):
    """D-3b: `drivebase start 56 112 5` boots with a 5 ms RT tick.

    Issue #120 made the RT tick settable as the third positional arg.
    Verifies the daemon comes up cleanly with a non-default tick and
    publishes through GET_STATUS / GET_STATE the same way as the
    default 2 ms cadence.
    """

    _ensure_stopped(p)
    start = p.sendCommand("drivebase start 56 112 5", timeout=8)
    assert "started" in start, f"start failed: {start!r}"
    # Issue #143 (b501935) changed the banner from "tick=5 ms" to the
    # microseconds form, so 5 ms = "tick=5000 us".
    assert "tick=5000 us" in start, (
        f"missing tick echo in start banner: {start!r}"
    )
    time.sleep(1.0)

    out = p.sendCommand("drivebase status", timeout=5)
    st = _parse_status(out)
    assert st["daemon_attached"] == 1
    assert st["last_publish_us"] > 0

    p.sendCommand("drivebase stop", timeout=8)
    time.sleep(0.3)


def test_drivebase_start_rejects_bad_tick(p):
    """D-3c: tick_ms outside [1, 20] is rejected before the daemon spawns."""

    _ensure_stopped(p)

    out = p.sendCommand("drivebase start 56 112 0", timeout=5)
    assert "tick_ms must be in" in out, f"expected reject: {out!r}"

    out = p.sendCommand("drivebase start 56 112 25", timeout=5)
    assert "tick_ms must be in" in out, f"expected reject: {out!r}"

    # Confirm rejected attempts did not actually start the daemon.
    out = p.sendCommand("drivebase status", timeout=5)
    st = _parse_status(out)
    assert st["daemon_attached"] == 0, (
        f"daemon should not have attached after reject: {out!r}"
    )


def test_drivebase_start_already_running(p):
    """D-4: second `drivebase start` while running returns EALREADY."""

    _ensure_stopped(p)
    p.sendCommand("drivebase start", timeout=8)
    time.sleep(2.0)

    out = p.sendCommand("drivebase start", timeout=5)
    # CLI surfaces the negated errno via strerror.  NuttX libc does
    # not carry a string for EALREADY (114) so the prefix lands as
    # "Unknown error 114"; tolerate both forms.
    assert "already" in out.lower() or "114" in out, (
        f"second start should reject with -EALREADY: {out!r}"
    )

    p.sendCommand("drivebase stop", timeout=8)
    time.sleep(0.3)


def test_drivebase_get_state_under_live_daemon(p):
    """D-5: GET_STATE returns RT-thread tick metadata.

    Verifies the second user-facing ioctl (DRIVEBASE_GET_STATE,
    state_db active-slot read) survives the same heap pressure
    scenario as GET_STATUS, and that the RT thread is actually
    running ticks.

    `done` is intentionally NOT asserted on: drivebase_control.c
    initializes pid.done=false at db_pid_init/reset and only flips it
    to true after a finite trajectory finishes within tolerance for
    `done_window_ms`.  At fresh start with no command issued there is
    no trajectory, so the steady-state idle value is 0.  `cmd=0`
    (no active command) is the meaningful invariant for "no command
    in flight" and is checked instead.
    """

    _ensure_stopped(p)
    p.sendCommand("drivebase start", timeout=8)
    time.sleep(1.0)

    out = p.sendCommand("drivebase get-state", timeout=8)
    state = _parse_state(out)
    assert state.get("cmd") == 0, f"unexpected cmd in idle: {out!r}"
    # tick must be advancing at the configured RT cadence — anything
    # past a handful proves the RT pthread reached its loop.
    assert state.get("tick", 0) > 50, f"RT loop not progressing: {out!r}"

    p.sendCommand("drivebase stop", timeout=8)
    time.sleep(0.3)


def test_drivebase_multiple_start_stop_cycles(p):
    """D-6: 3× start/stop cycle leaves the daemon detached cleanly.

    Verifies the lifecycle FSM is genuinely re-entrant: previous
    teardown drained the cmd_ring, released the LEGOSENSOR CLAIMs,
    and let the chardev rearm without leaking the attach_generation
    counter into a stuck state.
    """

    _ensure_stopped(p)
    last_gen = 0
    for cycle in range(3):
        start = p.sendCommand("drivebase start", timeout=8)
        assert "started" in start, f"cycle {cycle} start failed: {start!r}"
        time.sleep(1.0)

        stop = p.sendCommand("drivebase stop", timeout=8)
        assert "stopped" in stop, f"cycle {cycle} stop failed: {stop!r}"
        time.sleep(0.3)

        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        assert st["daemon_attached"] == 0, (
            f"cycle {cycle}: daemon still attached after stop: {st}"
        )
        assert st["attach_generation"] > last_gen, (
            f"cycle {cycle}: attach_generation did not advance"
        )
        last_gen = st["attach_generation"]


def test_drivebase_other_builtins_unaffected_by_daemon(p):
    """D-7: spawning other NSH builtins keeps working under live daemon.

    Cross-check for the heap-fragmentation regression: even with the
    daemon's 8 KB main stack + 4 KB RT pthread stack consuming user
    heap, lightweight builtins like `hello` still spawn — both before
    and after the TLS_ALIGNED fix.  If this fails, something else has
    started leaking user-heap chunks the way the alignment penalty
    used to.
    """

    _ensure_stopped(p)
    p.sendCommand("drivebase start", timeout=8)
    time.sleep(1.5)

    out = p.sendCommand("hello", timeout=5)
    assert "Hello, World!!" in out, (
        f"hello builtin failed under daemon load: {out!r}"
    )

    p.sendCommand("drivebase stop", timeout=8)
    time.sleep(0.3)


# ---------------------------------------------------------------------------
# Non-interactive: `_alg ff-trace` offline feed-forward replay (Phase 7,
# Issue #158).  No motors, no daemon required — geometry falls back to the
# compiled 56/112 mm SPIKE default when the daemon is stopped, so these
# are deterministic.
# ---------------------------------------------------------------------------


_FF_AXIS_HDR_RE = re.compile(
    r"^#\s+(?P<axis>distance|heading)\s+axis:\s+x1=(?P<x1>-?\d+)\s+"
    r"v_peak=(?P<v>-?\d+).*kV=(?P<kv>-?\d+)\s+kA=(?P<ka>-?\d+)",
    re.MULTILINE,
)


def test_alg_ff_trace_turn_state_space_conversion(p):
    """D-FFTRACE-1: `_alg ff-trace turn` uses heading STATE-space units.

    The decisive units check (Issue #158): a turn's heading state delta
    is `deg * 1000 * axle_t / wheel_d`.  With the default 56/112 mm
    geometry the axle/wheel ratio is exactly 2, so `turn 90` must report
    x1 = 90000 * 2 = 180000 motor-mdeg.  A regression to chassis-degree
    (x1 = 90000) or a wrong ratio would fail here.  Daemon stopped so the
    compiled default geometry is used.
    """
    _ensure_stopped(p)
    out = p.sendCommand("drivebase _alg ff-trace turn 90", timeout=5)
    m = _FF_AXIS_HDR_RE.search(out)
    assert m and m.group("axis") == "heading", (
        f"ff-trace turn missing heading axis header: {out!r}"
    )
    x1 = int(m.group("x1"))
    assert x1 == 180000, (
        f"ff-trace turn 90 state-space x1={x1}, expected 180000 "
        f"(= 90000 * axle/wheel=2): {out!r}"
    )


def test_alg_ff_trace_duty_ff_matches_gains(p):
    """D-FFTRACE-2: duty_ff column equals kV*(v/1000)+kA*(a/1000).

    Self-consistency check of the FF formula the trace shares with the
    RT path.  Parse the header kV/kA and a cruise-phase row (a=0, v at
    peak), then confirm the printed duty_ff matches kV*(v/1000) exactly.
    Uses the distance axis where kV is non-zero in production.
    """
    _ensure_stopped(p)
    out = p.sendCommand("drivebase _alg ff-trace straight 300", timeout=5)
    m = _FF_AXIS_HDR_RE.search(out)
    assert m and m.group("axis") == "distance", (
        f"ff-trace straight missing distance axis header: {out!r}"
    )
    kv = int(m.group("kv"))
    ka = int(m.group("ka"))

    # Data rows: t_ms ref_x ref_v ref_a duty_ff  (5 signed ints).
    row_re = re.compile(
        r"^\s*(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s+(-?\d+)\s*$",
        re.MULTILINE,
    )
    rows = [tuple(int(x) for x in g) for g in row_re.findall(out)]
    assert rows, f"ff-trace straight produced no data rows: {out!r}"

    def _div1000_trunc(n):
        # C integer division truncates toward zero; mirror it exactly.
        return -((-n) // 1000) if n < 0 else n // 1000

    checked = 0
    for _t, _x, v, a, duty_ff in rows:
        v_dps = _div1000_trunc(v)
        a_dps2 = _div1000_trunc(a)
        expect = kv * v_dps + ka * a_dps2
        assert duty_ff == expect, (
            f"duty_ff={duty_ff} != kV*{v_dps}+kA*{a_dps2}={expect} "
            f"(row v={v} a={a}): {out!r}"
        )
        checked += 1
    assert checked >= 3, f"too few ff-trace rows to validate: {out!r}"


# ---------------------------------------------------------------------------
# Interactive: require physical motor pair (odd + even port)
# ---------------------------------------------------------------------------


@pytest.mark.interactive
def test_drivebase_motor_drain_with_motors(p):
    """D-INT-1: hidden `_motor read` verb walks the encoder fd path.

    Requires SPIKE Medium Motors on at least two ports — one odd
    (B/D/F → motor_l) and one even (A/C/E → motor_r).  Verifies the
    `drivebase_motor_init` → CLAIM → SELECT mode 2 → drain pipeline
    against real LUMP traffic before we stand up the full daemon.
    """

    p.waitUser(
        "Plug a SPIKE Medium Motor into both an odd port (B/D/F) and an "
        "even port (A/C/E), wait for SYNCED in dmesg, then press Enter"
    )
    _ensure_stopped(p)

    out = p.sendCommand("drivebase _motor read l", timeout=5)
    assert "raw=" in out, f"motor_l drain returned no sample: {out!r}"
    assert "type=" in out, f"motor_l drain missing type: {out!r}"

    out = p.sendCommand("drivebase _motor read r", timeout=5)
    assert "raw=" in out, f"motor_r drain returned no sample: {out!r}"
    assert "type=" in out, f"motor_r drain missing type: {out!r}"


@pytest.mark.interactive
def test_drivebase_drive_straight_with_motors(p):
    """D-INT-2: end-to-end `drive_straight` distance closes the loop.

    Requires the full SPIKE drivebase wheel/axle (default 56 mm wheel,
    112 mm axle).  Verifies the daemon FSM + RT loop + observer + PID
    chain converge on a target distance and the on_completion=BRAKE
    policy fires.  Visual confirmation (the robot moves ~200 mm
    forward) is required because the encoder reading alone could come
    from a wheel hand-spun in mid-air.
    """

    p.waitUser(
        "Mount the drivebase, place it on the floor with ~30 cm clear "
        "ahead, and press Enter (the robot will move forward ~200 mm)"
    )

    _ensure_stopped(p)
    p.sendCommand("drivebase start", timeout=8)
    time.sleep(2.0)

    p.sendCommand("drivebase straight 200 brake", timeout=5)
    # The trapezoidal trajectory at ~458 mdeg/s peak velocity needs
    # ~1.5 sec for 200 mm; sample state at 2 sec to be past completion.
    time.sleep(2.0)

    out = p.sendCommand("drivebase get-state", timeout=5)
    state = _parse_state(out)
    assert state.get("done") == 1, f"drive not done: {out!r}"
    # Allow ±15 % tolerance — encoder slip + tile friction varies.
    dist = state.get("dist", 0)
    assert 170 <= dist <= 230, f"distance off target (200 mm): {out!r}"

    p.sendCommand("drivebase stop", timeout=8)
    p.waitUser("Confirm the robot moved roughly 200 mm forward then stopped")
