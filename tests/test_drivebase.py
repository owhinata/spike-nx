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


def _parse_state(text):
    """Parse the single-line `drivebase get-state` output."""
    out = {}
    for kv in text.split():
        if "=" in kv:
            k, _, v = kv.partition("=")
            try:
                out[k] = int(v)
            except ValueError:
                pass
    return out


def _ensure_stopped(p):
    """Best-effort: make sure no daemon is alive entering a test."""
    p.sendCommand("drivebase stop", timeout=5)
    # Give the kernel close-cleanup + sem_post a beat to settle.
    time.sleep(0.3)


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
    """

    _ensure_stopped(p)
    p.sendCommand("drivebase start", timeout=8)
    time.sleep(1.0)

    out = p.sendCommand("drivebase get-state", timeout=8)
    state = _parse_state(out)
    # `done=1` in the idle state, `cmd=0` (no active command).  The
    # tick counter must be advancing at ~200 Hz — anything past a
    # handful proves the RT pthread reached its loop.
    assert state.get("done") == 1, f"unexpected get-state: {out!r}"
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
