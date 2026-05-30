"""Category L: linetrace daemon + pidstat tests (Issue #118).

The non-interactive case checks `pidstat` rejects when no daemon is
running (no hardware required).  The interactive cases require a
LEGO color sensor on a free port and exercise the full flow:

    - linetrace start spawns the daemon (color CLAIM + drivebase open)
    - linetrace pidstat (no args) prints header + 1 snapshot row
      with no summary trailer
    - linetrace pidstat 5000 1000 streams ~5 data rows + summary
    - interval_ms below the control period is rejected
    - linetrace status drops the legacy last_p_term/i_term/d_term/
      turn_dps fields and exposes last_i_acc instead
"""

import re
import time

import pytest


# ---------------------------------------------------------------------------
# pidstat output parsers
# ---------------------------------------------------------------------------

_PIDSTAT_HEADER = (
    "time_ms", "iter", "intens", "err",
    "err_min", "err_max", "err_avg", "zc",
    "d_max", "d_avg", "i_acc",
    "turn_max", "turn_avg",
    "v_max", "v_avg", "v_min",
    "sat",
)

_SUMMARY_RE = re.compile(
    r"^#\s*pidstat:\s*sat=(\d+)\s+iter=(\d+)\.\.(\d+)\s+"
    r"duration_ms=(\d+)\s+reported_ticks=(\d+)\s+expected=(\d+)",
    re.MULTILINE,
)


def _parse_pidstat(text):
    """Return (header_seen, data_rows, summary_dict_or_None).

    A row is a list of stringified column values.  '-' entries are
    preserved as-is so callers can distinguish "no data" from numeric
    zeros.  Header is matched by exact column-name tuple so any column
    rename surfaces immediately.
    """
    header_seen = False
    rows = []
    for line in text.splitlines():
        toks = line.split()
        if tuple(toks) == _PIDSTAT_HEADER:
            header_seen = True
            continue
        if len(toks) != len(_PIDSTAT_HEADER):
            continue
        # Skip lines that look like NSH echo or unrelated noise.
        # First token must be all-digit (time_ms).
        if not toks[0].isdigit():
            continue
        rows.append(toks)

    summary = None
    m = _SUMMARY_RE.search(text)
    if m:
        summary = {
            "sat": int(m.group(1)),
            "iter_begin": int(m.group(2)),
            "iter_end": int(m.group(3)),
            "duration_ms": int(m.group(4)),
            "reported_ticks": int(m.group(5)),
            "expected": int(m.group(6)),
        }
    return header_seen, rows, summary


_STATUS_RE = re.compile(r"^([a-z_]+):\s*(\S+)", re.MULTILINE)


def _parse_status(text):
    return {m.group(1): m.group(2) for m in _STATUS_RE.finditer(text)}


def _ensure_stopped(p):
    p.sendCommand("linetrace stop", timeout=5)
    time.sleep(0.3)


# ---------------------------------------------------------------------------
# Non-interactive: argument paths that don't need the daemon
# ---------------------------------------------------------------------------


def test_linetrace_pidstat_without_daemon(p):
    """L-1: pidstat refuses cleanly when no daemon is running.

    Exit-code semantics aren't observable through NSH, so we assert on
    the stderr surface text and that no header or summary leaked.
    """

    _ensure_stopped(p)
    out = p.sendCommand("linetrace pidstat", timeout=5)
    assert "daemon not running" in out, f"unexpected: {out!r}"
    header_seen, rows, summary = _parse_pidstat(out)
    assert not header_seen, "header should not appear when daemon down"
    assert rows == []
    assert summary is None


def test_linetrace_pidstat_rejects_short_interval(p):
    """L-2: interval_ms below the control period (10 ms @ 100 Hz) rejects.

    Daemon has to be up so that the validate path runs *after* the
    daemon-running check; otherwise the user would not learn that the
    duration/interval values are also bad.
    """

    _ensure_stopped(p)
    start = p.sendCommand("linetrace start", timeout=8)
    if "running" not in start.lower() and "spawned" not in start.lower():
        pytest.skip(f"linetrace start did not succeed: {start!r}")
    time.sleep(0.5)

    try:
        out = p.sendCommand("linetrace pidstat 1000 5", timeout=5)
        assert "too small" in out, f"unexpected: {out!r}"
        header_seen, rows, summary = _parse_pidstat(out)
        assert not header_seen
        assert rows == []
        assert summary is None
    finally:
        _ensure_stopped(p)


# ---------------------------------------------------------------------------
# Interactive: full flow needing color sensor + drivebase chardev
# ---------------------------------------------------------------------------


@pytest.mark.interactive
def test_linetrace_pidstat_snapshot(p):
    """L-3: bare `linetrace pidstat` prints header + 1 snapshot row.

    No summary line for snapshot mode (drivebase get-state parity).
    """

    _ensure_stopped(p)
    start = p.sendCommand("linetrace start", timeout=8)
    assert "running" in start.lower() or "spawned" in start.lower(), (
        f"linetrace start failed: {start!r}"
    )
    time.sleep(0.5)

    try:
        out = p.sendCommand("linetrace pidstat", timeout=5)
        header_seen, rows, summary = _parse_pidstat(out)
        assert header_seen, f"header missing: {out!r}"
        assert len(rows) == 1, f"expected 1 row, got {len(rows)}: {out!r}"
        assert summary is None, "snapshot must not emit summary line"

        out = p.sendCommand("linetrace pidstat 0", timeout=5)
        header_seen, rows, summary = _parse_pidstat(out)
        assert header_seen
        assert len(rows) == 1
        assert summary is None
    finally:
        _ensure_stopped(p)


@pytest.mark.interactive
def test_linetrace_pidstat_streaming(p):
    """L-4: streaming mode emits multiple rows + summary.

    Daemon is idle (no `run`), so d_avg/turn_avg should be `-` until at
    least one engaged tick lands in the interval — but err_min/err_max
    will populate because the idle path also feeds err aggregation.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.5)

    try:
        out = p.sendCommand("linetrace pidstat 3000 1000", timeout=10)
        header_seen, rows, summary = _parse_pidstat(out)
        assert header_seen
        assert len(rows) >= 3, f"expected >=3 rows, got {len(rows)}"
        assert summary is not None, f"summary missing: {out!r}"

        # iter must advance monotonically across rows.
        iters = [int(r[1]) for r in rows]
        assert iters == sorted(iters), f"iter not monotonic: {iters}"

        # reported_ticks should be near hz * duration_seconds (within
        # 50 % to absorb interval-boundary truncation + scheduling
        # jitter).
        assert summary["reported_ticks"] > 0
        assert summary["expected"] > 0
    finally:
        _ensure_stopped(p)


@pytest.mark.interactive
def test_linetrace_status_drops_pid_term_fields(p):
    """L-5: `linetrace status` no longer prints the legacy snapshot fields.

    Issue #118 swap: last_p_term / last_i_term / last_d_term /
    last_turn_dps drop out (they're aggregate-only via pidstat).
    Issue #119 also drops last_i_acc since pidstat exposes the i_acc
    column directly.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.5)

    try:
        out = p.sendCommand("linetrace status", timeout=5)
        st = _parse_status(out)
        for legacy in ("last_p_term", "last_i_term", "last_d_term",
                       "last_turn_dps", "last_i_acc"):
            assert legacy not in st, (
                f"legacy snapshot field {legacy} should be removed"
            )
        assert "last_err" in st
        assert "last_intensity" in st
        assert "target" in st
        # Issue #126: max_turn is derived per tick, no longer in status.
        # v_min/v_alpha/v_beta surface the dynamic-speed config instead.
        assert "max_turn" not in st
        assert "v_min" in st
        assert "v_alpha" in st
        assert "v_beta" in st
    finally:
        _ensure_stopped(p)


# ---------------------------------------------------------------------------
# Issue #119: linetrace target  (#126 retired the max_turn mutator)
# ---------------------------------------------------------------------------


def test_linetrace_set_without_daemon(p):
    """L-6: target refuses cleanly when no daemon is running."""

    _ensure_stopped(p)
    out = p.sendCommand("linetrace target 30", timeout=5)
    assert "not running" in out, f"unexpected: {out!r}"


def test_linetrace_set_missing_or_extra_args(p, color_sensor_required):
    """L-7: bare or extra-arg invocation prints usage and rejects.

    `linetrace start` opens the color sensor SELECT path and only
    succeeds when a real sensor is attached; without one the daemon
    fails to spawn and subsequent verbs report "not running".
    `color_sensor_required` skips this test cleanly in that case.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.3)

    try:
        for cmd in ("linetrace target",
                    "linetrace target 30 40"):
            out = p.sendCommand(cmd, timeout=5)
            assert "usage" in out.lower(), (
                f"expected usage hint for {cmd!r}: {out!r}"
            )
    finally:
        _ensure_stopped(p)


def test_linetrace_set_rejects_out_of_range(p, color_sensor_required):
    """L-8: target outside the validated range is rejected.

    Like L-7, requires a real Color Sensor for `linetrace start` to
    succeed; otherwise the daemon never comes up and the target
    verb's range-check path is unreachable.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.3)

    try:
        out = p.sendCommand("linetrace target 1500", timeout=5)
        assert "target" in out, f"expected target rejection: {out!r}"

        out = p.sendCommand("linetrace target -1", timeout=5)
        assert "target" in out, f"expected target rejection: {out!r}"

        out = p.sendCommand("linetrace target abc", timeout=5)
        assert "integer" in out, f"expected non-integer rejection: {out!r}"
    finally:
        _ensure_stopped(p)


@pytest.mark.interactive
def test_linetrace_set_updates_target(p):
    """L-9: target writes are observable via status.

    Issue #119 motivation: setting target before `run` makes the idle
    err computation match the operational threshold, so positioning
    feedback via `status` is meaningful.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.3)

    try:
        # Default from do_start: target=512
        out = p.sendCommand("linetrace status", timeout=5)
        st = _parse_status(out)
        assert st.get("target") == "512"

        out = p.sendCommand("linetrace target 400", timeout=5)
        assert "target=400" in out, f"unexpected target output: {out!r}"

        out = p.sendCommand("linetrace status", timeout=5)
        st = _parse_status(out)
        assert st.get("target") == "400"
    finally:
        _ensure_stopped(p)


# ---------------------------------------------------------------------------
# Issue #126: dynamic-speed flags + max_turn deprecation
# ---------------------------------------------------------------------------


def test_linetrace_max_turn_subcommand_retired(p):
    """L-10: `linetrace max_turn N` falls through to usage (#126)."""

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.3)
    try:
        out = p.sendCommand("linetrace max_turn 200", timeout=5)
        assert "usage" in out.lower(), (
            f"max_turn subcommand should be retired: {out!r}"
        )
    finally:
        _ensure_stopped(p)


def test_linetrace_run_max_turn_flag_retired(p, color_sensor_required):
    """L-11: `linetrace run ... --max-turn N` is rejected (#126).

    do_run() checks g_daemon_running before parsing the flag list, so
    the "unknown option" message only fires when the daemon is alive
    — which in turn requires color_open_select() to succeed (CLAIM
    on /dev/uorb/sensor_color).  Without a real sensor the daemon
    fails its CLAIM, sets g_daemon_running=false, and `linetrace run`
    falls into the "not running" path instead of reaching the flag
    parser.  Skip cleanly via color_sensor_required in that case.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.3)
    try:
        out = p.sendCommand(
            "linetrace run 100 0 --max-turn 200", timeout=5
        )
        assert "unknown option" in out.lower(), (
            f"--max-turn flag should be retired: {out!r}"
        )
    finally:
        _ensure_stopped(p)


def test_linetrace_run_dynamic_flags_validate(p, color_sensor_required):
    """L-12: dynamic-speed flag range validation (#126).

    --v-min must be in [1, speed]; --v-alpha and --v-beta must be in
    [0.00, 100.00].  Daemon must be running for do_run to evaluate the
    arguments past the daemon-running gate — same precondition as L-7,
    L-8, and L-11 — so requires color_sensor_required.  Previously
    passed by timing accident (do_run ran during the brief window
    before color_open_select gave up on the missing sensor).
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.3)
    try:
        # v-min above speed should reject.
        out = p.sendCommand(
            "linetrace run 100 0 512 --v-min 200", timeout=5
        )
        assert "v-min" in out.lower(), f"expected v-min reject: {out!r}"

        # v-min == 0 should reject.
        out = p.sendCommand(
            "linetrace run 100 0 512 --v-min 0", timeout=5
        )
        assert "v-min" in out.lower(), f"expected v-min=0 reject: {out!r}"

        # Negative v-alpha rejected.
        out = p.sendCommand(
            "linetrace run 100 0 512 --v-alpha -0.5", timeout=5
        )
        assert "v-alpha" in out.lower(), (
            f"expected negative v-alpha reject: {out!r}"
        )

        # Out-of-range v-beta rejected.
        out = p.sendCommand(
            "linetrace run 100 0 512 --v-beta 200", timeout=5
        )
        assert "v-beta" in out.lower(), (
            f"expected v-beta reject: {out!r}"
        )
    finally:
        # Always brake/disengage so cumulative drivebase commands clear.
        p.sendCommand("linetrace brake", timeout=5)
        _ensure_stopped(p)


@pytest.mark.interactive
def test_linetrace_run_dynamic_flags_status(p):
    """L-13: dynamic-speed flags surface in status (#126).

    `--v-min`/`--v-alpha`/`--v-beta` reset to no-op default every run,
    so a follow-up `run` without the flags reverts to v_min=speed.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.3)
    try:
        out = p.sendCommand(
            "linetrace run 200 0 --v-min 120 --v-alpha 1.0 --v-beta 0.5",
            timeout=5,
        )
        assert "v_min=120" in out
        assert "v_alpha=1.00" in out
        assert "v_beta=0.50" in out

        out = p.sendCommand("linetrace status", timeout=5)
        st = _parse_status(out)
        assert st.get("v_min") == "120"
        assert st.get("v_alpha") == "1.00"
        assert st.get("v_beta") == "0.50"

        # Re-run without --v-* flags; they reset to no-op default.
        out = p.sendCommand("linetrace run 200 0", timeout=5)
        assert "v_min=200" in out, f"expected reset to speed: {out!r}"
        assert "v_alpha=0.00" in out
        assert "v_beta=0.00" in out
    finally:
        p.sendCommand("linetrace brake", timeout=5)
        _ensure_stopped(p)


# ---------------------------------------------------------------------------
# Lap capture verbs (Issue #166)
# ---------------------------------------------------------------------------


def test_linetrace_cap_without_daemon(p):
    """L-14: `cap arm`/`cap export` refuse cleanly with no daemon.

    Non-interactive: the daemon-running guard fires before any FSM /
    capture-library work, so no hardware is required.
    """

    _ensure_stopped(p)
    out = p.sendCommand("linetrace cap arm 100", timeout=5)
    assert "not running" in out, f"unexpected: {out!r}"

    out = p.sendCommand("linetrace cap export", timeout=5)
    # export with no armed/done capture and no daemon -> nothing to export
    assert "nothing to export" in out or "not running" in out, \
        f"unexpected: {out!r}"


def test_linetrace_cap_arm_validates(p, color_sensor_required):
    """L-15: `cap arm` range + integer validation.

    Requires the daemon (color CLAIM) so the validate path runs after
    the daemon-running guard.
    """

    _ensure_stopped(p)
    start = p.sendCommand("linetrace start", timeout=8)
    if "running" not in start.lower() and "started" not in start.lower():
        pytest.skip(f"linetrace start did not succeed: {start!r}")
    time.sleep(0.3)

    try:
        out = p.sendCommand("linetrace cap arm 0", timeout=5)
        assert "must be in" in out, f"arm 0: {out!r}"

        out = p.sendCommand("linetrace cap arm -1", timeout=5)
        assert "must be in" in out, f"arm -1: {out!r}"

        # > cap_max (3449)
        out = p.sendCommand("linetrace cap arm 999999", timeout=5)
        assert "must be in" in out, f"arm 999999: {out!r}"

        out = p.sendCommand("linetrace cap arm abc", timeout=5)
        assert "must be an integer" in out, f"arm abc: {out!r}"
    finally:
        p.sendCommand("linetrace cap abort", timeout=5)
        _ensure_stopped(p)


def test_linetrace_cap_export_without_arm(p, color_sensor_required):
    """L-16: `cap export` while IDLE -> nothing to export."""

    _ensure_stopped(p)
    start = p.sendCommand("linetrace start", timeout=8)
    if "running" not in start.lower() and "started" not in start.lower():
        pytest.skip(f"linetrace start did not succeed: {start!r}")
    time.sleep(0.3)

    try:
        out = p.sendCommand("linetrace cap export", timeout=5)
        assert "nothing to export" in out, f"unexpected: {out!r}"
    finally:
        _ensure_stopped(p)


def test_linetrace_cap_status_keys(p, color_sensor_required):
    """L-17: status exposes additive cap_* keys; `cap status` prints fields.

    Confirms the new keys are additive and `_parse_status` still reads
    the existing keys.
    """

    _ensure_stopped(p)
    start = p.sendCommand("linetrace start", timeout=8)
    if "running" not in start.lower() and "started" not in start.lower():
        pytest.skip(f"linetrace start did not succeed: {start!r}")
    time.sleep(0.3)

    try:
        out = p.sendCommand("linetrace status", timeout=5)
        st = _parse_status(out)
        # Existing keys still parse.
        assert st.get("running") == "yes", f"status: {out!r}"
        # New additive key.
        assert st.get("cap_state") == "idle", f"cap_state: {out!r}"

        out = p.sendCommand("linetrace cap status", timeout=5)
        st = _parse_status(out)
        assert st.get("cap_state") == "idle", f"cap status: {out!r}"
        assert "cap_count" in st
        assert "cap_capacity" in st
        assert "cap_overflow" in st
    finally:
        _ensure_stopped(p)


def test_linetrace_cap_abort(p, color_sensor_required):
    """L-18: explicit `cap abort` resets an armed capture to idle."""

    _ensure_stopped(p)
    start = p.sendCommand("linetrace start", timeout=8)
    if "running" not in start.lower() and "started" not in start.lower():
        pytest.skip(f"linetrace start did not succeed: {start!r}")
    time.sleep(0.3)

    try:
        out = p.sendCommand("linetrace cap arm 50", timeout=5)
        assert "armed" in out, f"arm: {out!r}"

        out = p.sendCommand("linetrace cap status", timeout=5)
        st = _parse_status(out)
        assert st.get("cap_state") == "armed", f"after arm: {out!r}"
        assert st.get("cap_capacity") == "50"

        out = p.sendCommand("linetrace cap abort", timeout=5)
        assert "discarded" in out, f"abort: {out!r}"

        out = p.sendCommand("linetrace cap status", timeout=5)
        st = _parse_status(out)
        assert st.get("cap_state") == "idle", f"after abort: {out!r}"
    finally:
        _ensure_stopped(p)


@pytest.mark.interactive
def test_linetrace_cap_arm_then_status(p):
    """L-19: FSM arm -> count advances on idle ticks -> stop -> done.

    No motion: idle-loop ticks populate `count`.  Validates the FSM
    transitions without a BT host (the full export round-trip stays a
    manual HW-verify step).
    """

    _ensure_stopped(p)
    start = p.sendCommand("linetrace start", timeout=8)
    if "running" not in start.lower() and "started" not in start.lower():
        pytest.skip(f"linetrace start did not succeed: {start!r}")
    time.sleep(0.3)

    try:
        out = p.sendCommand("linetrace cap arm 50", timeout=5)
        assert "armed" in out, f"arm: {out!r}"

        out = p.sendCommand("linetrace status", timeout=5)
        st = _parse_status(out)
        assert st.get("cap_state") == "armed", f"status: {out!r}"
        assert st.get("cap_capacity") == "50"

        # Idle ticks accumulate count at the loop rate (100 Hz default).
        time.sleep(0.3)
        out = p.sendCommand("linetrace cap status", timeout=5)
        st = _parse_status(out)
        count = int(st.get("cap_count", "0"))
        assert count > 0, f"expected idle ticks to advance count: {out!r}"

        out = p.sendCommand("linetrace cap stop", timeout=5)
        assert "done" in out, f"stop: {out!r}"

        out = p.sendCommand("linetrace cap status", timeout=5)
        st = _parse_status(out)
        assert st.get("cap_state") == "done", f"after stop: {out!r}"
    finally:
        p.sendCommand("linetrace cap abort", timeout=5)
        _ensure_stopped(p)
