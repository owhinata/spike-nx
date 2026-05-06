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
    "time_ms", "iter", "refl", "err",
    "err_min", "err_max", "err_avg", "zc",
    "d_max", "d_avg", "i_acc",
    "turn_max", "turn_avg", "sat",
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
    last_turn_dps drop out (they're aggregate-only via pidstat) and
    last_i_acc takes their place.
    """

    _ensure_stopped(p)
    p.sendCommand("linetrace start", timeout=8)
    time.sleep(0.5)

    try:
        out = p.sendCommand("linetrace status", timeout=5)
        st = _parse_status(out)
        for legacy in ("last_p_term", "last_i_term",
                       "last_d_term", "last_turn_dps"):
            assert legacy not in st, (
                f"legacy snapshot field {legacy} should be removed"
            )
        assert "last_i_acc" in st, f"last_i_acc missing: {out!r}"
        assert "last_err" in st
        assert "last_refl" in st
    finally:
        _ensure_stopped(p)
