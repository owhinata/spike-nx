"""Category D-SYSID: drivebase _sysid CLI smoke tests (Issue #152 Step 6.4).

Every test here is `interactive` — they drive real motors on the SPIKE
Prime Hub, on the ground (free-spin underestimates kS and ruins kA), and
must not race the production daemon.  The non-interactive smoke suite
covers the dispatch and arg-validation surfaces without spinning motors.
"""

import re

import pytest


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


_KS_LINE_RE = re.compile(
    r"^kS_measured:\s*L=(?P<l>-?\d+)\s+R=(?P<r>-?\d+).*max=(?P<m>-?\d+)",
    re.MULTILINE,
)
_KS_NOMINAL_RE = re.compile(
    r"^kS_nominal\s*:\s*(?P<n>-?\d+)", re.MULTILINE
)
_KV_AVG_RE = re.compile(
    r"^kV_measured:.*avg=(?P<v>-?\d+)", re.MULTILINE
)
_KA_VSTEADY_RE = re.compile(
    r"^v_steady_avg\s*=\s*(?P<v>-?\d+)", re.MULTILINE
)
_KA_ESTIMATE_RE = re.compile(
    r"^kA_estimate\s*=\s*(?P<k>-?\d+)", re.MULTILINE
)
_VBAT_RE = re.compile(r"^vbat=(?P<mv>-?\d+)\s+mV", re.MULTILINE)
_VBAT_COMPENSATION_RE = re.compile(
    r"compensation_factor=(?P<c>-?\d+)/1000.*"
    r"normalisation_factor=(?P<n>-?\d+)/1000",
    re.MULTILINE,
)


def _ensure_stopped(p, timeout=10.0):
    """`drivebase stop` until it reports either stopped or not running.

    SysId verbs reject with -EBUSY while the daemon is attached, so the
    test must guarantee daemon_attached=0 before any sysid command.
    Mirrors the helper in test_drivebase.py without importing it (these
    files run independently under pytest discovery).
    """
    import time
    deadline = time.time() + timeout
    while time.time() < deadline:
        out = p.sendCommand("drivebase stop", timeout=5)
        if "stopped" in out or "not running" in out or "Unknown error 11" in out:
            return
        time.sleep(0.5)


# ---------------------------------------------------------------------------
# Non-interactive: dispatch + arg-validation (no motors required)
# ---------------------------------------------------------------------------


def test_sysid_usage_lists_all_verbs(p):
    """D-SYSID-1: bare `_sysid` prints usage with the 4 documented verbs.

    The dispatch should not touch motor hardware when no subverb is
    given, so this runs without the `interactive` mark.  Also verifies
    the help string mentions the ground-only contract from Plan D8.
    """
    out = p.sendCommand("drivebase _sysid", timeout=3)
    for verb in ("ramp-ks", "ramp-kv", "ramp-ka", "vbat"):
        assert verb in out, f"_sysid usage missing verb '{verb}': {out!r}"
    assert "level ground" in out.lower() or "wheels-up" in out.lower(), (
        f"_sysid usage missing ground-only safety note: {out!r}"
    )


def test_sysid_rejects_unknown_subverb(p):
    """D-SYSID-2: unknown subverb prints an explicit error, no crash."""
    out = p.sendCommand("drivebase _sysid frobnicate", timeout=3)
    assert "unknown" in out.lower(), f"silent failure on bad verb: {out!r}"


def test_sysid_usage_documents_pivot_head_axis(p):
    """D-SYSID-PIVOT-1: bare `_sysid` usage documents the pivot/head axis.

    Phase 7 (#158): pivot mode identifies the HEADING axis (ff_head_*)
    via an in-place rotation.  The usage must surface both the keyword
    and that it targets ff_head_*, and the safety note must mention the
    in-place rotation clearance (distinct from the 2 m straight one).
    No motors are driven for a bare usage print.
    """
    out = p.sendCommand("drivebase _sysid", timeout=3)
    assert "pivot" in out.lower(), f"_sysid usage missing pivot: {out!r}"
    assert "ff_head" in out.lower(), (
        f"_sysid usage missing ff_head axis note: {out!r}"
    )
    assert "in place" in out.lower() or "in-place" in out.lower(), (
        f"_sysid usage missing in-place rotation clearance note: {out!r}"
    )


def test_sysid_ramp_kv_pivot_usage_targets_head(p):
    """D-SYSID-PIVOT-2: `ramp-kv pivot` with no numeric args -> usage.

    Confirms the trailing `pivot` keyword is stripped before positional
    parsing (so `ramp-kv pivot` is treated as "missing args", not a
    spurious argv) and that the per-verb usage advertises ff_head_kV.
    Insufficient args means the usage prints before any motor drive, so
    this stays non-interactive.
    """
    out = p.sendCommand("drivebase _sysid ramp-kv pivot", timeout=3)
    assert "usage" in out.lower(), (
        f"ramp-kv pivot (no args) should print usage: {out!r}"
    )
    assert "ff_head_kv" in out.lower(), (
        f"ramp-kv usage missing ff_head_kV target for pivot: {out!r}"
    )


def test_sysid_vbat_round_trip(p):
    """D-SYSID-3: `_sysid vbat` reads /dev/bat0 even without daemon.

    Verifies the independent BATIOC_VOLTAGE path used by SysId works
    when the production daemon (and its drivebase_battery.c atomic) is
    not running.  Bound check: realistic Hub voltage is 6000-8400 mV.
    Also confirms the compensation/normalisation factor pair labels
    are present (Codex Round 2 NIT — both directions are useful so
    operators don't have to mentally invert).
    """
    _ensure_stopped(p)
    out = p.sendCommand("drivebase _sysid vbat", timeout=3)
    m = _VBAT_RE.search(out)
    assert m, f"_sysid vbat output missing vbat= line: {out!r}"
    mv = int(m.group("mv"))
    assert 5000 < mv < 9000, f"implausible vbat={mv} mV: {out!r}"

    mf = _VBAT_COMPENSATION_RE.search(out)
    assert mf, (
        f"_sysid vbat missing compensation/normalisation labels: {out!r}"
    )
    comp = int(mf.group("c"))
    norm = int(mf.group("n"))
    # The two factors must be inverses (within rounding): comp * norm ≈ 1e6
    product = comp * norm
    assert 990_000 < product < 1_010_000, (
        f"compensation_factor={comp} and normalisation_factor={norm} "
        f"are not inverses (product={product}): {out!r}"
    )


# ---------------------------------------------------------------------------
# Interactive: real motors on the ground
# ---------------------------------------------------------------------------


@pytest.mark.interactive
@pytest.mark.slow
def test_sysid_ramp_ks_finds_breakaway(p):
    """D-SYSID-INT-1: ramp-ks finds the breakaway duty for both wheels.

    Plan D8 acceptance: kS in [400, 1200] (.01% duty) — anything outside
    that window flags a measurement issue (wheels stuck / loaded
    incorrectly / vbat way off).  The verb prints both a raw and a
    nominal-normalised value; both should fall inside.
    """
    p.waitUser(
        "Place the drivebase on a flat surface with ~2 m clear straight "
        "ahead.  No motors freely spinning — wheels touching ground.  "
        "Press Enter to start ramp-ks (will sweep duty 0 -> 25% in 50-"
        "count increments, ~5 seconds)."
    )
    _ensure_stopped(p)

    out = p.sendCommand(
        "drivebase _sysid ramp-ks 50 2500 200", timeout=15
    )

    m_measured = _KS_LINE_RE.search(out)
    assert m_measured, f"_sysid ramp-ks missing kS_measured line: {out!r}"
    kS_L = int(m_measured.group("l"))
    kS_R = int(m_measured.group("r"))
    kS_max = int(m_measured.group("m"))
    assert 400 <= kS_max <= 1200, (
        f"kS_max={kS_max} outside plan range [400, 1200] "
        f"(L={kS_L} R={kS_R}): {out!r}"
    )

    m_nominal = _KS_NOMINAL_RE.search(out)
    assert m_nominal, f"_sysid ramp-ks missing kS_nominal line: {out!r}"
    kS_nominal = int(m_nominal.group("n"))
    assert 300 <= kS_nominal <= 1400, (
        f"kS_nominal={kS_nominal} suspicious (raw_max={kS_max}): {out!r}"
    )


@pytest.mark.interactive
@pytest.mark.slow
def test_sysid_ramp_ks_pivot_finds_breakaway(p):
    """D-SYSID-INT-PIVOT-1: pivot ramp-ks finds breakaway for both wheels.

    Phase 7 (#158): in pivot the left wheel runs backward (vL < 0), so
    breakaway must be detected on the magnitude.  This test guards
    against a sign regression that would make the left wheel never
    register as "moving".  The PIVOT MODE banner must be present so the
    operator knows the chassis spins in place.
    """
    p.waitUser(
        "Place the drivebase on a flat surface with a clear CIRCLE to "
        "rotate in place (it will NOT drive straight).  Wheels touching "
        "ground.  Press Enter to start pivot ramp-ks (~5 seconds)."
    )
    _ensure_stopped(p)

    out = p.sendCommand(
        "drivebase _sysid ramp-ks 50 2500 200 pivot", timeout=15
    )
    assert "PIVOT" in out, f"pivot ramp-ks missing PIVOT banner: {out!r}"

    m_measured = _KS_LINE_RE.search(out)
    assert m_measured, f"pivot ramp-ks missing kS_measured line: {out!r}"
    kS_L = int(m_measured.group("l"))
    kS_R = int(m_measured.group("r"))
    # Both wheels must register breakaway (sign-agnostic detection).  The
    # reported value is the positive ramp duty at which each broke away.
    assert kS_L > 0 and kS_R > 0, (
        f"pivot ramp-ks failed to find breakaway on a wheel "
        f"(L={kS_L} R={kS_R}) — sign handling regression?: {out!r}"
    )


@pytest.mark.interactive
@pytest.mark.slow
def test_sysid_ramp_kv_pivot_positive_kv(p):
    """D-SYSID-INT-PIVOT-2: pivot ramp-kv yields a POSITIVE heading kV.

    The left wheel is driven with -duty in pivot, so the fit must see
    the actual per-side applied duty (uL_buf=-d) — otherwise the fitted
    kV flips sign.  This is the decisive sign/units check: a plausible
    positive kV (.01% per deg/s) means uL/uR buffer separation is
    correct and the per-wheel fit maps onto the heading axis.
    """
    p.waitUser(
        "Place the drivebase on a flat surface with a clear CIRCLE to "
        "rotate in place.  Wheels touching ground.  Press Enter to start "
        "pivot ramp-kv (~6 seconds)."
    )
    _ensure_stopped(p)

    # kS_subtract=700 is the Phase 6.2 straight value as a placeholder;
    # the operator should ideally feed the pivot ramp-ks output.  A wrong
    # kS only biases the magnitude, not the sign this test checks.
    out = p.sendCommand(
        "drivebase _sysid ramp-kv 6000 3300 700 pivot", timeout=20
    )
    assert "PIVOT" in out, f"pivot ramp-kv missing PIVOT banner: {out!r}"

    m_avg = _KV_AVG_RE.search(out)
    assert m_avg, f"pivot ramp-kv missing kV_measured avg line: {out!r}"
    kV_avg = int(m_avg.group("v"))
    assert kV_avg > 0, (
        f"pivot ramp-kv produced non-positive kV_avg={kV_avg} — uL/uR "
        f"sign handling regression?: {out!r}"
    )
    assert 1 <= kV_avg <= 100, (
        f"pivot ramp-kv kV_avg={kV_avg} implausible (.01%/deg/s): {out!r}"
    )

    m_app = re.search(r"To apply: set\s+(\w+)", out)
    assert m_app and m_app.group(1) == "ff_head_kV", (
        f"pivot ramp-kv should target ff_head_kV, got "
        f"{m_app.group(1) if m_app else None}: {out!r}"
    )


@pytest.mark.interactive
@pytest.mark.slow
def test_sysid_ramp_ka_pivot_magnitude_average(p):
    """D-SYSID-INT-PIVOT-3: pivot ramp-ka uses the magnitude average.

    In pivot vL and vR are opposite-signed; a plain (vL+vR)/2 would be
    ~0 and the 63.2% rise detection would fail ("failed to find 63%
    rise").  The implementation averages magnitudes, so v_steady_avg
    must come out clearly positive and a kA estimate must be produced.
    Also confirms the apply line targets ff_head_kA.
    """
    p.waitUser(
        "Place the drivebase on a flat surface with a clear CIRCLE to "
        "rotate in place.  Wheels touching ground.  Press Enter to start "
        "pivot ramp-ka (~2 seconds)."
    )
    _ensure_stopped(p)

    out = p.sendCommand(
        "drivebase _sysid ramp-ka 6000 1500 10 pivot", timeout=12
    )
    assert "PIVOT" in out, f"pivot ramp-ka missing PIVOT banner: {out!r}"

    m_v = _KA_VSTEADY_RE.search(out)
    assert m_v, (
        f"pivot ramp-ka missing v_steady_avg (63% rise likely failed — "
        f"signed average regression?): {out!r}"
    )
    v_steady = int(m_v.group("v"))
    assert v_steady > 0, (
        f"pivot ramp-ka v_steady_avg={v_steady} not positive — magnitude "
        f"average regression?: {out!r}"
    )

    m_k = _KA_ESTIMATE_RE.search(out)
    assert m_k, f"pivot ramp-ka missing kA_estimate line: {out!r}"

    m_app = re.search(r"To apply: set\s+(\w+)", out)
    assert m_app and m_app.group(1) == "ff_head_kA", (
        f"pivot ramp-ka should target ff_head_kA, got "
        f"{m_app.group(1) if m_app else None}: {out!r}"
    )
