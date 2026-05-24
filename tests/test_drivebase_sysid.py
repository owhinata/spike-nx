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
