"""Category G: drivebase gyro-locked heading PID (Issue #148 / #157).

These tests cover the user-visible surface of the `set-gyro` ioctl and
the published `status` fields that surround it (use_gyro / use_gyro_
latched / last_set_gyro_rc / imu_present).  All non-interactive cases
run with the daemon attached but no motor motion required.

Issue #157 removed the old `1d` mode: `3d` (fused-quaternion forward
projection — what `1d` used to compute) is now the sole gyro heading
mode, and `set-gyro 1d` is rejected at the CLI.

Interactive cases (mid-motion EBUSY, cold-boot un-calibrated path, cfg
restart, 51° tilted bench accuracy) live behind `@pytest.mark.inter
active` / `@pytest.mark.slow` so CI only runs the smoke set without
hardware that's specific to the gyro-injection plan.
"""

import re
import time

import pytest


_STATUS_RE = re.compile(r"^\s*([A-Za-z_]+)\s*=\s*(-?\d+)\s*$", re.MULTILINE)


def _parse_status(text):
    return {m.group(1): int(m.group(2)) for m in _STATUS_RE.finditer(text)}


def _ensure_stopped(p, timeout=15.0):
    """Same pattern as test_drivebase._ensure_stopped — copied locally so
    the gyro test file stays standalone.
    """
    deadline = time.time() + timeout
    last_stop = 0.0
    while time.time() < deadline:
        if time.time() - last_stop > 3.0:
            p.sendCommand("drivebase stop", timeout=5)
            last_stop = time.time()
        time.sleep(0.3)
        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        if st.get("daemon_attached") == 0:
            return


def _start_with_imu(p, settle_s=5.0):
    """Start the daemon and wait long enough for Madgwick to converge.

    The 200 ms bias-idle window plus a few hundred ms of margin reliably
    leaves the IMU `calibrated=1` by the time the test issues set-gyro.
    """
    _ensure_stopped(p)
    start = p.sendCommand("drivebase start", timeout=8)
    assert "started" in start, f"start failed: {start!r}"
    time.sleep(settle_s)


# ---------------------------------------------------------------------------
# Non-interactive: status surface around set-gyro
# ---------------------------------------------------------------------------


def test_g1_set_gyro_3d_status_fields(p):
    """G-1: `drivebase set-gyro 3d` makes the requested mode visible.

    After issuing the ioctl with the daemon idle the published status
    must report:
      - imu_present == 1            (IMU opened at start)
      - use_gyro_requested == 2     (the requested mode = 3D)
      - use_gyro_latched   == 0     (no motion ⇒ nothing to latch yet)
      - last_set_gyro_rc   == 0     (ioctl accepted)
    `use_gyro` (legacy union alias) must mirror `use_gyro_requested` so
    pre-Phase-3b consumers stay source-compatible.
    """
    try:
        _start_with_imu(p)
        out = p.sendCommand("drivebase set-gyro 3d", timeout=5)
        # CLI prints nothing on success; absence of "SET_USE_GYRO" error
        # is the asynchronous-ioctl success path (env queued, daemon
        # dispatches in <1 tick).
        assert "SET_USE_GYRO" not in out, (
            f"set-gyro 3d should not error: {out!r}"
        )

        # Give the daemon idle loop (50 ms cadence) two wakes to refresh
        # the published status snapshot.
        time.sleep(0.2)

        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        assert st["imu_present"] == 1, st
        assert st["use_gyro_requested"] == 2, st
        assert st["use_gyro"] == 2, st        # union alias mirror
        assert st["use_gyro_latched"] == 0, st
        assert st["last_set_gyro_rc"] == 0, st
    finally:
        p.sendCommand("drivebase stop", timeout=8)
        time.sleep(0.3)


def test_g2_set_gyro_1d_rejected(p):
    """G-2: `drivebase set-gyro 1d` is rejected — 1D was removed (#157).

    The CLI no longer accepts `1d`; it prints a migration hint pointing
    at `3d` and bails *before* issuing the ioctl, so the daemon's
    requested mode and last_set_gyro_rc are left untouched.
    """
    try:
        _start_with_imu(p)
        # Baseline: capture whatever the boot cfg left requested at (the
        # compiled default is now 3D, #157), so we assert *unchanged*
        # rather than a fixed value.
        st_before = _parse_status(p.sendCommand("drivebase status", timeout=5))
        req_before = st_before["use_gyro_requested"]
        rc_before = st_before["last_set_gyro_rc"]

        out = p.sendCommand("drivebase set-gyro 1d", timeout=5)
        # CLI rejects with a removal hint and does not send the ioctl.
        assert "removed" in out, (
            f"set-gyro 1d should print a removal hint: {out!r}"
        )
        time.sleep(0.2)

        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        # CLI bailed before the ioctl — requested/latched/rc all unchanged.
        assert st["use_gyro_requested"] == req_before, st
        assert st["use_gyro_latched"]   == 0, st
        assert st["last_set_gyro_rc"] == rc_before, st
    finally:
        p.sendCommand("drivebase stop", timeout=8)
        time.sleep(0.3)


# ---------------------------------------------------------------------------
# Interactive: manual rotation publishes through gyro origin
# ---------------------------------------------------------------------------


@pytest.mark.interactive
def test_g1b_manual_rotation_publish(p):
    """G-1b: hand-rotate the Hub 90° and confirm angle_mdeg tracks.

    The publish overwrite (drivebase_daemon.c) folds the IMU heading
    into `get-state.angle_mdeg` whenever `use_gyro_requested == 3D`
    and a valid origin is held — including when no motion is in
    flight.  This case proves the manual-spin path: set-gyro 3d →
    reset 0 0 → rotate by hand → angle_mdeg ≈ 90000.
    """
    try:
        _start_with_imu(p)
        p.sendCommand("drivebase set-gyro 3d", timeout=5)
        time.sleep(0.3)
        p.sendCommand("drivebase reset 0 0", timeout=5)
        time.sleep(0.3)

        p.waitUser("Hand-rotate the Hub +90 deg (CCW viewed from above)")

        out = p.sendCommand("drivebase get-state", timeout=5)
        m = re.search(r"^\s*(\d+)\s+\S+\s+\S+\s+(-?\d+)\s+", out, re.MULTILINE)
        assert m, f"could not parse get-state row: {out!r}"
        angle_mdeg = int(m.group(2))
        assert 80000 <= angle_mdeg <= 100000, (
            f"manual 90 deg should land in [80, 100]k mdeg, got {angle_mdeg}"
        )
    finally:
        p.sendCommand("drivebase stop", timeout=8)
        time.sleep(0.3)


@pytest.mark.interactive
def test_g1_5_reset_nonzero_origin(p):
    """G-1.5: `reset 0 90` baselines publish at +90 deg, even under gyro.

    The origin baseline formula keeps `publish_rel = raw - gyro_origin`
    pointing at the user-supplied angle, so an immediate get-state
    after `reset 0 90` must read ≈ 90000 — not 0, and not raw IMU yaw.
    """
    try:
        _start_with_imu(p)
        p.sendCommand("drivebase set-gyro 3d", timeout=5)
        time.sleep(0.3)
        p.sendCommand("drivebase reset 0 90", timeout=5)
        time.sleep(0.3)

        out = p.sendCommand("drivebase get-state", timeout=5)
        m = re.search(r"^\s*(\d+)\s+\S+\s+\S+\s+(-?\d+)\s+", out, re.MULTILINE)
        assert m, f"could not parse get-state row: {out!r}"
        angle_mdeg = int(m.group(2))
        assert 85000 <= angle_mdeg <= 95000, (
            f"reset 0 90 should keep publish at 90 deg, got {angle_mdeg}"
        )
    finally:
        p.sendCommand("drivebase stop", timeout=8)
        time.sleep(0.3)


@pytest.mark.interactive
def test_g3_set_gyro_busy_mid_motion(p):
    """G-3: mid-motion `set-gyro` returns -EBUSY without disturbing latched.

    Issue a long `forever` drive, then attempt set-gyro 3d.  The daemon
    rejects with -EBUSY (status.last_set_gyro_rc == -16); requested
    stays at its previous value.  After `stop-motion` the next set-gyro
    succeeds with rc=0.
    """
    try:
        _start_with_imu(p)
        # Boot cfg may already arm a mode (compiled default is 3D, #157);
        # capture it so the mid-motion EBUSY assertion checks *unchanged*.
        req0 = _parse_status(
            p.sendCommand("drivebase status", timeout=5))["use_gyro_requested"]
        p.sendCommand("drivebase forever 0 60", timeout=5)
        time.sleep(0.3)

        p.sendCommand("drivebase set-gyro 3d", timeout=5)
        time.sleep(0.2)
        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        assert st["last_set_gyro_rc"] == -16, st   # -EBUSY
        assert st["use_gyro_requested"] == req0, st

        p.sendCommand("drivebase stop-motion coast", timeout=5)
        time.sleep(0.5)

        p.sendCommand("drivebase set-gyro 3d", timeout=5)
        time.sleep(0.2)
        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        assert st["last_set_gyro_rc"] == 0, st
        assert st["use_gyro_requested"] == 2, st
    finally:
        p.sendCommand("drivebase stop-motion coast", timeout=5)
        p.sendCommand("drivebase stop", timeout=8)
        time.sleep(0.3)


@pytest.mark.interactive
def test_g4_uncalibrated_falls_back_to_encoder(p):
    """G-4: cold-boot before Madgwick converges keeps latched at NONE.

    Right after `drivebase start` the IMU is not yet calibrated; even
    if the user immediately requests 3D, the next command-start latch
    sees `gyro_origin_capturable() == false` and locks `latched = NONE`.
    Encoder publish stays in effect for that motion.  This proves the
    safety guard so a stale-data activation never reaches the PID input.

    Race window: Madgwick `calibrated=1` after ~200 ms of stillness
    *plus* the time for the daemon to reach IMU open (motor init +
    mode select + ODR force) — empirically ~300-400 ms post-start.
    Set-gyro must race ahead of that, so this test issues it within
    100 ms of start.  If your hardware happens to converge faster,
    `use_gyro_latched` may flip to 1 — the test then reports the actual
    value via the assertion message so you can decide whether the race
    window has tightened enough to retire this case.
    """
    try:
        _ensure_stopped(p)
        p.sendCommand("drivebase start", timeout=8)
        # Race the calibration window — < 200 ms so Madgwick's stillness
        # idle has not yet declared calibrated=1.
        time.sleep(0.1)
        p.sendCommand("drivebase set-gyro 3d", timeout=5)
        # Immediately issue a short motion so latch fires now.
        p.sendCommand("drivebase straight 100 100 brake", timeout=5)
        time.sleep(0.2)
        st = _parse_status(p.sendCommand("drivebase status", timeout=5))
        # latched must be 0 (NONE) — capturable was false at command start.
        assert st["use_gyro_latched"] == 0, (
            f"expected latched=0 during race-with-calibration window; "
            f"if this fires repeatedly, daemon init may now run faster "
            f"than the 100 ms race window: {st}"
        )
    finally:
        p.sendCommand("drivebase stop-motion coast", timeout=5)
        p.sendCommand("drivebase stop", timeout=8)
        time.sleep(0.3)


@pytest.mark.slow
@pytest.mark.interactive
def test_g5_cfg_boot_time_mode(p):
    """G-5: cfg `use_gyro_plus1=3` activates 3D at boot.

    Requires:
      1. /mnt/flash/drivebase.cfg has `use_gyro_plus1 = 3`
         (a legacy `= 2` is auto-aliased to 3 with a warning, #157)
      2. test_g5 reboots the Hub so config is re-loaded

    After reboot, `drivebase status` reports use_gyro_requested == 2
    even before any user issues set-gyro.  The first command after
    Madgwick converges should latch.
    """
    p.reboot(timeout=20)
    time.sleep(5.0)   # Madgwick converge after rcS auto-start

    st = _parse_status(p.sendCommand("drivebase status", timeout=5))
    if st.get("daemon_attached") == 0:
        pytest.skip("rcS did not auto-start drivebase — skip cfg gate test")
    assert st["use_gyro_requested"] == 2, st
    # latched not asserted: depends on whether rcS issued any command.


# ---------------------------------------------------------------------------
# Bench acceptance (51° tilted): turn / straight accuracy comparison
# ---------------------------------------------------------------------------


@pytest.mark.interactive
@pytest.mark.slow
def test_g6_turn_accuracy_tilted_bench(p):
    """G-6: encoder vs gyro residual after `turn 360 x 3` on a 51° bench.

    Pre-condition: SPIKE Prime Hub mounted at ~51 deg tilt, motors L/R
    on standard ports, drivebase wheel/axle defaults.  Operator places
    a tape mark before each turn and reads the residual angle after.

    Compare encoder mode (`set-gyro none`) against gyro mode
    (`set-gyro 3d`).  Issue #157: 3d is the projection heading the old
    `1d` used to compute, so this re-validates the same behaviour under
    the new name (it must still meet the bars below).

    Acceptance (AND):
      - encoder mode residual >= 5 deg (measurement floor)
      - gyro mode (3d) residual <= encoder * 1/3 AND <= +-4 deg absolute
    """
    p.waitUser("Mount Hub at 51 deg tilt, mark start angle on tape")
    pytest.skip(
        "G-6 is operator-driven; record encoder (set-gyro none) vs gyro "
        "(set-gyro 3d) residuals manually and verify ratio. See "
        "/home/ouwa/.claude/plans/157-partitioned-music.md"
    )


@pytest.mark.interactive
@pytest.mark.slow
def test_g7_straight_lateral_drift_bench(p):
    """G-7: encoder vs gyro lateral drift after `straight 1000 x 3`.

    Pre-condition: floor grid (100 mm spacing), bench centered on a
    grid line, both modes attempted from the same start.  Operator
    measures lateral drift at end of each run.

    Compare encoder mode (`set-gyro none`) against gyro mode
    (`set-gyro 3d`).

    Acceptance (AND):
      - encoder mode drift >= 10 mm (measurement floor)
      - gyro mode (3d) drift <= encoder * 1/2 AND <= +-15 mm absolute
    """
    p.waitUser("Center bench on floor grid line, mark start position")
    pytest.skip(
        "G-7 is operator-driven; record encoder (set-gyro none) vs gyro "
        "(set-gyro 3d) drift manually. See "
        "/home/ouwa/.claude/plans/157-partitioned-music.md"
    )
