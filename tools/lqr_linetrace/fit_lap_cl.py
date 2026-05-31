#!/usr/bin/env python3
"""Offline grey-box fitter for line-tracing lap captures (P0c, Issue #167).

Consumes one or more `.cap` files (schema ``linetrace_lap_run``, magic
0x0012) recorded by the P0b lap-capture daemon and estimates the line-sensor
+ disturbance model used by the LQG observer planned in P1+:

  - ``c``  (counts/mm) : sensor intensity slope inside the linear band,
  - ``Q = diag(q_pos, q_head)`` : process-noise spectral densities,
  - ``R``  (counts^2)   : measurement-noise variance (linear-regime effective),
  - a residual additive-bias INDICATOR ``b`` (counts) that P3 (a bias state)
    would absorb.

``L`` (sensor look-ahead, mm) is ANCHORED to the P0a bench value (default
55 mm) and NOT estimated: a single closed-loop lap constrains the product
``c*L`` and the noise levels well but separates ``c`` from ``L`` only weakly,
so the bench M1 measurement stays the primary c/L anchor and the lap merely
refines ``c`` around the anchored ``L``.

Method (pure Python stdlib, no numpy/scipy, self-contained -- does NOT import
lqr_sim.py):

  * Descriptor-driven ``.cap`` parser (offsets come from the field
    descriptors, not hard-coded), with span/overlap/type-width sanity checks.
  * Rail exclusion + moving-only + straight-segment selection (straight-only
    is the DEFAULT for decision-grade fits, because the relative-line heading
    propagation is only exact for a straight reference; the whole-lap fit is
    diagnostic-only and flagged CURVATURE_UNMODELED).
  * A 2-state Extended Kalman filter run as a prediction-error (innovations)
    evaluator; the Gaussian innovations negative-log-likelihood is minimised
    jointly over (c, q_pos, q_head, R) by a coarse log-grid + a log-space
    Nelder-Mead simplex.
  * A profile-likelihood band for ``c`` (identifiability diagnostic, ridge
    detection), plus an augmented-bias fit + robust mean-innovation
    cross-check.
  * Decision outputs: whether P3 (bias state) is needed (gated to require
    >=2 lighting/surface-varying laps) and a ``kf_rmeas`` suggestion.

The decision-grade numbers are reported only from the straight-segment fit.
With a single lap the P3 verdict is intentionally UNDECIDED and ``kf_rmeas`` is
reported as provisional.

See ``--help`` for the full CLI.  Determinism: no RNG; fixed iteration counts.
"""

import argparse
import json
import math
import struct
import sys

# ---------------------------------------------------------------------------
# Capture-file format constants (mirror apps/capture/include/capture_format.h).
# ---------------------------------------------------------------------------

CAP_MAGIC = 0x42504143          # "CAPB" little-endian
CAP_FILE_VERSION = 1
LAP_SCHEMA_MAGIC = 0x0012       # linetrace_lap_run schema sentinel

HEADER_SIZE = 64
FIELD_DESC_SIZE = 48
NAME_MAX = 32
FIELD_NAME_MAX = 16
FIELD_UNIT_MAX = 16

# CAPTURE_TYPE_* -> (struct code, byte width, signed).
CAP_TYPES = {
    0: ("B", 1),   # U8
    1: ("b", 1),   # I8
    2: ("H", 2),   # U16
    3: ("h", 2),   # I16
    4: ("I", 4),   # U32
    5: ("i", 4),   # I32
    6: ("Q", 8),   # U64
    7: ("q", 8),   # I64
    8: ("f", 4),   # F32
    9: ("d", 8),   # F64
}

# Required lap fields (by name).  edge is optional.
REQUIRED_FIELDS = (
    "ts_us", "intensity", "target", "turn_cmd_dps",
    "heading_mdeg", "turn_rate_dps", "speed_mmps",
)

DEG = 180.0 / math.pi
CHI2_1_95 = 3.841459             # chi-square 95% quantile, 1 dof
DELTA_NLL_95 = 0.5 * CHI2_1_95   # = 1.9207, profile-likelihood half-width


# ---------------------------------------------------------------------------
# .cap parser.
# ---------------------------------------------------------------------------

class CapError(Exception):
    """Raised on a malformed / unexpected .cap file."""


def parse_cap(path):
    """Parse a .cap file into (header_dict, fields, records).

    ``fields`` is a list of field descriptors (dicts); ``records`` is a list
    of dicts keyed by field name with scaled values applied.  The record
    decode is driven entirely by the field descriptors, so the parser is
    robust to schema evolution; we still cross-check every documented
    invariant and fail hard (CapError) on any mismatch.
    """
    with open(path, "rb") as fp:
        data = fp.read()

    if len(data) < HEADER_SIZE:
        raise CapError("%s: file shorter than 64-byte header" % path)

    magic, version, schema_magic, start_ts_us, record_size, record_count = \
        struct.unpack_from("<IHHQII", data, 0)
    if magic != CAP_MAGIC:
        raise CapError("%s: bad magic 0x%08x (expected 0x%08x 'CAPB')"
                       % (path, magic, CAP_MAGIC))
    if version != CAP_FILE_VERSION:
        sys.stderr.write("WARNING: %s: file version %d (expected %d)\n"
                         % (path, version, CAP_FILE_VERSION))
    if schema_magic != LAP_SCHEMA_MAGIC:
        sys.stderr.write("WARNING: %s: schema_magic 0x%04x (expected 0x%04x "
                         "lap); layout is self-describing, continuing\n"
                         % (path, schema_magic, LAP_SCHEMA_MAGIC))

    schema_name = data[24:24 + NAME_MAX].split(b"\0", 1)[0].decode(
        "ascii", "replace")
    field_count = data[56]

    payload_start = HEADER_SIZE + field_count * FIELD_DESC_SIZE
    if len(data) < payload_start:
        raise CapError("%s: truncated field-descriptor table (need %d bytes, "
                       "have %d)" % (path, payload_start, len(data)))

    fields = []
    spans = []
    seen_names = set()
    for i in range(field_count):
        off = HEADER_SIZE + i * FIELD_DESC_SIZE
        fname = data[off:off + FIELD_NAME_MAX].split(b"\0", 1)[0].decode(
            "ascii", "replace")
        ftype = data[off + 16]
        foff = data[off + 17]
        fsize = data[off + 18]
        fscale = struct.unpack_from("<b", data, off + 19)[0]
        funit = data[off + 20:off + 20 + FIELD_UNIT_MAX].split(
            b"\0", 1)[0].decode("ascii", "replace")

        if ftype not in CAP_TYPES:
            raise CapError("%s: field %r has unknown type tag %d"
                           % (path, fname, ftype))
        code, width = CAP_TYPES[ftype]
        if fsize != width:
            raise CapError("%s: field %r size %d != type-tag width %d"
                           % (path, fname, fsize, width))
        if foff + fsize > record_size:
            raise CapError("%s: field %r span [%d,%d) exceeds record_size %d"
                           % (path, fname, foff, foff + fsize, record_size))
        if fname in seen_names:
            raise CapError("%s: duplicate field name %r in descriptor table"
                           % (path, fname))
        seen_names.add(fname)
        fields.append(dict(name=fname, type=ftype, offset=foff, size=fsize,
                           scale_log10=fscale, unit=funit, code=code))
        spans.append((foff, foff + fsize, fname))

    # No two field byte-spans may overlap.
    spans.sort()
    for a, b in zip(spans, spans[1:]):
        if a[1] > b[0]:
            raise CapError("%s: fields %r and %r overlap ([%d,%d) vs [%d,%d))"
                           % (path, a[2], b[2], a[0], a[1], b[0], b[1]))

    # Total-byte invariant.
    expect = payload_start + record_count * record_size
    if len(data) != expect:
        raise CapError("%s: size %d != header+desc+records %d "
                       "(payload@%d + %d*%d)"
                       % (path, len(data), expect, payload_start,
                          record_count, record_size))

    by_name = {f["name"]: f for f in fields}
    for req in REQUIRED_FIELDS:
        if req not in by_name:
            raise CapError("%s: required field %r missing from schema"
                           % (path, req))

    # Decode all records via the descriptors.
    records = []
    for r in range(record_count):
        base = payload_start + r * record_size
        rec = {}
        for f in fields:
            raw = struct.unpack_from("<" + f["code"], data, base + f["offset"])[0]
            if f["scale_log10"]:
                rec[f["name"]] = raw * (10.0 ** f["scale_log10"])
            else:
                rec[f["name"]] = raw
        records.append(rec)

    header = dict(magic=magic, version=version, schema_magic=schema_magic,
                  start_ts_us=start_ts_us, record_size=record_size,
                  record_count=record_count, schema_name=schema_name,
                  field_count=field_count, payload_start=payload_start,
                  file_size=len(data))
    return header, fields, records


# ---------------------------------------------------------------------------
# Tick preprocessing: per-record dt, derived inputs, classification flags.
# ---------------------------------------------------------------------------

def build_ticks(records, args):
    """Turn raw records into a list of tick dicts with dt, inputs and flags.

    Each tick carries: dt (s), v (mm/s), omega (rad/s), intensity, target,
    err, turn (max(|cmd|,|achieved|) used for straight detection), and the
    boolean flags ``moving``, ``in_band``, ``straight``, ``post_settle``.
    The final tick has no successor dt and is dropped.
    """
    ticks = []
    n = len(records)
    last_target = None
    settle_left = 0
    for i in range(n - 1):
        a = records[i]
        b = records[i + 1]
        dt_us = (int(b["ts_us"]) - int(a["ts_us"])) & 0xFFFFFFFF
        dt = dt_us * 1e-6
        # Reject non-physical dt: keep walking but mark predict-only (skip
        # the whole tick -- no valid time base for the predict).
        bad_dt = (dt <= 0.0) or (dt > args.dt_max)

        intensity = float(a["intensity"])
        target = float(a["target"])
        speed = float(a["speed_mmps"])
        turn_ach = float(a["turn_rate_dps"])
        turn_cmd = float(a["turn_cmd_dps"])

        # target-change settling window
        if last_target is not None and target != last_target:
            settle_left = args.target_settle_ticks
        last_target = target
        post_settle = (settle_left <= 0)
        if settle_left > 0:
            settle_left -= 1

        omega_dps = turn_cmd if args.use_cmd else turn_ach
        omega = math.radians(omega_dps)
        v = speed

        moving = speed > args.speed_min
        in_band = (args.rail_lo < intensity < args.rail_hi)
        # straight metric: BOTH commanded and achieved turn must be small
        turn_mag = max(abs(turn_cmd), abs(turn_ach))
        low_turn = turn_mag < args.straight_turn_thresh

        ticks.append(dict(
            idx=i, dt=dt, bad_dt=bad_dt, v=v, omega=omega,
            intensity=intensity, target=target, err=target - intensity,
            speed=speed, turn_mag=turn_mag, low_turn=low_turn,
            moving=moving, in_band=in_band, post_settle=post_settle,
        ))
    return ticks


def mark_straight_runs(ticks, args):
    """Mark ticks belonging to a straight RUN (>= straight_min_ticks of
    consecutive low-turn, non-bad-dt ticks).  Sets tick['straight']."""
    for t in ticks:
        t["straight"] = False
    run = []

    def flush(run):
        if len(run) >= args.straight_min_ticks:
            for t in run:
                t["straight"] = True

    for t in ticks:
        if t["low_turn"] and not t["bad_dt"]:
            run.append(t)
        else:
            flush(run)
            run = []
    flush(run)


def select_ticks(ticks, args, straight_only, target_filter):
    """Return the ordered tick subsequence used for one fit.

    The whole continuous-time sequence is preserved (so predict-only ticks
    keep the time base); a per-tick ``use`` flag decides whether the tick's
    innovation enters the NLL.  Returns the list with each tick annotated
    ``use`` (measurement-update + counts in NLL) for the requested regime.
    """
    out = []
    for t in ticks:
        t2 = dict(t)
        use = True
        if t2["bad_dt"]:
            use = False
        if not t2["moving"]:
            use = False
        if not t2["in_band"]:
            use = False
        if not t2["post_settle"]:
            use = False
        if straight_only and not t2["straight"]:
            use = False
        if target_filter is not None and t2["target"] != target_filter:
            use = False
        t2["use"] = use
        out.append(t2)
    return out


# ---------------------------------------------------------------------------
# 2x2 linear algebra (hand-rolled; mirrors lqr_sim mm/mt/madd but local).
# ---------------------------------------------------------------------------

def mm2(A, B):
    return [[A[0][0] * B[0][0] + A[0][1] * B[1][0],
             A[0][0] * B[0][1] + A[0][1] * B[1][1]],
            [A[1][0] * B[0][0] + A[1][1] * B[1][0],
             A[1][0] * B[0][1] + A[1][1] * B[1][1]]]


def mt2(A):
    return [[A[0][0], A[1][0]], [A[0][1], A[1][1]]]


def madd2(A, B):
    return [[A[0][0] + B[0][0], A[0][1] + B[0][1]],
            [A[1][0] + B[1][0], A[1][1] + B[1][1]]]


S_MIN = 1e-9


# ---------------------------------------------------------------------------
# EKF prediction-error evaluator -> innovations NLL.
# ---------------------------------------------------------------------------

def ekf_nll(ticks, c, q_pos, q_head, R, args, edge_sign, bias=0.0,
            collect_innov=False):
    """Run the 2-state EKF over ``ticks`` and return the innovations NLL.

    State x = [e_y (mm), e_theta (rad)].  Predict (ZOH, per-record dt):
        e_y'    = e_y + v sin(e_theta) dt
        e_theta'= e_theta + omega dt
    Covariance uses the Jacobian Ad = [[1, v cos(e_theta) dt],[0,1]].
    Process noise Q*dt added each predict (rate-spectral; --q-no-dt-scale
    adds raw Q).  Measurement h(x) = edge_sign*c*(e_y + L sin e_theta) + bias,
    Jacobian C = [edge_sign*c, edge_sign*c*L cos e_theta].  Joseph-form update.

    ``edge_sign`` is +/-1; fitted ``c`` stays a positive magnitude.  The first
    ``burn-in-ticks`` measurement updates and predict-only ticks are excluded
    from the NLL sum (state still propagates).  Returns NLL (float) or, with
    ``collect_innov``, the tuple (NLL, n_updates, innov_list).
    """
    L = args.L
    ey = 0.0
    eth = 0.0
    P = [[args.p0_pos, 0.0], [0.0, args.p0_head]]
    Q = [[q_pos, 0.0], [0.0, q_head]]

    nll = 0.0
    n_upd = 0          # measurement updates counted into the NLL
    seen_upd = 0       # measurement updates total (for burn-in gating)
    coast = 0          # contiguous predict-only run length
    innovs = []

    for t in ticks:
        dt = t["dt"]
        if t["bad_dt"]:
            # No valid time base: do not propagate, treat as a gap boundary.
            coast += 1
            if coast > args.max_coast_ticks:
                P = [[args.p0_pos, 0.0], [0.0, args.p0_head]]
            continue

        v = t["v"]
        omega = t["omega"]
        sin_eth = math.sin(eth)
        cos_eth = math.cos(eth)

        # Predict (state).
        ey_p = ey + v * sin_eth * dt
        eth_p = eth + omega * dt

        # Predict (covariance) with Jacobian and dt-scaled (or raw) Q.
        Ad = [[1.0, v * cos_eth * dt], [0.0, 1.0]]
        if args.q_no_dt_scale:
            Qk = Q
        else:
            Qk = [[Q[0][0] * dt, 0.0], [0.0, Q[1][1] * dt]]
        Pp = madd2(mm2(mm2(Ad, P), mt2(Ad)), Qk)

        if not t["use"]:
            # Predict-only: propagate state estimate, no correction.
            ey, eth = ey_p, eth_p
            P = Pp
            coast += 1
            if coast > args.max_coast_ticks:
                P = [[args.p0_pos, 0.0], [0.0, args.p0_head]]
            continue
        coast = 0

        # Measurement update.  Nonlinear h(x) and its Jacobian are evaluated at
        # the PREDICTED state (eth_p), not the pre-predict angle:
        #   h(x) = edge_sign*c*(e_y + L*sin e_theta) + bias
        #   C    = [edge_sign*c, edge_sign*c*L*cos e_theta]
        sin_ethp = math.sin(eth_p)
        cos_ethp = math.cos(eth_p)
        cc0 = edge_sign * c
        cc1 = edge_sign * c * L * cos_ethp
        # Innovation (true nonlinear predicted measurement)
        h = edge_sign * c * (ey_p + L * sin_ethp) + bias
        nu = t["err"] - h
        # S = C Pp C' + R
        PpCt0 = Pp[0][0] * cc0 + Pp[0][1] * cc1
        PpCt1 = Pp[1][0] * cc0 + Pp[1][1] * cc1
        S = cc0 * PpCt0 + cc1 * PpCt1 + R
        if S < S_MIN:
            S = S_MIN
        K0 = PpCt0 / S
        K1 = PpCt1 / S
        ey = ey_p + K0 * nu
        eth = eth_p + K1 * nu
        # Joseph form: P = (I-KC) Pp (I-KC)' + K R K'
        ImKC = [[1.0 - K0 * cc0, -K0 * cc1],
                [-K1 * cc0, 1.0 - K1 * cc1]]
        P = madd2(mm2(mm2(ImKC, Pp), mt2(ImKC)),
                  [[K0 * R * K0, K0 * R * K1], [K1 * R * K0, K1 * R * K1]])

        seen_upd += 1
        if seen_upd <= args.burn_in_ticks:
            continue  # burn-in: update state but skip the NLL contribution
        nll += 0.5 * math.log(2.0 * math.pi * S) + 0.5 * nu * nu / S
        n_upd += 1
        if collect_innov:
            innovs.append(nu)

    if collect_innov:
        return nll, n_upd, innovs
    return nll


# ---------------------------------------------------------------------------
# Optimisers (stdlib, deterministic): coarse log-grid + log-space Nelder-Mead.
# ---------------------------------------------------------------------------

def _logspace(lo, hi, n):
    if n == 1:
        return [math.sqrt(lo * hi)]
    a = math.log(lo)
    b = math.log(hi)
    return [math.exp(a + (b - a) * i / (n - 1)) for i in range(n)]


def coarse_grid(ticks, args, edge_sign, with_bias=False):
    """Coordinate-descent over log-spaced ranges to find a good basin.

    Returns the best (params, nll).  params is [c, q_pos, q_head, R] (+ bias
    if with_bias).  Deterministic: fixed grids, a few descent sweeps.
    """
    c_grid = _logspace(30.0, 300.0, 12)
    r_grid = _logspace(1.0, 1e4, 10)
    qp_grid = _logspace(1e-3, 1e2, 8)
    qh_grid = _logspace(1e-8, 1e-1, 8)

    # Seed at geometric centres.
    c = 110.0
    qp = 1.0
    qh = 1e-4
    R = 25.0
    bias = 0.0

    def f(c, qp, qh, R, bias):
        return ekf_nll(ticks, c, qp, qh, R, args, edge_sign, bias=bias)

    best = f(c, qp, qh, R, bias)
    for _ in range(3):
        for cg in c_grid:
            v = f(cg, qp, qh, R, bias)
            if v < best:
                best, c = v, cg
        for rg in r_grid:
            v = f(c, qp, qh, rg, bias)
            if v < best:
                best, R = v, rg
        for g in qp_grid:
            v = f(c, g, qh, R, bias)
            if v < best:
                best, qp = v, g
        for g in qh_grid:
            v = f(c, qp, g, R, bias)
            if v < best:
                best, qh = v, g
        if with_bias:
            for bg in [-50, -20, -10, -5, -2, 0, 2, 5, 10, 20, 50]:
                v = f(c, qp, qh, R, float(bg))
                if v < best:
                    best, bias = v, float(bg)
    params = [c, qp, qh, R] + ([bias] if with_bias else [])
    return params, best


def nelder_mead(fn, x0, steps, max_iter=600, tol=1e-7):
    """Textbook Nelder-Mead simplex minimiser (pure stdlib, deterministic).

    Minimises ``fn`` over the vector ``x0`` (caller supplies log-space
    coordinates).  ``steps`` is the initial per-dimension simplex offset.
    """
    n = len(x0)
    alpha, gamma, rho, sigma = 1.0, 2.0, 0.5, 0.5

    simplex = [list(x0)]
    for i in range(n):
        pt = list(x0)
        pt[i] += steps[i]
        simplex.append(pt)
    fvals = [fn(p) for p in simplex]

    for _ in range(max_iter):
        order = sorted(range(n + 1), key=lambda k: fvals[k])
        simplex = [simplex[k] for k in order]
        fvals = [fvals[k] for k in order]

        if abs(fvals[-1] - fvals[0]) <= tol * (abs(fvals[0]) + tol):
            break

        # Centroid of all but worst.
        cen = [sum(simplex[k][j] for k in range(n)) / n for j in range(n)]
        worst = simplex[-1]
        # Reflection
        xr = [cen[j] + alpha * (cen[j] - worst[j]) for j in range(n)]
        fr = fn(xr)
        if fvals[0] <= fr < fvals[-2]:
            simplex[-1], fvals[-1] = xr, fr
            continue
        if fr < fvals[0]:
            xe = [cen[j] + gamma * (xr[j] - cen[j]) for j in range(n)]
            fe = fn(xe)
            if fe < fr:
                simplex[-1], fvals[-1] = xe, fe
            else:
                simplex[-1], fvals[-1] = xr, fr
            continue
        # Contraction
        xc = [cen[j] + rho * (worst[j] - cen[j]) for j in range(n)]
        fc = fn(xc)
        if fc < fvals[-1]:
            simplex[-1], fvals[-1] = xc, fc
            continue
        # Shrink
        best = simplex[0]
        for k in range(1, n + 1):
            simplex[k] = [best[j] + sigma * (simplex[k][j] - best[j])
                          for j in range(n)]
            fvals[k] = fn(simplex[k])

    order = sorted(range(n + 1), key=lambda k: fvals[k])
    return simplex[order[0]], fvals[order[0]]


# Log-space bounds for nuisance params, used to flag at-bound ridges.
NUIS_BOUNDS = {
    "q_pos": (1e-4, 1e3),
    "q_head": (1e-9, 1e0),
    "R": (1e-1, 1e6),
}


def _at_bound(name, val, frac=0.02):
    lo, hi = NUIS_BOUNDS[name]
    llo, lhi = math.log(lo), math.log(hi)
    lv = math.log(val)
    span = lhi - llo
    return (lv - llo) < frac * span or (lhi - lv) < frac * span


def fit_full(ticks, args, edge_sign, with_bias=False, fixed_c=None,
             seed_override=None):
    """Fit (c, q_pos, q_head, R[, bias]) by grid + Nelder-Mead in log-space.

    If ``fixed_c`` is given, ``c`` is held and only the nuisance params (and
    bias) are optimised (used for the profile likelihood).  ``seed_override``
    (a dict with c/q_pos/q_head/R[/bias]) replaces the grid seed -- used to
    start the augmented-bias fit from the no-bias optimum so the bias model is
    a TRUE superset (guaranteeing LR >= 0 up to optimiser tolerance).  Returns
    a dict with params, nll, and at-bound flags.
    """
    if seed_override is not None:
        seed = [seed_override["c"], seed_override["q_pos"],
                seed_override["q_head"], seed_override["R"]]
        if with_bias:
            seed.append(seed_override.get("bias", 0.0))
    else:
        seed, _ = coarse_grid(ticks, args, edge_sign, with_bias=with_bias)
    # seed = [c, qp, qh, R] (+bias)
    if fixed_c is not None:
        seed[0] = fixed_c

    # Decide which params are free.  c is free unless fixed_c.
    # Order of free log-vars: [c?, qp, qh, R, bias?]
    free_c = fixed_c is None

    def unpack(x):
        k = 0
        if free_c:
            c = math.exp(x[k]); k += 1
        else:
            c = fixed_c
        qp = math.exp(x[k]); k += 1
        qh = math.exp(x[k]); k += 1
        R = math.exp(x[k]); k += 1
        if with_bias:
            bias = x[k]; k += 1
        else:
            bias = 0.0
        return c, qp, qh, R, bias

    def obj(x):
        c, qp, qh, R, bias = unpack(x)
        # Soft positivity already via log; bias is linear.
        try:
            return ekf_nll(ticks, c, qp, qh, R, args, edge_sign, bias=bias)
        except (ValueError, OverflowError):
            return 1e18

    x0 = []
    steps = []
    if free_c:
        x0.append(math.log(seed[0])); steps.append(0.4)
    x0.append(math.log(seed[1])); steps.append(0.7)
    x0.append(math.log(seed[2])); steps.append(0.7)
    x0.append(math.log(seed[3])); steps.append(0.7)
    if with_bias:
        x0.append(seed[4] if with_bias and len(seed) > 4 else 0.0)
        steps.append(5.0)

    if getattr(args, "grid_only", False):
        # Debug: skip Nelder-Mead, report the (seeded) grid optimum.
        xopt, fopt = x0, obj(x0)
    else:
        xopt, fopt = nelder_mead(obj, x0, steps)
    c, qp, qh, R, bias = unpack(xopt)

    at_bound = {
        "q_pos": _at_bound("q_pos", qp),
        "q_head": _at_bound("q_head", qh),
        "R": _at_bound("R", R),
    }
    return dict(c=c, q_pos=qp, q_head=qh, R=R, bias=bias, nll=fopt,
                at_bound=at_bound)


def profile_c(ticks, args, edge_sign, c_hat, nll_hat):
    """Profile-likelihood band for c: sweep c, re-optimise nuisance params,
    report the c range within DELTA_NLL_95 of the minimum.  Also detect a
    nuisance-at-bound ridge (downgrades the 'band' to 'ridge')."""
    # Sweep a log grid around c_hat (and across the plausible bracket).
    lo = max(20.0, c_hat / 4.0)
    hi = min(400.0, c_hat * 4.0)
    grid = _logspace(lo, hi, 25)
    pts = []
    ridge = False
    for cg in grid:
        r = fit_full(ticks, args, edge_sign, with_bias=False, fixed_c=cg)
        pts.append((cg, r["nll"]))
        if any(r["at_bound"].values()):
            ridge = True
    # Band: contiguous c where nll - nll_hat <= DELTA_NLL_95.
    within = [cg for (cg, nll) in pts if (nll - nll_hat) <= DELTA_NLL_95]
    if within:
        band = (min(within), max(within))
    else:
        band = None
    # Flat/degenerate: band spans most of the swept bracket.
    flat = False
    if band is not None:
        span_frac = (math.log(band[1]) - math.log(band[0])) / \
                    (math.log(hi) - math.log(lo))
        flat = span_frac > 0.6
    return dict(grid=pts, band=band, ridge=ridge, flat=flat,
                bracket=(lo, hi))


# ---------------------------------------------------------------------------
# Per-lap fit driver.
# ---------------------------------------------------------------------------

def median(xs):
    s = sorted(xs)
    n = len(s)
    if n == 0:
        return float("nan")
    if n % 2:
        return s[n // 2]
    return 0.5 * (s[n // 2 - 1] + s[n // 2])


def stdev(xs):
    n = len(xs)
    if n < 2:
        return 0.0
    m = sum(xs) / n
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (n - 1))


def fit_regime(ticks, args, edge_sign, label):
    """Fit one tick subsequence (already ``use``-annotated) and assemble the
    full per-regime result block.  ``ticks`` must retain predict-only ticks."""
    n_used = sum(1 for t in ticks if t["use"])
    res = dict(label=label, n_used=n_used)
    # Need enough measurement updates AFTER the burn-in skip to actually have a
    # non-empty NLL; otherwise the objective is a flat zero and the optimiser
    # returns a meaningless "fit".
    n_post_burnin = n_used - args.burn_in_ticks
    res["n_post_burnin"] = n_post_burnin
    if n_used < args.min_fit_updates or n_post_burnin < args.min_fit_updates:
        res["status"] = "INSUFFICIENT_UPDATES"
        return res

    base = fit_full(ticks, args, edge_sign, with_bias=False)
    res.update(base)
    res["status"] = "OK"
    res["cL"] = base["c"] * args.L
    res["sqrt_R"] = math.sqrt(base["R"]) if base["R"] > 0 else float("nan")

    # RMS innovation + innovations for the bias cross-check.
    _, n_upd, innovs = ekf_nll(ticks, base["c"], base["q_pos"], base["q_head"],
                               base["R"], args, edge_sign, bias=0.0,
                               collect_innov=True)
    res["rms_innov"] = (math.sqrt(sum(v * v for v in innovs) / len(innovs))
                        if innovs else float("nan"))
    res["median_innov"] = median(innovs) if innovs else float("nan")
    if getattr(args, "verbose", False):
        sys.stderr.write("# innovations [%s]: %s\n"
                         % (label, " ".join("%.2f" % v for v in innovs)))

    # Profile-likelihood band for c.
    prof = profile_c(ticks, args, edge_sign, base["c"], base["nll"])
    res["profile"] = prof
    # Sanity gate FIRST: a c outside the plausible bracket [30,300] means the
    # lap can't identify the measurement model (e.g. curvature-contaminated
    # data drives c->0); flag it and defer to bench M1, regardless of the
    # profile shape.
    if not (30.0 <= base["c"] <= 300.0):
        res["c_label"] = "out-of-bracket"
    elif prof["ridge"] or prof["flat"] or any(base["at_bound"].values()):
        res["c_label"] = "ridge"   # not separable -- defer to bench M1
    else:
        res["c_label"] = "band"

    # Augmented bias fit (5th param) + LR statistic.  Seed from the no-bias
    # optimum (bias=0) so the bias model is a TRUE superset: the LR is then
    # guaranteed >= 0 up to optimiser tolerance.
    bf = fit_full(ticks, args, edge_sign, with_bias=True,
                  seed_override=dict(c=base["c"], q_pos=base["q_pos"],
                                     q_head=base["q_head"], R=base["R"],
                                     bias=0.0))
    res["bias_fit"] = bf
    res["bias_b"] = bf["bias"]
    res["bias_dey_mm"] = (bf["bias"] / bf["c"]) if bf["c"] else float("nan")
    lr = 2.0 * (base["nll"] - bf["nll"])
    # Clamp tiny negative LR from optimiser tolerance to 0; a materially
    # negative LR signals non-convergence -> flag it.
    res["bias_lr_raw"] = lr
    res["bias_nonconverged"] = lr < -1e-3
    res["bias_lr"] = max(0.0, lr)
    res["bias_significant"] = (res["bias_lr"] > CHI2_1_95
                               and not res["bias_nonconverged"])
    # Robust cross-check: median no-bias innovation -> b ~ median(nu).
    res["bias_b_robust"] = res["median_innov"]
    res["bias_dey_robust_mm"] = (res["median_innov"] / base["c"]
                                 if base["c"] else float("nan"))
    return res


def fit_lap(path, args):
    """Parse and fit one lap; return a structured result dict."""
    header, fields, records = parse_cap(path)
    ticks_all = build_ticks(records, args)
    mark_straight_runs(ticks_all, args)

    # Sample-accounting diagnostics.
    n_rec = header["record_count"]
    n_tick = len(ticks_all)
    n_rail = sum(1 for t in ticks_all if not t["in_band"])
    n_moving = sum(1 for t in ticks_all if t["moving"])
    n_straight = sum(1 for t in ticks_all if t["straight"])
    dts = [t["dt"] for t in ticks_all if not t["bad_dt"]]
    n_bad_dt = sum(1 for t in ticks_all if t["bad_dt"])
    rail_frac = (n_rail / n_tick) if n_tick else 0.0
    targets = sorted({t["target"] for t in ticks_all})

    lap = dict(
        path=path, header=header,
        n_records=n_rec, n_ticks=n_tick, n_rail_excluded=n_rail,
        rail_frac=rail_frac, n_moving=n_moving, n_straight=n_straight,
        n_bad_dt=n_bad_dt, targets=targets,
        dt_min=min(dts) if dts else float("nan"),
        dt_max=max(dts) if dts else float("nan"),
        dt_mean=(sum(dts) / len(dts)) if dts else float("nan"),
        dt_median=median(dts) if dts else float("nan"),
    )

    edge_sign = -1.0 if args.edge_sign == "neg" else 1.0
    lap["edge_sign"] = edge_sign

    # Decision-grade fit = straight-only, per-target regime.
    straight_total = sum(
        1 for t in select_ticks(ticks_all, args, True, None) if t["use"])
    lap["straight_used_total"] = straight_total

    regimes = []
    if not args.whole_lap_only:
        if straight_total < args.min_straight_ticks_total:
            lap["straight_status"] = "INSUFFICIENT_STRAIGHT_DATA"
        else:
            lap["straight_status"] = "OK"
            tgts = targets if args.target_filter is None else [args.target_filter]
            for tg in tgts:
                sel = select_ticks(ticks_all, args, True, tg)
                if sum(1 for t in sel if t["use"]) < args.min_fit_updates:
                    continue
                r = fit_regime(sel, args, edge_sign,
                               "straight target=%g" % tg)
                regimes.append(r)
            # Combined (all targets) straight fit too.
            sel_all = select_ticks(ticks_all, args, True, args.target_filter)
            r_all = fit_regime(sel_all, args, edge_sign, "straight all-targets")
            regimes.append(r_all)
    lap["regimes"] = regimes

    # Whole-lap diagnostic (opt-in), flagged CURVATURE_UNMODELED.
    lap["whole_lap"] = None
    if args.whole_lap or args.whole_lap_only:
        sel = select_ticks(ticks_all, args, False, args.target_filter)
        wl = fit_regime(sel, args, edge_sign, "whole-lap (CURVATURE_UNMODELED)")
        wl["diagnostic_only"] = True
        lap["whole_lap"] = wl

    # Pick the decision regime: prefer the combined straight all-targets fit.
    # A regime whose c is out-of-bracket has NOT identified the measurement
    # model (sanity gate) and must not feed the c/R/bias decisions; record it
    # for reporting but mark it non-decision-grade.
    def usable(r):
        # Decision-grade requires a clean, identified fit: status OK, the c
        # profile a genuine band (NOT ridge/flat/out-of-bracket), and no
        # nuisance parameter pinned at a search bound.  Anything else is a
        # diagnostic only and must not feed kf_rmeas / P3.
        return (r.get("status") == "OK"
                and r.get("c_label") == "band"
                and not any(r.get("at_bound", {}).values()))

    decision = None
    for r in regimes:
        if r.get("label", "").startswith("straight all-targets") and usable(r):
            decision = r
            break
    if decision is None:
        for r in regimes:
            if usable(r):
                decision = r
                break
    # If only out-of-bracket OK fits exist, surface that explicitly so the
    # aggregate reports c*L diagnostics but refuses decision numbers.
    lap["has_ok_fit"] = any(r.get("status") == "OK" for r in regimes)
    lap["decision_regime"] = decision
    if decision is None and lap["has_ok_fit"]:
        lap["straight_status"] = lap.get("straight_status", "OK") \
            if lap.get("straight_status") == "INSUFFICIENT_STRAIGHT_DATA" \
            else "MODEL_NOT_IDENTIFIED"
    return lap


# ---------------------------------------------------------------------------
# Reporting.
# ---------------------------------------------------------------------------

def fmt_band(prof):
    if prof is None or prof.get("band") is None:
        return "n/a"
    lo, hi = prof["band"]
    tag = " [RIDGE]" if prof["ridge"] else (" [FLAT]" if prof["flat"] else "")
    return "[%.1f, %.1f]%s" % (lo, hi, tag)


def print_regime(r, args, indent="  "):
    if r.get("status") != "OK":
        print("%s%s: %s (%d updates)"
              % (indent, r["label"], r.get("status"), r.get("n_used", 0)))
        return
    print("%s%s  (%d measurement updates)" % (indent, r["label"], r["n_used"]))
    print("%s  c        = %.2f counts/mm   [label: %s]"
          % (indent, r["c"], r["c_label"]))
    print("%s  c-band   = %s  (profile-likelihood, dNLL<=%.3f)"
          % (indent, fmt_band(r["profile"]), DELTA_NLL_95))
    print("%s  c*L      = %.1f  (L=%.1f mm anchored; bench expects ~%g..%g)"
          % (indent, r["cL"], args.L, 90 * args.L, 150 * args.L))
    print("%s  R        = %.3f counts^2   (linear-regime effective)"
          % (indent, r["R"]))
    print("%s  sqrt(R)  = %.2f counts      (sanity ~2..7)" % (indent, r["sqrt_R"]))
    print("%s  Q spec   = q_pos=%.4g  q_head=%.4g  (rate-spectral, per dt-scaled)"
          % (indent, r["q_pos"], r["q_head"]))
    print("%s  RMS innov= %.2f   median innov=%.2f"
          % (indent, r["rms_innov"], r["median_innov"]))
    print("%s  bias b   = %.2f counts (augmented) | %.2f (robust median)"
          % (indent, r["bias_b"], r["bias_b_robust"]))
    print("%s  bias dey = %.3f mm (aug) | %.3f mm (robust)   LR=%.2f %s"
          % (indent, r["bias_dey_mm"], r["bias_dey_robust_mm"], r["bias_lr"],
             "SIGNIFICANT" if r["bias_significant"] else "ns"))
    if r.get("bias_nonconverged"):
        print("%s  WARNING: augmented-bias fit did not improve on the no-bias "
              "fit (raw LR=%.2f<0) -> bias indicator NOT reliable here"
              % (indent, r["bias_lr_raw"]))
    if r["c_label"] == "out-of-bracket":
        print("%s  WARNING: fitted c=%.2f is outside the plausible bracket "
              "[30,300] -> measurement model NOT identified on this segment "
              "(likely curvature/transient contamination); defer to bench M1, "
              "do NOT use these c/R/bias numbers for decisions"
              % (indent, r["c"]))
    ab = r["at_bound"]
    if any(ab.values()):
        flagged = ",".join(k for k, v in ab.items() if v)
        print("%s  WARNING: nuisance param(s) at search bound: %s "
              "-> c interval is a RIDGE, lean on bench M1" % (indent, flagged))


def q_pertick(q_spec, dt_nominal, args):
    """Convert rate-spectral Q to the per-tick additive convention that
    lqr_sim / firmware kf_qpos/kf_qhead use."""
    if args.q_no_dt_scale:
        return q_spec    # already per-tick (raw add)
    return q_spec * dt_nominal


def report_lap(lap, args):
    print("=" * 72)
    print("LAP: %s" % lap["path"])
    h = lap["header"]
    print("  parser: magic=0x%08x ver=%d schema=0x%04x name=%s fields=%d "
          "record_size=%d records=%d file=%dB payload@%d (byte-exact)"
          % (h["magic"], h["version"], h["schema_magic"], h["schema_name"],
             h["field_count"], h["record_size"], h["record_count"],
             h["file_size"], h["payload_start"]))
    print("  ticks: %d  rail-excluded=%d (%.1f%%)  moving=%d  straight=%d  "
          "bad-dt=%d  targets=%s"
          % (lap["n_ticks"], lap["n_rail_excluded"], 100 * lap["rail_frac"],
             lap["n_moving"], lap["n_straight"], lap["n_bad_dt"],
             lap["targets"]))
    print("  dt[ms]: min=%.2f max=%.2f mean=%.3f median=%.3f"
          % (1e3 * lap["dt_min"], 1e3 * lap["dt_max"],
             1e3 * lap["dt_mean"], 1e3 * lap["dt_median"]))
    print("  straight ticks used (decision base): %d  (min required %d) -> %s"
          % (lap["straight_used_total"], args.min_straight_ticks_total,
             lap.get("straight_status", "n/a")))

    if lap["regimes"]:
        print("  --- straight-segment fits (DECISION-GRADE) ---")
        for r in lap["regimes"]:
            print_regime(r, args)
    elif lap.get("straight_status") == "INSUFFICIENT_STRAIGHT_DATA":
        print("  INSUFFICIENT_STRAIGHT_DATA: refusing decision numbers; "
              "reporting diagnostics only.")

    if lap["whole_lap"]:
        print("  --- whole-lap fit (CURVATURE_UNMODELED -- diagnostic only) ---")
        print_regime(lap["whole_lap"], args)

    return lap


# ---------------------------------------------------------------------------
# Aggregate + decisions.
# ---------------------------------------------------------------------------

def aggregate_and_decide(laps, args):
    """Aggregate per-lap decision regimes and emit the two decisions."""
    decided = [l["decision_regime"] for l in laps
               if l.get("decision_regime")]
    n_laps_used = len(decided)

    print("=" * 72)
    print("AGGREGATE / DECISIONS")
    print("  laps with a usable straight-segment fit: %d / %d"
          % (n_laps_used, len(laps)))

    out = dict(n_laps=len(laps), n_laps_used=n_laps_used)

    if n_laps_used == 0:
        # Report any c*L diagnostics from OK-but-non-identified fits, but
        # refuse decision numbers (sanity gate / INSUFFICIENT_STRAIGHT_DATA).
        diag = []
        for l in laps:
            for r in l.get("regimes", []):
                if r.get("status") == "OK":
                    diag.append(r)
        if diag:
            cls = [r["cL"] for r in diag]
            print("  No DECISION-GRADE fit (model not identified / "
                  "insufficient straight data); diagnostic c*L only:")
            print("    c*L range = [%.1f, %.1f]  (bench expects ~%g..%g)"
                  % (min(cls), max(cls), 90 * args.L, 150 * args.L))
            print("    -> lean on bench M1 for c/L; capture straighter "
                  "segments or >=2 laps for noise/bias decisions.")
            out["diag_cL"] = [min(cls), max(cls)]
        else:
            print("  No usable straight-segment fits; cannot issue decisions.")
        out["p3_needed"] = "UNDECIDED (NO_USABLE_FIT)"
        out["kf_rmeas"] = None
        return out

    cs = [r["c"] for r in decided]
    Rs = [r["R"] for r in decided]
    deys = [abs(r["bias_dey_mm"]) for r in decided]
    sigs = [r["bias_significant"] for r in decided]

    # c-slope variation.
    c_min, c_max = min(cs), max(cs)
    c_mean = sum(cs) / len(cs)
    c_spread_frac = (c_max - c_min) / c_mean if c_mean else float("nan")
    print("  c across laps: min=%.1f max=%.1f mean=%.1f stdev=%.1f  "
          "spread=(max-min)/mean=%.1f%%"
          % (c_min, c_max, c_mean, stdev(cs), 100 * c_spread_frac))
    out["c_min"], out["c_max"], out["c_mean"] = c_min, c_max, c_mean
    out["c_spread_frac"] = c_spread_frac

    # Bias spread + common component.
    max_dey = max(deys)
    bias_spread = (max(deys) - min(deys))
    print("  |Delta e_y| across laps: max=%.3f mm  spread(max-min)=%.3f mm  "
          "any-significant=%s"
          % (max_dey, bias_spread, any(sigs)))
    out["max_dey_mm"] = max_dey
    out["bias_spread_mm"] = bias_spread
    out["any_bias_significant"] = any(sigs)

    # Decision 2: kf_rmeas.  A hard decision number is only issued with >=2
    # usable decision-grade laps AND sqrt(R) in the sane 2..7 band; otherwise
    # it is reported as PROVISIONAL and the machine field kf_rmeas is left null
    # (the provisional value lives in kf_rmeas_provisional) so consumers never
    # mistake a single-lap / out-of-sanity reading for a committed value.
    R_med = median(Rs)
    sqrtR = math.sqrt(R_med) if R_med > 0 else float("nan")
    kf_rmeas_val = round(R_med)
    r_sane = 2.0 <= sqrtR <= 7.0
    decision_grade = (n_laps_used >= 2) and r_sane
    out["R_median"] = R_med
    out["R_per_lap"] = Rs
    out["kf_rmeas_provisional"] = kf_rmeas_val
    if decision_grade:
        out["kf_rmeas"] = kf_rmeas_val
        out["kf_rmeas_status"] = "DECISION_GRADE"
    else:
        out["kf_rmeas"] = None
        out["kf_rmeas_status"] = "PROVISIONAL"
    status_txt = "DECISION-GRADE" if decision_grade else "PROVISIONAL"
    print("  Decision 2 -- kf_rmeas = %d counts^2 [%s]  (median R; per-lap "
          "R=%s; sqrt=%.2f)"
          % (kf_rmeas_val, status_txt, ["%.1f" % x for x in Rs], sqrtR))
    if not r_sane:
        print("       WARNING: sqrt(R)=%.2f outside ~2..7 -> likely absorbing "
              "curvature/transient/censoring; treat as PROVISIONAL" % sqrtR)
    if n_laps_used < 2:
        print("       (%d usable lap(s) < 2 -> kf_rmeas is PROVISIONAL, "
              "machine field null)" % n_laps_used)

    # Q suggestions (both conventions), from the decision regime(s).
    dt_nom = median([l["dt_median"] for l in laps])
    qp_spec = median([r["q_pos"] for r in decided])
    qh_spec = median([r["q_head"] for r in decided])
    kf_qpos = q_pertick(qp_spec, dt_nom, args)
    kf_qhead = q_pertick(qh_spec, dt_nom, args)
    out["kf_qpos"] = kf_qpos
    out["kf_qhead"] = kf_qhead
    out["q_pos_spectral"] = qp_spec
    out["q_head_spectral"] = qh_spec
    out["dt_nominal"] = dt_nom
    print("  Q suggestions (P1a/lqr_sim per-tick convention, drop-in):")
    print("       kf_qpos  = %.4g    kf_qhead = %.4g    (dt_nominal=%.4f s)"
          % (kf_qpos, kf_qhead, dt_nom))
    print("     physical reference (rate-spectral, dt-scaled EKF):")
    print("       q_pos = %.4g [mm^2/s]   q_head = %.4g [rad^2/s]"
          % (qp_spec, qh_spec))
    print("       (P1a defaults today: kf_qpos=1, kf_qhead=1e-4)")

    # Decision 1: P3 needed?  Gated to require >=2 USABLE (decision-grade)
    # lighting/surface laps -- supplying two captures where only one yields a
    # usable straight fit must NOT unlock a hard yes/no.
    if n_laps_used < 2:
        verdict = "UNDECIDED (INSUFFICIENT_LAPS_FOR_AGGREGATE)"
        print("  Decision 1 -- P3_NEEDED: %s" % verdict)
        print("       %d usable decision-grade lap(s) < 2: cannot separate a "
              "physical ambient bias from a setpoint/x0/geometry offset "
              "(plan 3.4)." % n_laps_used)
        print("       within-lap residual-bias indicator: |Delta e_y|=%.3f mm "
              "(threshold %.2f mm), %s"
              % (max_dey, args.p3_bias_thresh_mm,
                 "would-trip" if max_dey >= args.p3_bias_thresh_mm
                 else "below-threshold"))
    else:
        # (i) >=2 laps (satisfied here); (ii) max |dey| >= thresh;
        # (iii) significant in laps that exhibit it; (iv) bias TRACKS lighting
        # (varies across laps -- spread material vs the common component).
        cond_mag = max_dey >= args.p3_bias_thresh_mm
        cond_sig = any(sigs)
        # bias tracks lighting iff the spread is a material fraction of the max
        # (a lap-invariant constant -> small spread -> setpoint/geometry, not P3)
        cond_tracks = bias_spread >= 0.5 * args.p3_bias_thresh_mm
        if cond_mag and cond_sig and cond_tracks:
            verdict = "yes"
        else:
            reasons = []
            if not cond_mag:
                reasons.append("|dey|<thresh")
            if not cond_sig:
                reasons.append("not-significant")
            if not cond_tracks:
                reasons.append("lap-invariant(->P0d/P1b not P3)")
            verdict = "no (%s)" % ",".join(reasons)
        print("  Decision 1 -- P3_NEEDED: %s" % verdict)
    out["p3_needed"] = verdict
    return out


# ---------------------------------------------------------------------------
# CLI.
# ---------------------------------------------------------------------------

def build_argparser():
    p = argparse.ArgumentParser(
        description="Offline grey-box fitter for line-tracing lap captures "
                    "(P0c #167): estimates sensor c, process noise Q, "
                    "measurement noise R, and a residual-bias indicator, then "
                    "emits the P3-needed + kf_rmeas decisions. stdlib only.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument("caps", nargs="+", metavar="LAP.cap",
                   help="one or more lap .cap files")
    p.add_argument("--L", type=float, default=55.0,
                   help="look-ahead anchor [mm] (P0a ruler)")
    p.add_argument("--rail-lo", type=float, default=220.0,
                   help="exclude intensity <= N")
    p.add_argument("--rail-hi", type=float, default=1010.0,
                   help="exclude intensity >= N")
    p.add_argument("--speed-min", type=float, default=1.0,
                   help="require speed_mmps > N")
    p.add_argument("--use-cmd", action="store_true",
                   help="use turn_cmd_dps instead of achieved turn_rate_dps")
    p.add_argument("--straight-turn-thresh", type=float, default=15.0,
                   help="|turn| below this [dps] counts as straight")
    p.add_argument("--straight-min-ticks", type=int, default=25,
                   help="min consecutive straight ticks per run")
    p.add_argument("--min-straight-ticks-total", type=int, default=200,
                   help="refuse decisions below this many straight ticks")
    p.add_argument("--whole-lap", action="store_true",
                   help="ALSO fit the whole lap (diagnostic, CURVATURE_UNMODELED)")
    p.add_argument("--whole-lap-only", action="store_true",
                   help="ONLY fit the whole lap (diagnostic; skip straight fit)")
    p.add_argument("--target-filter", type=float, default=None,
                   help="fit only ticks with target==N")
    p.add_argument("--target-settle-ticks", type=int, default=20,
                   help="drop N ticks after any target change")
    p.add_argument("--burn-in-ticks", type=int, default=30,
                   help="exclude first N measurement updates from the NLL")
    p.add_argument("--max-coast-ticks", type=int, default=50,
                   help="clamp P to P0 after a longer predict-only run")
    p.add_argument("--edge-sign", choices=["neg", "pos"], default="neg",
                   help="sign in C=[-c,-cL]; fitted c stays a positive magnitude")
    p.add_argument("--p3-bias-thresh-mm", type=float, default=1.0,
                   help="P3 decision threshold [mm]")
    p.add_argument("--q-no-dt-scale", action="store_true",
                   help="add raw Q per predict (literal kf_q match) not Q*dt")
    p.add_argument("--p0-pos", type=float, default=100.0,
                   help="diffuse initial covariance P0[e_y] [mm^2]")
    p.add_argument("--p0-head", type=float, default=1.0,
                   help="diffuse initial covariance P0[e_theta] [rad^2]")
    p.add_argument("--dt-max", type=float, default=0.100,
                   help="reject ticks whose dt exceeds this [s]")
    p.add_argument("--min-fit-updates", type=int, default=30,
                   help="minimum measurement updates to attempt a fit")
    p.add_argument("--grid-only", action="store_true",
                   help="report grid optimum (skip Nelder-Mead) -- debug")
    p.add_argument("--json", action="store_true",
                   help="emit a machine-readable summary block")
    p.add_argument("--verbose", action="store_true",
                   help="per-tick innovation dump -- debug")
    return p


def main(argv=None):
    args = build_argparser().parse_args(argv)

    laps = []
    try:
        for path in args.caps:
            lap = fit_lap(path, args)
            report_lap(lap, args)
            laps.append(lap)
    except CapError as e:
        sys.stderr.write("ERROR: %s\n" % e)
        return 2

    summary = aggregate_and_decide(laps, args)

    if args.json:
        block = dict(
            decisions=summary,
            laps=[_lap_json(l, args) for l in laps],
        )
        print("--- JSON ---")
        print(json.dumps(block, indent=2, default=_json_default))

    return 0


def _json_default(o):
    if isinstance(o, float) and (math.isnan(o) or math.isinf(o)):
        return None
    return str(o)


def _lap_json(lap, args):
    def reg_json(r):
        if not r or r.get("status") != "OK":
            return dict(label=r.get("label") if r else None,
                        status=r.get("status") if r else None,
                        n_used=r.get("n_used") if r else None)
        return dict(
            label=r["label"], status=r["status"], n_used=r["n_used"],
            c=r["c"], c_label=r["c_label"], cL=r["cL"],
            c_band=r["profile"]["band"], c_ridge=r["profile"]["ridge"],
            c_flat=r["profile"]["flat"],
            R=r["R"], sqrt_R=r["sqrt_R"],
            q_pos=r["q_pos"], q_head=r["q_head"],
            rms_innov=r["rms_innov"], median_innov=r["median_innov"],
            bias_b=r["bias_b"], bias_dey_mm=r["bias_dey_mm"],
            bias_lr=r["bias_lr"], bias_significant=r["bias_significant"],
            at_bound=r["at_bound"],
        )
    return dict(
        path=lap["path"],
        records=lap["n_records"], ticks=lap["n_ticks"],
        rail_excluded=lap["n_rail_excluded"], rail_frac=lap["rail_frac"],
        moving=lap["n_moving"], straight=lap["n_straight"],
        targets=lap["targets"],
        dt_ms=dict(min=1e3 * lap["dt_min"], max=1e3 * lap["dt_max"],
                   mean=1e3 * lap["dt_mean"], median=1e3 * lap["dt_median"]),
        straight_status=lap.get("straight_status"),
        straight_used_total=lap["straight_used_total"],
        regimes=[reg_json(r) for r in lap["regimes"]],
        whole_lap=reg_json(lap["whole_lap"]) if lap["whole_lap"] else None,
        decision_regime=reg_json(lap["decision_regime"])
        if lap.get("decision_regime") else None,
    )


if __name__ == "__main__":
    sys.exit(main())
