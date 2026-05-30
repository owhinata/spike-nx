#!/usr/bin/env python3
"""Offline LQR-vs-PID design simulator for the spike-nx line follower.

Pre-plan design tool (Issue: LQR line-tracing investigation).  No firmware
is touched: this models the *outer* kinematic loop the line follower runs
(producing a turn-rate command that the drivebase tracks) and compares the
current fixed-gain PID against a closed-form, speed-scheduled LQR.

Stdlib only (no numpy/scipy/matplotlib on this host).

Model (line-relative frame, small-angle kinematics)
---------------------------------------------------
    state x = [e_y, e_theta]      e_y     cross-track error      [mm]
                                  e_theta heading error          [rad]
    dot e_y     = v * sin(e_theta)
    dot e_theta = omega           (omega = commanded turn rate, the input)

    A = [[0, v], [0, 0]]   B = [[0],[1]]   u = omega

Sensor (single reflected-intensity reading, MODE 5 intensity 0..1024)
---------------------------------------------------------------------
The sensor sits a look-ahead distance L ahead of the wheel axle, so it
measures a *blend* of position and heading:

    p   = e_y + L * sin(e_theta)                 lateral offset of the spot [mm]
    int = clamp(round(target + c*p + noise), 0, 1024)
    err = target - int   ==  -c * (e_y + L*sin e_theta)   (within the linear band)

  c [counts/mm] = intensity slope across the line edge  -- MUST be calibrated
  L [mm]        = sensor look-ahead distance            -- MUST be measured

The negative sign makes positive-gain feedback (firmware turn = +kp*err)
stabilising, which is also why a pure-P follower with L>0 is stable at all:
the look-ahead injects implicit heading damping.

LQR closed form (verified to machine precision against the CARE residual)
-------------------------------------------------------------------------
    K1 = sqrt(q1/r)                  gain on e_y      -- SPEED-INDEPENDENT
    K2 = sqrt(2*v*K1 + q2/r)         gain on e_theta  -- grows ~sqrt(v)
    omega = -K1*e_y - K2*e_theta
    closed-loop char poly: s^2 + K2*s + v*K1  =>  wn=sqrt(v*K1), zeta=K2/(2*wn)

With q2 = 0 this yields zeta == 1/sqrt(2) = 0.7071 at EVERY speed, with
bandwidth wn scaling as sqrt(v).  A fixed-gain PID can only hit 0.707 at one
speed; off that speed its damping drifts.  That is the core LQR win for a
robot that already varies v (the dynamic-speed feature, v=150..300 mm/s).

Tier-1 implementable law (measured-signal coordinates, no e_theta sensor)
-------------------------------------------------------------------------
    omega = Gp*err + Gd(v)*d(err)/dt
    Gp    = K1/c                 (constant)
    Gd(v) = K2/(c*v)  ->  with q2=0, Gd = sqrt(2*K1)/(c*sqrt(v))  ~ 1/sqrt(v)

i.e. a speed-scheduled PD whose P/D ratio and schedule shape come from LQR
theory, anchored by a single calibration (c) and the look-ahead (L).
"""

import argparse
import math
import os
import sys

PI = math.pi
DEG = 180.0 / PI  # rad/s -> deg/s


# --------------------------------------------------------------------------
# fixed-point / C-arithmetic helpers (to replicate the firmware faithfully)
# --------------------------------------------------------------------------
def c_div(a, b):
    """C integer division: truncate toward zero (Python // floors)."""
    q = abs(int(a)) // abs(int(b))
    return q if (a < 0) == (b < 0) else -q


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


# --------------------------------------------------------------------------
# LQR closed form + CARE residual self-check
# --------------------------------------------------------------------------
def lqr_gains(v, q1, q2, r):
    """Closed-form stabilising LQR gains for A=[[0,v],[0,0]], B=[[0],[1]]."""
    K1 = math.sqrt(q1 / r)
    K2 = math.sqrt(2.0 * v * K1 + q2 / r)
    return K1, K2


def care_residual_max(v, q1, q2, r):
    """Max |entry| of A'P+PA - P B R^-1 B' P + Q for the closed-form P.

    Should be ~1e-12 (machine precision) -- proves the closed form is the
    exact stabilising Riccati solution.
    """
    p12 = math.sqrt(r * q1)
    p22 = math.sqrt(r * (2.0 * v * p12 + q2))
    p11 = p12 * p22 / (r * v)
    r11 = -p12 * p12 / r + q1
    r12 = v * p11 - p12 * p22 / r
    r22 = 2.0 * v * p12 - p22 * p22 / r + q2
    return max(abs(r11), abs(r12), abs(r22))


def closed_loop(v, K1, K2):
    wn = math.sqrt(v * K1)
    zeta = K2 / (2.0 * wn) if wn > 0 else float("inf")
    return wn, zeta


# --------------------------------------------------------------------------
# effective physical 2nd-order gains (for the analytic zeta(v) comparison)
#
#   omega = -G1*e_y - G2*e_theta   ->   s^2 + G2 s + v G1,  zeta=G2/(2 sqrt(v G1))
#
# Both controllers, after substituting the look-ahead sensor model
#   err      ~ -c (e_y + L e_theta)
#   d err/dt ~ -c (v e_theta)              (drop the small L*omega term)
# pick up a look-ahead heading term G2_lookahead = (P-gain)*c*L.
# --------------------------------------------------------------------------
def pid_effective_gains(v, kp, kd, c, L):
    # firmware: omega[deg/s] = kp*err + kd*(d err/dt);  p_term=kp*err, d_term=kd*derr/dt
    # convert to rad/s control authority (model units) via /DEG
    g1 = (kp * c) / DEG                 # on e_y      (const)
    g2 = (kp * c * L + kd * c * v) / DEG  # on e_theta (lookahead + D term)
    return g1, g2


def lqr_effective_gains(v, K1, K2, c, L):
    # Tier-1 law omega = Gp err + Gd derr/dt with Gp=K1/c, Gd=K2/(c v) gives,
    # after the same substitution, G1=K1, G2 = K1*L + K2 (look-ahead adds K1*L).
    g1 = K1
    g2 = K1 * L + K2
    return g1, g2


def zeta_of(v, g1, g2):
    wn = math.sqrt(v * g1) if g1 > 0 else 0.0
    return (g2 / (2.0 * wn) if wn > 0 else float("inf")), wn


# --------------------------------------------------------------------------
# controllers (discrete, run at hz)
# --------------------------------------------------------------------------
class FirmwarePID:
    """Faithful port of apps/linetrace/linetrace_main.c control law."""

    def __init__(self, kp_x100, ki_x100, kd_x100, target, hz, speed_mmps):
        self.kp_x100 = kp_x100
        self.ki_x100 = ki_x100
        self.kd_x100 = kd_x100
        self.target = target
        self.hz = hz
        self.speed = speed_mmps
        self.i_acc = 0
        self.prev_err = 0
        self.has_prev = False

    def step(self, intensity, v=None):
        err = self.target - intensity
        dt_ms = 1000 // self.hz
        p = c_div(self.kp_x100 * err, 100)

        self.i_acc += err * dt_ms
        i = 0
        if self.ki_x100 != 0:
            ki_abs = abs(self.ki_x100)
            ilim = c_div(self.speed * 100 * 1000, ki_abs)
            self.i_acc = clamp(self.i_acc, -ilim, ilim)
            i = c_div(self.ki_x100 * self.i_acc, 100 * 1000)
        else:
            self.i_acc = 0

        derr = (err - self.prev_err) if self.has_prev else 0
        d = c_div(self.kd_x100 * derr * 10, dt_ms) if (self.has_prev and dt_ms > 0) else 0
        self.prev_err = err
        self.has_prev = True

        max_turn = self.speed if self.speed > 0 else 0
        s = p + i + d
        turn = clamp(s, -max_turn, max_turn)
        return turn, dict(p=p, i=i, d=d, err=err, derr=derr)


# --------------------------------------------------------------------------
# tiny 2x2 linear algebra (stdlib only) for the LQG observer
# --------------------------------------------------------------------------
def mm(A, B):
    return [[A[0][0] * B[0][0] + A[0][1] * B[1][0], A[0][0] * B[0][1] + A[0][1] * B[1][1]],
            [A[1][0] * B[0][0] + A[1][1] * B[1][0], A[1][0] * B[0][1] + A[1][1] * B[1][1]]]


def mt(A):
    return [[A[0][0], A[1][0]], [A[0][1], A[1][1]]]


def madd(A, B):
    return [[A[0][0] + B[0][0], A[0][1] + B[0][1]], [A[1][0] + B[1][0], A[1][1] + B[1][1]]]


def kalman_steady_gain(v, dt, c, L, q_pos, q_head, r_meas, iters=2000):
    """Steady-state discrete Kalman gain for the 2-state model with the
    look-ahead measurement err = C x, C = [-c, -c L].  Iterates the Riccati
    difference equation to convergence (cheap for a 2-state system)."""
    Ad = [[1.0, v * dt], [0.0, 1.0]]
    AdT = mt(Ad)
    Cc = [-c, -c * L]                      # measurement row
    Q = [[q_pos, 0.0], [0.0, q_head]]
    P = [[1.0, 0.0], [0.0, 1.0]]
    kf_prev = None
    for _ in range(iters):
        Pp = madd(mm(mm(Ad, P), AdT), Q)   # P_pred = Ad P Ad' + Q
        # S = C Pp C' + R  (scalar)
        PpCt = [Pp[0][0] * Cc[0] + Pp[0][1] * Cc[1],
                Pp[1][0] * Cc[0] + Pp[1][1] * Cc[1]]
        S = Cc[0] * PpCt[0] + Cc[1] * PpCt[1] + r_meas
        Kf = [PpCt[0] / S, PpCt[1] / S]    # Kalman gain (2x1)
        # P = (I - Kf C) Pp
        KC = [[Kf[0] * Cc[0], Kf[0] * Cc[1]], [Kf[1] * Cc[0], Kf[1] * Cc[1]]]
        ImKC = [[1.0 - KC[0][0], -KC[0][1]], [-KC[1][0], 1.0 - KC[1][1]]]
        P = mm(ImKC, Pp)
        if kf_prev and abs(Kf[0] - kf_prev[0]) + abs(Kf[1] - kf_prev[1]) < 1e-15:
            break
        kf_prev = Kf
    return Kf


class LqgObserver:
    """Tier-3 LQG: a Kalman observer reconstructs (e_y, e_theta) from the
    look-ahead intensity error and the KNOWN turn-rate command, then a clean
    speed-scheduled state-feedback law u = -K1*e_y - K2*e_theta is applied.

    Unlike Tier-1, this never differentiates err, so it is immune to the
    L*omega corruption that breaks the naive PD at large look-ahead.  If a
    gyro yaw-rate were available (drivebase db_imu via GET_HEADING), it would
    feed the predict step directly and tighten the estimate further.
    """

    def __init__(self, c, L, q1, q2, r, hz, target,
                 kf_qpos=1.0, kf_qhead=1e-4, kf_rmeas=4.0):
        self.c = c
        self.L = L
        self.q1 = q1
        self.q2 = q2
        self.r = r
        self.hz = hz
        self.dt = 1.0 / hz
        self.target = target
        self.Cc = [-c, -c * L]
        self.kf_q = (kf_qpos, kf_qhead)
        self.kf_r = kf_rmeas
        self.xhat = [0.0, 0.0]            # [e_y_hat, e_theta_hat]
        self.prev_w = 0.0                 # last applied omega [rad/s]
        self._Kf = None
        self._v = None

    def _ensure_gain(self, v):
        if v != self._v:
            self._Kf = kalman_steady_gain(v, self.dt, self.c, self.L,
                                          self.kf_q[0], self.kf_q[1], self.kf_r)
            self._v = v

    def step(self, intensity, v):
        self._ensure_gain(v)
        dt = self.dt
        # 1. predict with the previously-applied command (ZOH)
        ey, eth = self.xhat
        ey_p = ey + v * math.sin(eth) * dt
        eth_p = eth + self.prev_w * dt
        # 2. correct with the measurement
        err = self.target - intensity                       # = C x (true) + noise
        yhat = self.Cc[0] * ey_p + self.Cc[1] * eth_p       # predicted measurement
        innov = err - yhat
        ey = ey_p + self._Kf[0] * innov
        eth = eth_p + self._Kf[1] * innov
        self.xhat = [ey, eth]
        # 3. clean state feedback (speed-scheduled LQR)
        K1, K2 = lqr_gains(v, self.q1, self.q2, self.r)
        omega = -K1 * ey - K2 * eth                         # rad/s
        turn = clamp(omega * DEG, -v, v)
        self.prev_w = math.radians(turn)
        return turn, dict(ey=ey, eth=math.degrees(eth), K1=K1, K2=K2)


class LqrPD:
    """Tier-1 speed-scheduled LQR-PD: omega = Gp*err + Gd(v)*d(err)/dt."""

    def __init__(self, c, q1, q2, r, hz, target):
        self.c = c
        self.q1 = q1
        self.q2 = q2
        self.r = r
        self.hz = hz
        self.target = target
        self.prev_err = None

    def step(self, intensity, v):
        err = self.target - intensity
        dt = 1.0 / self.hz
        derr_per_s = 0.0 if self.prev_err is None else (err - self.prev_err) / dt
        self.prev_err = err

        K1, K2 = lqr_gains(v, self.q1, self.q2, self.r)
        Gp = K1 / self.c
        Gd = K2 / (self.c * v)
        omega_rad = Gp * err + Gd * derr_per_s   # rad/s
        turn = clamp(omega_rad * DEG, -v, v)     # deg/s, clamp to +-speed
        return turn, dict(K1=K1, K2=K2, Gp=Gp, Gd=Gd, err=err)


# --------------------------------------------------------------------------
# closed-loop time-domain simulation (RK4 plant, ZOH controller)
# --------------------------------------------------------------------------
def simulate(ctrl, is_lqr, v, e_y0, e_th0_deg, L, c, target, hz, tsim,
             tau=0.0, noise_lsb=0.0, dist_bias=0.0, seed=1):
    dt_ctrl = 1.0 / hz
    nsub = 8
    h = dt_ctrl / nsub
    e_y = float(e_y0)
    e_th = math.radians(e_th0_deg)
    w_act = 0.0
    rng = _Lcg(seed)
    nticks = int(tsim * hz)
    rows = []

    def f2(ey, eth, w):
        return (v * math.sin(eth), w)

    def f3(ey, eth, w, w_cmd):
        return (v * math.sin(eth), w, (w_cmd - w) / tau)

    for _ in range(nticks):
        p = e_y + L * math.sin(e_th)
        intensity_real = target + c * p + dist_bias
        if noise_lsb > 0:
            intensity_real += rng.uniform(-noise_lsb, noise_lsb)
        intensity = int(clamp(round(intensity_real), 0, 1024))

        if is_lqr:
            turn_dps, _ = ctrl.step(intensity, v)
        else:
            turn_dps, _ = ctrl.step(intensity)
        w_cmd = math.radians(turn_dps)

        for _s in range(nsub):
            if tau > 0:
                k1 = f3(e_y, e_th, w_act, w_cmd)
                k2 = f3(e_y + h / 2 * k1[0], e_th + h / 2 * k1[1], w_act + h / 2 * k1[2], w_cmd)
                k3 = f3(e_y + h / 2 * k2[0], e_th + h / 2 * k2[1], w_act + h / 2 * k2[2], w_cmd)
                k4 = f3(e_y + h * k3[0], e_th + h * k3[1], w_act + h * k3[2], w_cmd)
                e_y += h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
                e_th += h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])
                w_act += h / 6 * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2])
            else:
                w_act = w_cmd
                k1 = f2(e_y, e_th, w_act)
                k2 = f2(e_y + h / 2 * k1[0], e_th + h / 2 * k1[1], w_act)
                k3 = f2(e_y + h / 2 * k2[0], e_th + h / 2 * k2[1], w_act)
                k4 = f2(e_y + h * k3[0], e_th + h * k3[1], w_act)
                e_y += h / 6 * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0])
                e_th += h / 6 * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1])

        rows.append((e_y, math.degrees(e_th), turn_dps, intensity))
    return rows


class _Lcg:
    """Tiny deterministic PRNG (host stdlib `random` would also work; this
    keeps results identical regardless of interpreter)."""

    def __init__(self, seed=1):
        self.s = seed & 0xFFFFFFFF

    def _next(self):
        self.s = (1103515245 * self.s + 12345) & 0x7FFFFFFF
        return self.s

    def uniform(self, a, b):
        return a + (b - a) * (self._next() / 0x7FFFFFFF)


def metrics(rows, hz, e_y0):
    dt = 1.0 / hz
    n = len(rows)
    iae = sum(abs(r[0]) for r in rows) * dt
    peak = max(abs(r[0]) for r in rows)
    peak_turn = max(abs(r[2]) for r in rows)
    # overshoot past the opposite side (only meaningful for a step from e_y0)
    if e_y0 > 0:
        ovr = max(0.0, -min(r[0] for r in rows))
    elif e_y0 < 0:
        ovr = max(0.0, max(r[0] for r in rows))
    else:
        ovr = peak
    ovr_pct = 100.0 * ovr / abs(e_y0) if e_y0 != 0 else float("nan")
    # 5% settling time (last time |e_y| leaves the 5%*|e_y0| band)
    band = 0.05 * abs(e_y0) if e_y0 != 0 else 0.5
    settle = None
    for k in range(n - 1, -1, -1):
        if abs(rows[k][0]) > band:
            settle = (k + 1) * dt
            break
    if settle is None:
        settle = 0.0
    ss = sum(r[0] for r in rows[int(0.9 * n):]) / max(1, n - int(0.9 * n))
    return dict(iae=iae, peak=peak, ovr_pct=ovr_pct, settle=settle,
                peak_turn=peak_turn, ss=ss)


# --------------------------------------------------------------------------
# ASCII plot
# --------------------------------------------------------------------------
def ascii_plot(series_list, hz, width=72, height=15, title=""):
    """series_list: [(label, [values])].  Overlays with distinct glyphs."""
    glyphs = "*o#+"
    allv = [v for _, ys in series_list for v in ys]
    if not allv:
        return ""
    lo, hi = min(allv), max(allv)
    if hi - lo < 1e-9:
        hi = lo + 1.0
    n = max(len(ys) for _, ys in series_list)
    grid = [[" "] * width for _ in range(height)]

    def yrow(val):
        f = (val - lo) / (hi - lo)
        return int(round((1 - f) * (height - 1)))

    for si, (_, ys) in enumerate(series_list):
        g = glyphs[si % len(glyphs)]
        for x in range(width):
            idx = int(x * (n - 1) / (width - 1)) if width > 1 else 0
            if idx < len(ys):
                grid[yrow(ys[idx])][clamp(x, 0, width - 1)] = g
    # zero line
    if lo < 0 < hi:
        zr = yrow(0.0)
        for x in range(width):
            if grid[zr][x] == " ":
                grid[zr][x] = "."
    out = []
    if title:
        out.append(title)
    for r, line in enumerate(grid):
        val = hi - (hi - lo) * r / (height - 1)
        out.append("%8.2f |%s" % (val, "".join(line)))
    out.append("%8s +%s" % ("", "-" * width))
    tspan = (n - 1) / hz
    out.append("%8s  0%ss%*.2fs" % ("", " " * (width - 8), 6, tspan))
    leg = "  ".join("%s=%s" % (glyphs[i % len(glyphs)], lbl)
                    for i, (lbl, _) in enumerate(series_list))
    out.append("         " + leg)
    return "\n".join(out)


# --------------------------------------------------------------------------
# reports
# --------------------------------------------------------------------------
def report_verify(args):
    print("== CARE closed-form self-check (residual must be ~1e-12) ==")
    for (v, q1, q2, r) in [(100, 1, 0, 1), (300, 4, 0, 1),
                           (300, 4, 2, 0.5), (150, 9, 1, 2)]:
        K1, K2 = lqr_gains(v, q1, q2, r)
        wn, z = closed_loop(v, K1, K2)
        res = care_residual_max(v, q1, q2, r)
        print("  v=%4d q1=%g q2=%g r=%g : K1=%.4f K2=%.4f  |res|=%.2e  wn=%.3f zeta=%.5f"
              % (v, q1, q2, r, K1, K2, res, wn, z))


def matched_q1r(kp, c):
    """q1/r that makes the LQR P-authority equal the firmware kp at the
    sensor level: Gp*DEG == kp  =>  K1 = kp*c/DEG  =>  q1/r = K1^2."""
    K1 = kp * c / DEG
    return K1 * K1


def report_damping(args):
    kp = args.kp
    kd = args.kd
    c = args.c
    L = args.L
    q1r = args.q1r if args.q1r is not None else matched_q1r(kp, c)
    q2 = args.q2
    r = 1.0
    q1 = q1r * r
    print("== Damping zeta(v): fixed-gain PID vs closed-form LQR ==")
    print("   sensor c=%.1f counts/mm  look-ahead L=%.1f mm" % (c, L))
    print("   PID kp=%.3f kd=%.3f   LQR q1/r=%.4g q2=%.4g (q2=0 -> zeta target 0.707)"
          % (kp, kd, q1r, q2))
    print("   %6s | %-22s | %-22s" % ("v", "PID", "LQR (Tier-1, w/ look-ahead)"))
    print("   %6s | %8s %8s %s | %8s %8s" % ("mm/s", "wn", "zeta", "", "wn", "zeta"))
    vs = args.speeds
    for v in vs:
        g1p, g2p = pid_effective_gains(v, kp, kd, c, L)
        zp, wnp = zeta_of(v, g1p, g2p)
        K1, K2 = lqr_gains(v, q1, q2, r)
        g1l, g2l = lqr_effective_gains(v, K1, K2, c, L)
        zl, wnl = zeta_of(v, g1l, g2l)
        flagp = "  <-under" if zp < 0.6 else ("  <-over" if zp > 0.95 else "")
        print("   %6d | %8.3f %8.4f%-6s | %8.3f %8.4f"
              % (v, wnp, zp, flagp, wnl, zl))
    print("   (PID zeta drifts with v; LQR holds ~0.707, slightly higher at low v")
    print("    because the un-modelled look-ahead adds K1*L of bonus damping.)")


def report_coeffs(args):
    kp = args.kp
    c = args.c
    q1r = args.q1r if args.q1r is not None else matched_q1r(kp, c)
    r = 1.0
    q1 = q1r * r
    q2 = args.q2
    K1 = math.sqrt(q1 / r)
    Gp = K1 / c                      # rad/s per count
    Gp_dps = Gp * DEG                # deg/s per count  (compare to kp)
    print("== Tier-1 firmware coefficients (q2=%g) ==" % q2)
    print("   chosen q1/r = %.6g   (r normalised to 1)" % q1r)
    print("   K1 = sqrt(q1/r)      = %.5f  (rad/s per mm, speed-independent)" % K1)
    print("   Gp = K1/c            = %.6f rad/s per count" % Gp)
    print("        = %.4f deg/s per count  (firmware-kp equivalent; baseline kp=%.3f)"
          % (Gp_dps, kp))
    print("   Gd(v) = K2/(c*v),  K2=sqrt(2*v*K1+q2/r)")
    if q2 == 0:
        Kd0 = math.sqrt(2.0 * K1) / c        # Gd = Kd0 / sqrt(v)
        print("        q2=0 -> Gd(v) = %.6f / sqrt(v)   (rad/s per count/s)" % Kd0)
        print("                     = %.4f / sqrt(v) deg/s per (count/s)" % (Kd0 * DEG))
    print("   --- per-speed schedule ---")
    print("   %6s %10s %10s %10s %10s" % ("v", "K2", "Gd", "wn", "zeta"))
    for v in args.speeds:
        K1v, K2v = lqr_gains(v, q1, q2, r)
        Gd = K2v / (c * v)
        wn, z = closed_loop(v, K1v, K2v)
        print("   %6d %10.4f %10.6f %10.3f %10.4f" % (v, K2v, Gd, wn, z))
    print("   suggested config keys: lqr_q1r_x1000=%d  lqr_q2_x1000=%d  lqr_c_x100=%d  lqr_lookahead_um=%d"
          % (round(q1r * 1000), round(q2 * 1000), round(c * 100), round(L_to_um(args.L))))


def L_to_um(L_mm):
    return L_mm * 1000.0


def report_step(args):
    kp = args.kp
    ki = args.ki
    kd = args.kd
    c = args.c
    L = args.L
    target = args.target
    hz = args.hz
    q1r = args.q1r if args.q1r is not None else matched_q1r(kp, c)
    q2 = args.q2
    r = 1.0
    q1 = q1r * r

    print("== Step response: %g mm initial cross-track, %s ==" % (
        args.e_y0,
        "ideal inner loop" if args.tau == 0 else "inner-loop tau=%gs" % args.tau))
    print("   c=%.1f L=%.1f hz=%d noise=%gLSB dist_bias=%g counts" % (
        c, L, hz, args.noise, args.dist))
    hdr = "   %-16s %6s | %8s %8s %8s %8s %9s" % (
        "controller", "v", "IAE", "peak", "ovr%", "settle_s", "peakturn")
    for v in args.speeds:
        print("   --- v = %d mm/s ---" % v)
        print(hdr)
        # PID (faithful fixed-point)
        pid = FirmwarePID(round(kp * 100), round(ki * 100), round(kd * 100),
                          target, hz, v)
        rp = simulate(pid, False, v, args.e_y0, args.e_th0, L, c, target, hz,
                      args.tsim, args.tau, args.noise, args.dist)
        mp = metrics(rp, hz, args.e_y0)
        print("   %-16s %6d | %8.1f %8.2f %8.1f %8.3f %9.1f"
              % ("PID(fixed)", v, mp["iae"], mp["peak"], mp["ovr_pct"],
                 mp["settle"], mp["peak_turn"]))
        # LQR-PD (Tier-1, naive derivative)
        lqr = LqrPD(c, q1, q2, r, hz, target)
        rl = simulate(lqr, True, v, args.e_y0, args.e_th0, L, c, target, hz,
                      args.tsim, args.tau, args.noise, args.dist)
        ml = metrics(rl, hz, args.e_y0)
        print("   %-16s %6d | %8.1f %8.2f %8.1f %8.3f %9.1f"
              % ("LQR-PD(T1)", v, ml["iae"], ml["peak"], ml["ovr_pct"],
                 ml["settle"], ml["peak_turn"]))
        # LQG observer (Tier-3, robust to look-ahead)
        lqg = LqgObserver(c, L, q1, q2, r, hz, target,
                          kf_rmeas=max(1.0, args.noise * args.noise * 4))
        rg = simulate(lqg, True, v, args.e_y0, args.e_th0, L, c, target, hz,
                      args.tsim, args.tau, args.noise, args.dist)
        mg = metrics(rg, hz, args.e_y0)
        print("   %-16s %6d | %8.1f %8.2f %8.1f %8.3f %9.1f"
              % ("LQG(T3)", v, mg["iae"], mg["peak"], mg["ovr_pct"],
                 mg["settle"], mg["peak_turn"]))
        if args.dist != 0:
            print("       steady-state e_y: PID=%.3f  LQR-PD=%.3f  LQG=%.3f mm  (PID I-term & LQG bias-state remove offset; bare LQR/LQI needed for LQG)"
                  % (mp["ss"], ml["ss"], mg["ss"]))
        if args.plot:
            print(ascii_plot([("PID", [row[0] for row in rp]),
                              ("LQRpd", [row[0] for row in rl]),
                              ("LQG", [row[0] for row in rg])], hz,
                             title="       e_y(t) [mm] @ v=%d" % v))
        if args.csv:
            _write_csv(args.csv_dir, "step_v%d_pid.csv" % v, rp, hz)
            _write_csv(args.csv_dir, "step_v%d_lqrpd.csv" % v, rl, hz)
            _write_csv(args.csv_dir, "step_v%d_lqg.csv" % v, rg, hz)


def _write_csv(d, name, rows, hz):
    os.makedirs(d, exist_ok=True)
    path = os.path.join(d, name)
    with open(path, "w") as f:
        f.write("t_s,e_y_mm,e_theta_deg,turn_dps,intensity\n")
        for k, row in enumerate(rows):
            f.write("%.5f,%.5f,%.5f,%.3f,%d\n" % (k / hz, row[0], row[1], row[2], row[3]))
    print("       wrote %s" % path)


def report_dump_kf(args):
    """Dump the 24-point steady-state Kalman gain table for the C
    cross-check (Issue #169 / P1a).  The firmware builds the same table
    with lqg_build_kf_table() using the SIGNED c (edge*c), so this dumps
    BOTH edges and shows that the LEFT-edge (edge=-1) Kf is the
    sign-flipped RIGHT-edge (edge=+1) Kf — validating the signed-c table
    build (plan §3 / §8).

    Grid + Riccati params mirror the firmware compile constants:
      LQG_KF_PTS=24, LQG_V_LO=50, LQG_V_HI=400.
    """
    pts = args.kf_pts
    v_lo = args.kf_v_lo
    v_hi = args.kf_v_hi
    c = args.c
    L = args.L
    hz = args.hz
    dt = 1.0 / hz
    qpos = args.kf_qpos
    qhead = args.kf_qhead
    rmeas = args.kf_rmeas
    print("== Kf steady-state Kalman gain table (C cross-check) ==")
    print("   pts=%d v_lo=%g v_hi=%g  c=%g L=%g hz=%d dt=%g" % (
        pts, v_lo, v_hi, c, L, hz, dt))
    print("   qpos=%g qhead=%g rmeas=%g" % (qpos, qhead, rmeas))
    for edge in (+1, -1):
        c_signed = edge * c
        print("   --- edge=%+d  (c_signed=%+g) ---" % (edge, c_signed))
        print("   %4s %10s %14s %14s" % ("i", "v", "kf0", "kf1"))
        for i in range(pts):
            v = v_lo + (v_hi - v_lo) * float(i) / float(pts - 1)
            kf = kalman_steady_gain(v, dt, c_signed, L, qpos, qhead, rmeas)
            print("   %4d %10.4f %14.8e %14.8e" % (i, v, kf[0], kf[1]))
    # Explicit LEFT == -RIGHT spot-check at the table endpoints + midpoint.
    print("   --- edge sign-flip check (LEFT == -RIGHT) ---")
    for i in (0, pts // 2, pts - 1):
        v = v_lo + (v_hi - v_lo) * float(i) / float(pts - 1)
        kr = kalman_steady_gain(v, dt, +c, L, qpos, qhead, rmeas)
        kl = kalman_steady_gain(v, dt, -c, L, qpos, qhead, rmeas)
        d0 = abs(kr[0] + kl[0])
        d1 = abs(kr[1] + kl[1])
        print("   v=%8.3f  |kf0_R+kf0_L|=%.2e  |kf1_R+kf1_L|=%.2e" % (
            v, d0, d1))


def report_sweep(args):
    kp = args.kp
    c = args.c
    L = args.L
    target = args.target
    hz = args.hz
    q2 = args.q2
    r = 1.0
    matched = matched_q1r(kp, c)
    print("== q1/r sweep (q2=%g): step IAE/overshoot at low & high speed ==" % q2)
    print("   look-ahead L=%.1f mm, e_y0=%g mm; matched-P q1/r=%.4g" % (L, args.e_y0, matched))
    vlo, vhi = args.speeds[0], args.speeds[-1]
    factors = [0.25, 0.5, 1.0, 2.0, 4.0]
    print("   %12s %10s | %8s %8s %8s | %8s %8s %8s" % (
        "q1/r", "(xmatched)", "IAE@lo", "ovr%@lo", "zeta@lo", "IAE@hi", "ovr%@hi", "zeta@hi"))
    for fac in factors:
        q1r = matched * fac
        q1 = q1r * r
        out = []
        for v in (vlo, vhi):
            lqr = LqrPD(c, q1, q2, r, hz, target)
            rl = simulate(lqr, True, v, args.e_y0, args.e_th0, L, c, target, hz,
                          args.tsim, args.tau, args.noise, args.dist)
            ml = metrics(rl, hz, args.e_y0)
            K1, K2 = lqr_gains(v, q1, q2, r)
            g1l, g2l = lqr_effective_gains(v, K1, K2, c, L)
            zl, _ = zeta_of(v, g1l, g2l)
            out.append((ml["iae"], ml["ovr_pct"], zl))
        print("   %12.5g %10.2fx | %8.1f %8.1f %8.4f | %8.1f %8.1f %8.4f" % (
            q1r, fac, out[0][0], out[0][1], out[0][2],
            out[1][0], out[1][1], out[1][2]))
    print("   higher q1/r -> stiffer (smaller IAE, faster) but larger peak turn-rate;")
    print("   pick the largest q1/r whose peak turn-rate stays within +-v at your top speed.")


# --------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--report", default="all",
                   choices=["all", "verify", "damping", "step", "sweep",
                            "coeffs", "dump-kf"])
    # plant / sensor (CALIBRATE these against hardware)
    p.add_argument("--c", type=float, default=60.0, help="intensity slope [counts/mm] (CALIBRATE)")
    p.add_argument("--L", type=float, default=40.0, help="sensor look-ahead [mm] (MEASURE)")
    # baseline PID (apps/linetrace defaults, Issue #126)
    p.add_argument("--kp", type=float, default=0.36)
    p.add_argument("--ki", type=float, default=0.15)
    p.add_argument("--kd", type=float, default=0.01)
    p.add_argument("--target", type=int, default=512)
    p.add_argument("--hz", type=int, default=200)
    # LQR weights
    p.add_argument("--q1r", type=float, default=None, help="q1/r (default: matched-P to baseline kp)")
    p.add_argument("--q2", type=float, default=0.0, help="heading weight (0 -> zeta=0.707 all speeds)")
    # scenario
    p.add_argument("--speeds", type=int, nargs="+", default=[150, 300])
    p.add_argument("--e_y0", type=float, default=20.0, help="initial cross-track error [mm]")
    p.add_argument("--e_th0", type=float, default=0.0, help="initial heading error [deg]")
    p.add_argument("--tsim", type=float, default=2.0, help="sim duration [s]")
    p.add_argument("--tau", type=float, default=0.0, help="inner-loop (drivebase) 1st-order lag [s]")
    p.add_argument("--noise", type=float, default=1.0, help="sensor noise [+-LSB]")
    p.add_argument("--dist", type=float, default=0.0, help="steady intensity bias [counts] (tests I-term need)")
    # Kf table dump (Issue #169 / P1a C cross-check; mirrors firmware
    # LQG_KF_PTS / LQG_V_LO / LQG_V_HI compile constants + kf_q*/kf_r*).
    p.add_argument("--kf-pts", type=int, default=24, help="Kf table points (LQG_KF_PTS)")
    p.add_argument("--kf-v-lo", type=float, default=50.0, help="Kf table v lower bound (LQG_V_LO)")
    p.add_argument("--kf-v-hi", type=float, default=400.0, help="Kf table v upper bound (LQG_V_HI)")
    p.add_argument("--kf-qpos", type=float, default=1.0, help="Kalman Q[0][0]")
    p.add_argument("--kf-qhead", type=float, default=1e-4, help="Kalman Q[1][1]")
    p.add_argument("--kf-rmeas", type=float, default=49.0, help="Kalman R (scalar)")
    # output
    p.add_argument("--plot", action="store_true", help="ASCII overlay of e_y(t)")
    p.add_argument("--csv", action="store_true", help="dump trajectory CSVs")
    p.add_argument("--csv-dir", default=os.path.join(os.path.dirname(__file__), "out"))
    return p


def main(argv):
    args = build_parser().parse_args(argv)
    rep = args.report
    print("# spike-nx line-follower LQR design sim")
    print("# WARNING: c (counts/mm) and L (mm) are PLACEHOLDERS until calibrated"
          " -- absolute gains depend on them; the zeta(v) *shape* does not.\n")
    if rep in ("all", "verify"):
        report_verify(args); print()
    if rep in ("all", "damping"):
        report_damping(args); print()
    if rep in ("all", "coeffs"):
        report_coeffs(args); print()
    if rep in ("all", "step"):
        if rep == "all":
            args.plot = True
        report_step(args); print()
    if rep in ("all", "sweep"):
        report_sweep(args); print()
    if rep == "dump-kf":
        report_dump_kf(args); print()
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
