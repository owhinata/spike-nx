#!/usr/bin/env python3
"""Measure the line-sensor model parameters c and L for the LQR design sim.

The line-follower sensor model is  err = -c * (e_y + L * sin theta), where
  c [counts/mm]  intensity slope across the line edge (optical response)
  L [mm]         sensor look-ahead distance (axle-center -> spot, forward)
Both feed lqr_sim.py (--c / --L) and the LQG observer matrix C = [-c, -c L].

Two independent measurements pin down all three of {c, L, c*L} (c*L = c x L):

  L     ruler, once: distance from the wheel-axle center to the sensor spot,
        projected on the robot's forward axis.  Pass it with --L.

  c*L   ROTATION SWEEP (this script, --mode sweep): pivot the robot in place;
        the look-ahead spot moves laterally by L*sin(theta), so
        d(intensity)/d(theta) near the setpoint = c*L.  Robust to a lateral
        sensor-mount offset (that only shifts where intensity==target, not the
        slope).  c is then (c*L)/L.

  c     MANUAL JOG fallback (--mode fit-csv): slide the robot sideways by hand
        in known mm steps, record (offset_mm,intensity); the slope is c
        directly (no L needed).  Use this to cross-check the sweep.

Firmware is NOT modified -- this drives existing NSH verbs over serial:
  sensor color watch <ms>      -> RGBI rows; intensity = 4th data value (0..1024)
  drivebase turn <deg> <dps> hold   in-place pivot, holds between samples
  drivebase get-state 0        -> angle_mdeg (achieved heading x1000)
  linetrace stop               -> release the color-sensor CLAIM first

Reuses tests/conftest.py NuttxSerial.sendCommand (marker-synchronised).
Run the live sweep on the host wired to the Hub; the fit/self-test run anywhere.
"""

import argparse
import math
import os
import sys
import time

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))


# --------------------------------------------------------------------------
# NSH output parsers (kept pure so they can be self-tested without hardware)
# --------------------------------------------------------------------------
def parse_color_watch(text):
    """Extract intensity (4th RGBI data value) from `sensor color watch`.

    Row: 'time_ms port gen seq mode type nval R G B I'.  Returns [int,...].
    """
    out = []
    for line in text.splitlines():
        t = line.split()
        if len(t) < 8 or "time_ms" in line:
            continue
        # locate the data-type token, then nval, then nval data values
        ti = None
        for i, tok in enumerate(t):
            if tok in ("INT8", "INT16", "INT32", "FLOAT"):
                ti = i
                break
        if ti is None or ti + 1 >= len(t):
            continue
        try:
            nval = int(t[ti + 1])
            data = t[ti + 2:ti + 2 + nval]
            if nval >= 4 and len(data) >= 4:
                out.append(int(float(data[3])))   # 4th channel = intensity
        except (ValueError, IndexError):
            continue
    return out


def parse_get_state_angle_mdeg(text):
    """Extract angle_mdeg from `drivebase get-state`.  Returns int or None.

    Row: 'time_ms dist_mm v_mmps angle_mdeg tr_dps done stall cmd tick afault'.
    Takes the last data row (most recent sample).
    """
    val = None
    for line in text.splitlines():
        if "angle_mdeg" in line or "----" in line:
            continue
        t = line.split()
        if len(t) < 4:
            continue
        try:
            # heuristic: a data row starts with an int time_ms and has an int 4th col
            int(t[0])
            val = int(t[3])
        except ValueError:
            continue
    return val


def parse_get_state(text):
    """Return (angle_mdeg, done, stall) from `drivebase get-state`.

    Columns: time_ms dist_mm v_mmps angle_mdeg tr_dps done stall cmd tick afault.
    Returns (None,None,None) if no data row is found.
    """
    res = (None, None, None)
    for line in text.splitlines():
        if "angle_mdeg" in line or "----" in line:
            continue
        t = line.split()
        if len(t) < 7:
            continue
        try:
            int(t[0])
            res = (int(t[3]), int(t[5]), int(t[6]))
        except (ValueError, IndexError):
            continue
    return res


# --------------------------------------------------------------------------
# least-squares line fit (stdlib)
# --------------------------------------------------------------------------
def linregress(xs, ys):
    n = len(xs)
    if n < 2:
        return 0.0, 0.0, 0.0
    mx = sum(xs) / n
    my = sum(ys) / n
    sxx = sum((x - mx) ** 2 for x in xs)
    sxy = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    if sxx == 0:
        return 0.0, my, 0.0
    slope = sxy / sxx
    intc = my - slope * mx
    syy = sum((y - my) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intc)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / syy if syy > 0 else 1.0
    return slope, intc, r2


def select_linear_band(xs, ys, slope_frac):
    """Auto-detect the linear edge region: the longest contiguous run of
    points (ordered by angle/offset) whose local slope magnitude is
    >= slope_frac * peak slope.  This excludes the flat saturation shoulders
    of the sigmoid AND any stuck-sample cluster (slope ~ 0), so the fit lands
    on the true linear band without hand-tuning an intensity window."""
    pts = sorted(zip(xs, ys))
    n = len(pts)
    if n < 4:
        return list(pts)
    slope = [0.0] * n
    for i in range(1, n - 1):
        dx = pts[i + 1][0] - pts[i - 1][0]
        slope[i] = abs((pts[i + 1][1] - pts[i - 1][1]) / dx) if abs(dx) > 1e-6 else 0.0
    slope[0], slope[-1] = slope[1], slope[-2]
    speak = max(slope)
    if speak <= 0:
        return list(pts)
    thr = slope_frac * speak
    mask = [s >= thr for s in slope]
    # longest contiguous True run
    best_s, best_e, cur = -1, -2, None
    for i in range(n):
        if mask[i] and cur is None:
            cur = i
        if cur is not None and (not mask[i] or i == n - 1):
            end = i if (mask[i] and i == n - 1) else i - 1
            if end - cur > best_e - best_s:
                best_s, best_e = cur, end
            cur = None
    if best_s < 0:
        return list(pts)
    sel = pts[best_s:best_e + 1]
    return sel if len(sel) >= 2 else list(pts)


# --------------------------------------------------------------------------
# ASCII scatter (intensity vs x)
# --------------------------------------------------------------------------
def ascii_scatter(xs, ys, sel_x, xlabel, width=64, height=16, title=""):
    lo_x, hi_x = min(xs), max(xs)
    lo_y, hi_y = min(ys), max(ys)
    if hi_x - lo_x < 1e-9:
        hi_x = lo_x + 1
    if hi_y - lo_y < 1e-9:
        hi_y = lo_y + 1
    grid = [[" "] * width for _ in range(height)]
    sel_set = set(round(x, 6) for x in sel_x)
    for x, y in zip(xs, ys):
        cx = int(round((x - lo_x) / (hi_x - lo_x) * (width - 1)))
        cy = int(round((1 - (y - lo_y) / (hi_y - lo_y)) * (height - 1)))
        grid[cy][cx] = "#" if round(x, 6) in sel_set else "."
    out = []
    if title:
        out.append(title)
    for r, row in enumerate(grid):
        yv = hi_y - (hi_y - lo_y) * r / (height - 1)
        out.append("%6.0f |%s" % (yv, "".join(row)))
    out.append("%6s +%s" % ("", "-" * width))
    out.append("%6s  %-*.2f%6.2f  (%s)  [#=fit band]" % ("", width - 6, lo_x, hi_x, xlabel))
    return "\n".join(out)


# --------------------------------------------------------------------------
# fit a sweep/jog sample set -> c, c*L, band
# --------------------------------------------------------------------------
def fit_and_report(angle_or_offset, intensity, noise_std, L, x_is_angle,
                   central_frac, target):
    if x_is_angle:
        xs = angle_or_offset                      # degrees
        unit = "deg"
    else:
        xs = angle_or_offset                      # mm (manual jog)
        unit = "mm"
    sel = select_linear_band(xs, intensity, central_frac)
    sx = [p[0] for p in sel]
    sy = [p[1] for p in sel]
    slope, intc, r2 = linregress(sx, sy)

    print("\n== Fit (central %.0f%% of intensity swing, %d/%d points) ==" % (
        central_frac * 100, len(sel), len(xs)))
    print("   intensity swing: %d .. %d  (target=%d)" % (
        round(min(intensity)), round(max(intensity)), target))
    print("   fit: intensity = %.3f * %s + %.1f   (R^2=%.4f)" % (slope, unit, intc, r2))
    if r2 < 0.985:
        print("   WARNING: R^2 < 0.985 -- band may be too wide (lower --central-frac)"
              " or sweep not centred on the edge.")

    result = {"r2": r2}
    if x_is_angle:
        cL = abs(slope) * 180.0 / math.pi          # counts/rad  (= d int/d theta)
        print("   |slope| = %.4f counts/deg  ->  c*L = %.3f counts/rad" % (abs(slope), cL))
        result["cL"] = cL
        if L:
            c = cL / L
            band_deg = (max(sx) - min(sx))
            band_mm = 2 * L * math.sin(math.radians(band_deg / 2.0))
            print("   with L = %.1f mm:  c = c*L / L = %.3f counts/mm" % (L, c))
            print("   linear band ~ %.1f deg  =  %.2f mm lateral" % (band_deg, band_mm))
            result["c"] = c
            result["L"] = L
        else:
            print("   (pass --L <mm> to convert c*L -> c; or use --mode fit-csv for c directly)")
    else:
        c = abs(slope)                              # counts/mm directly
        band_mm = max(sx) - min(sx)
        print("   c = %.3f counts/mm   linear band ~ %.2f mm" % (c, band_mm))
        result["c"] = c
        if L:
            result["L"] = L
            result["cL"] = c * L

    if noise_std is not None:
        print("   sensor noise (mean per-angle std): %.2f counts  -> sim --noise %.1f" % (
            noise_std, max(1.0, noise_std)))
        result["noise"] = noise_std
    return result, sx, sy


def write_params(result, path):
    with open(path, "w") as f:
        f.write("# line-sensor model params for tools/lqr_linetrace/lqr_sim.py\n")
        for k in ("c", "L", "cL", "noise"):
            if k in result:
                f.write("%s=%.4f\n" % (k, result[k]))
    print("\n   wrote %s" % path)
    if "c" in result and "L" in result:
        print("   feed the sim:  python3 lqr_sim.py --report all --c %.2f --L %.2f%s"
              % (result["c"], result["L"],
                 " --noise %.0f" % result["noise"] if "noise" in result else ""))


# --------------------------------------------------------------------------
# live rotation sweep (needs the Hub)
# --------------------------------------------------------------------------
def run_sweep(args):
    sys.path.insert(0, os.path.join(REPO, "tests"))
    from conftest import NuttxSerial, PROMPT     # noqa: lazy import (needs pyserial+HW)

    ser = NuttxSerial(args.device, os.path.join(os.path.dirname(__file__), "logs"))

    def cmd(s, to=15):
        return ser.sendCommand(s, timeout=to)

    def read_state():
        return parse_get_state(cmd("drivebase get-state 0"))

    def wait_settle(timeout):
        """`drivebase turn` is ASYNCHRONOUS (enqueues, returns immediately) --
        poll get-state until done==1 and the angle has stabilised, so we never
        sample mid-turn or stack relative turns that fight each other."""
        time.sleep(0.15)                         # let the daemon pick up the cmd
        prev = None
        stalled = False
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            ang, done, stall = read_state()
            if ang is None:
                time.sleep(0.1)
                continue
            if stall:
                stalled = True
            if done == 1 and prev is not None and abs(ang - prev) <= args.settle_tol_mdeg:
                return ang, stalled
            prev = ang
            time.sleep(0.1)
        return prev, stalled

    def read_intensity():
        vals = parse_color_watch(cmd("sensor color watch %d" % args.watch_ms))
        if not vals:
            return None, None, 0
        mean = sum(vals) / len(vals)
        var = sum((v - mean) ** 2 for v in vals) / len(vals)
        return mean, math.sqrt(var), len(vals)

    try:
        # NuttxSerial.__init__ only opens the port -- it does NOT sync the
        # prompt or send `set +e` (that lives in reconnect()/the pytest `p`
        # fixture).  Without `set +e`, NSH aborts a `cmd ; echo MARKER` line
        # as soon as `cmd` returns non-zero (e.g. `linetrace stop` when the
        # daemon is idle), so the marker never prints and sendCommand times
        # out.  Replicate the fixture's init exactly.
        for attempt in range(3):
            try:
                ser.clean_buffer()
                ser.proc.send("\r\n")
                time.sleep(0.1)
                ser.proc.send("\r\n")
                ser.proc.expect(PROMPT, timeout=5)
                break
            except Exception:
                if attempt == 2:
                    raise
        ser.proc.sendline("set +e")
        ser.proc.expect(PROMPT, timeout=5)
        print("# rotation sweep: theta in [%d, %d] step %d deg @ %d dps" % (
            -args.theta_max, args.theta_max, args.theta_step, args.turn_dps))
        print("# place the sensor on the line EDGE (intensity ~= target) before starting.\n")
        cmd("linetrace stop")                       # release sensor CLAIM if held
        cmd("drivebase start")                      # ensure daemon (EALREADY is fine)
        cmd("drivebase reset 0 0")

        # preflight: confirm the sensor is on the edge before sweeping
        c0, _, _ = read_intensity()
        if c0 is not None:
            print("  preflight intensity = %.0f  (want it near target=%d; the edge"
                  " is where it reads ~half-way black<->white)\n" % (c0, args.target))

        cmd("drivebase turn %d %d hold" % (-args.theta_max, args.turn_dps), to=20)
        a0, _ = wait_settle(args.wait_timeout + 2.0)

        n = int(round(2 * args.theta_max / args.theta_step)) + 1
        angles, ints, stds = [], [], []
        any_stall = False
        for k in range(n):
            ang_mdeg, stalled = wait_settle(args.wait_timeout)
            any_stall = any_stall or bool(stalled)
            inten, std, cnt = read_intensity()
            if ang_mdeg is None or inten is None:
                print("  ! sample %d: angle=%s intensity=%s (skipped)" % (k, ang_mdeg, inten))
            else:
                ang = ang_mdeg / 1000.0
                angles.append(ang)
                ints.append(inten)
                stds.append(std)
                print("  theta=%7.2f deg  intensity=%6.1f  (std=%.2f, n=%d)%s"
                      % (ang, inten, std, cnt, "  STALL" if stalled else ""))
            if k < n - 1:
                cmd("drivebase turn %d %d hold" % (args.theta_step, args.turn_dps), to=20)

        # recentre
        cur, _ = wait_settle(args.wait_timeout)
        if cur is not None:
            cmd("drivebase turn %d %d brake" % (round(-cur / 1000.0), args.turn_dps), to=20)

    finally:
        ser.close()

    if len(angles) < 4:
        print("\nERROR: too few valid samples (%d). Check sensor placement / serial." % len(angles))
        return 2

    # --- validity gate: what matters is that we CROSSED the edge (both
    # shoulders + linear region), not that we covered the commanded angle.
    # A cable-tether-limited but edge-capturing sweep is perfectly valid. ---
    ang_range = max(angles) - min(angles)
    swing = max(ints) - min(ints)
    commanded = 2.0 * args.theta_max
    crosses_target = min(ints) <= args.target <= max(ints)
    problems = []
    if swing < 300:
        problems.append(
            "intensity swing only %d counts -> the sensor barely crossed the "
            "edge. Re-park so intensity ~= target (%d) at the START (the script "
            "sweeps +-%d deg around it), on a black/white boundary." % (
                round(swing), args.target, args.theta_max))
    elif not crosses_target:
        problems.append(
            "swing %d..%d never reaches target=%d -> sweep is off the edge; "
            "recentre so the sensor reads ~target at the start." % (
                round(min(ints)), round(max(ints)), args.target))
    if ang_range < 0.6 * commanded:
        # not fatal: warn only, since the edge may still be fully captured
        print("\n  note: robot covered only %.1f of %d deg commanded "
              "(cable tether / obstruction / stall=%s). OK *if* the edge was "
              "captured (check the swing below)." % (ang_range, commanded, any_stall))

    mean_std = sum(stds) / len(stds) if stds else None
    print(ascii_scatter(angles, ints, [], "theta deg", title="\nintensity vs theta:"))
    result, sx, sy = fit_and_report(angles, ints, mean_std, args.L, True,
                                    args.central_frac, args.target)
    print(ascii_scatter(angles, ints, sx, "theta deg",
                        title="\nintensity vs theta (fit band):"))
    _dump_sweep_csv(args, angles, ints, stds)

    valid = not problems and result.get("r2", 0) >= 0.985
    if valid:
        write_params(result, os.path.join(os.path.dirname(__file__), "cl_params.txt"))
        return 0
    print("\n*** MEASUREMENT INVALID -- do NOT feed these values to the sim ***")
    for p in problems:
        print("  - " + p)
    if result.get("r2", 0) < 0.985:
        print("  - fit R^2=%.3f < 0.985 (try --central-frac lower, or a cleaner edge)"
              % result.get("r2", 0))
    print("  raw samples saved to sweep_samples.csv for inspection.")
    return 3


def _dump_sweep_csv(args, angles, ints, stds):
    path = os.path.join(os.path.dirname(__file__), "sweep_samples.csv")
    with open(path, "w") as f:
        f.write("theta_deg,intensity,std,offset_mm\n")
        for a, i, s in zip(angles, ints, stds):
            off = args.L * math.sin(math.radians(a)) if args.L else float("nan")
            f.write("%.3f,%.2f,%.3f,%.4f\n" % (a, i, s, off))
    print("   wrote %s" % path)


# --------------------------------------------------------------------------
# offline fit from a CSV (manual lateral-jog, or re-fit a saved sweep)
# --------------------------------------------------------------------------
def run_fit_csv(args):
    xs, ys = [], []
    x_is_angle = False
    with open(args.csv) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = [p.strip() for p in line.replace(",", " ").split()]
            if not parts or not _isnum(parts[0]):
                # header row: detect angle vs offset column
                x_is_angle = any("theta" in p.lower() or "angle" in p.lower() for p in parts)
                continue
            xs.append(float(parts[0]))
            ys.append(float(parts[1]))
    if args.x_is_angle:
        x_is_angle = True
    if args.x_is_offset:
        x_is_angle = False
    print("# fit-csv: %d points, x = %s" % (len(xs), "angle(deg)" if x_is_angle else "offset(mm)"))
    print(ascii_scatter(xs, ys, [], "theta deg" if x_is_angle else "offset mm",
                        title="intensity vs x:"))
    result, _, _ = fit_and_report(xs, ys, None, args.L, x_is_angle,
                                  args.central_frac, args.target)
    write_params(result, os.path.join(os.path.dirname(__file__), "cl_params.txt"))
    return 0


def _isnum(s):
    try:
        float(s)
        return True
    except ValueError:
        return False


# --------------------------------------------------------------------------
# self-test: synthesise a clipped-linear edge with known c*L, verify recovery
# --------------------------------------------------------------------------
def run_self_test(args):
    print("# self-test: synthetic edge, c=80 counts/mm, L=35 mm -> c*L=2800 counts/rad")
    c_true, L_true, target = 80.0, 35.0, 512
    angles, ints = [], []
    for deg in range(-20, 21, 2):
        off = L_true * math.sin(math.radians(deg))     # mm
        val = target + c_true * off                     # counts
        # +-1.5 LSB deterministic wobble
        val += 1.5 * math.sin(deg * 1.7)
        val = max(0, min(1024, val))
        angles.append(float(deg))
        ints.append(val)
    print(ascii_scatter(angles, ints, [], "theta deg", title="synthetic intensity vs theta:"))
    result, _, _ = fit_and_report(angles, ints, 1.0, L_true, True, 0.6, target)
    cL_true = c_true * L_true
    ok = abs(result.get("cL", 0) - cL_true) / cL_true < 0.05 and \
        abs(result.get("c", 0) - c_true) / c_true < 0.05
    print("\n   expected c*L=%.0f c=%.0f ; recovered c*L=%.1f c=%.2f -> %s" % (
        cL_true, c_true, result.get("cL", 0), result.get("c", 0),
        "PASS" if ok else "FAIL"))
    return 0 if ok else 1


# --------------------------------------------------------------------------
def build_parser():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--mode", default="sweep",
                   choices=["sweep", "fit-csv", "self-test"])
    p.add_argument("--device", default="/dev/ttyACM0", help="Hub serial device")
    p.add_argument("--L", type=float, default=None, help="look-ahead [mm], ruler-measured")
    p.add_argument("--target", type=int, default=512, help="intensity setpoint")
    # sweep
    p.add_argument("--theta-max", type=int, default=8,
                   help="sweep half-range [deg]; small + centred on the edge to "
                        "stay inside a cable-tether-limited rotation arc")
    p.add_argument("--theta-step", type=int, default=1, help="sweep step [deg]")
    p.add_argument("--turn-dps", type=int, default=30, help="turn rate [deg/s] (slow)")
    p.add_argument("--wait-timeout", type=float, default=5.0,
                   help="max seconds to wait for each turn's done==1")
    p.add_argument("--settle-tol-mdeg", type=int, default=300,
                   help="angle stable within this many mdeg => settled")
    p.add_argument("--watch-ms", type=int, default=200, help="sensor watch window per sample")
    # fit
    p.add_argument("--csv", help="fit-csv input: rows of x,intensity")
    p.add_argument("--x-is-angle", action="store_true", help="force x column = theta[deg]")
    p.add_argument("--x-is-offset", action="store_true", help="force x column = offset[mm]")
    p.add_argument("--central-frac", type=float, default=0.5,
                   help="linear-region detector: keep points with local slope "
                        ">= this fraction of the peak slope (excludes saturation "
                        "shoulders); 0.3-0.6 typical")
    return p


def main(argv):
    args = build_parser().parse_args(argv)
    if args.mode == "self-test":
        return run_self_test(args)
    if args.mode == "fit-csv":
        if not args.csv:
            print("ERROR: --mode fit-csv needs --csv <file>")
            return 2
        return run_fit_csv(args)
    return run_sweep(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
