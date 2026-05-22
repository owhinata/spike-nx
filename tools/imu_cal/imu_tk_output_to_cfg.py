#!/usr/bin/env python3
"""
Combine imu_tk's per-triad .calib files (test_imu_acc.calib,
test_imu_gyro.calib) and the bin_to_ascii.py meta.txt into the single
imu_cal.txt that drivebase_imu_cal.c loads from /mnt/flash on the Hub.

imu_tk model (per CalibratedTriad save()):
    corrected = T @ K @ (raw - B)

  T = 3x3 misalignment (diagonal = 1, off-diag small)
  K = 3x3 diagonal scale
  B = 3x1 raw-LSB bias

Each .calib file is the Eigen << dump of T, blank line, K, blank line,
B, blank line.  Eigen prints row-major, whitespace-separated, one row
per line.

Output (Phase 2.5 spec, schema_version=1):

  schema_version = 1
  nominal_gyro_radps_per_lsb = <float>   # fsr_dps * pi/180 * 0.035/1000
  nominal_accel_ms2_per_lsb  = <float>   # fsr_g  * 9.80665 / 32768
  fsr_gy_dps = <int>
  fsr_xl_g   = <int>
  odr_hz     = <int>
  ambient_temp_c = <float>

  gyro_bias_lsb_x1000   = b0 b1 b2          # round(B * 1000)
  accel_bias_lsb_x1000  = b0 b1 b2
  gyro_M_x1000   = m00 m01 m02 m10 m11 m12 m20 m21 m22   # row-major
  accel_M_x1000  = m00 m01 m02 m10 m11 m12 m20 m21 m22

We multiply out M = T @ K, then divide elementwise by the nominal
sensitivity (rad/s per LSB for gyro, m/s² per LSB for accel) so the
on-device matmul `M_x1000 @ (raw - bias) / 1000` stays in raw-LSB
domain and the existing `gyro_mdps_num` path is unchanged.  Identity
near the diagonal (M_x1000[i][i] ≈ 1000) means imu_tk found a sensor
close to its nominal sensitivity.
"""

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

SCHEMA_VERSION = 1
# LSM6DSL nominal gyro sensitivity at FSR=N dps: N * 1000 / 32768 mdps/LSB
# ≈ N * 0.03052 mdps/LSB ≈ N * 35/1000 mdps/LSB.  Plan keeps the 0.035
# approximation since it matches the on-device gyro_mdps_num = fsr * 35
# pipeline and the residual error (~0.6%) is absorbed by the M_x1000
# scale factors.
GYRO_SENS_MDPS_PER_LSB_PER_DPS = 0.035
ACCEL_G_TO_MS2 = 9.80665
ACCEL_LSB_FULL_SCALE = 32768.0


def parse_calib(path):
    """Read one .calib file (Eigen << dump of T, K, B with blank
    separators).  Returns (T, K, B) as numpy arrays shaped (3, 3),
    (3, 3), (3,).
    """
    blocks = []
    current = []
    with path.open() as f:
        for line in f:
            stripped = line.strip()
            if not stripped:
                if current:
                    blocks.append(current)
                    current = []
                continue
            row = [float(x) for x in stripped.split()]
            current.append(row)
        if current:
            blocks.append(current)

    if len(blocks) < 3:
        sys.exit(
            f"error: {path}: expected 3 blocks (T, K, B), got "
            f"{len(blocks)} — file truncated?"
        )

    T = np.array(blocks[0], dtype=float)
    K = np.array(blocks[1], dtype=float)
    B_block = np.array(blocks[2], dtype=float)
    # Bias may be saved as a column (3 lines × 1 value) or a row (1 line ×
    # 3 values) depending on Eigen IO state — accept either.
    B = B_block.reshape(-1)
    if T.shape != (3, 3) or K.shape != (3, 3) or B.shape != (3,):
        sys.exit(
            f"error: {path}: unexpected shapes T={T.shape} K={K.shape} "
            f"B={B.shape}; expected (3,3), (3,3), (3,)"
        )
    return T, K, B


def parse_meta(path):
    """Read meta.txt key=value lines emitted by bin_to_ascii.py.  Only
    the keys we use downstream are required.
    """
    required = {"fsr_xl_g", "fsr_gy_dps", "odr_hz", "ambient_temp_c"}
    out = {}
    with path.open() as f:
        for line in f:
            m = re.match(r"\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.+?)\s*$", line)
            if not m:
                continue
            out[m.group(1)] = m.group(2)
    missing = required - out.keys()
    if missing:
        sys.exit(f"error: {path} missing keys: {sorted(missing)}")
    return {
        "fsr_xl_g": int(out["fsr_xl_g"]),
        "fsr_gy_dps": int(out["fsr_gy_dps"]),
        "odr_hz": int(out["odr_hz"]),
        "ambient_temp_c": float(out["ambient_temp_c"]),
    }


def format_int_row(values):
    return " ".join(str(int(v)) for v in values)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Combine imu_tk .calib files and bin_to_ascii.py meta.txt into "
            "the imu_cal.txt loaded by drivebase on the Hub."
        )
    )
    parser.add_argument(
        "--session-dir",
        type=Path,
        default=Path("."),
        help="directory containing test_imu_acc.calib, test_imu_gyro.calib, "
             "meta.txt (default: cwd)",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="output path (default: <session-dir>/imu_cal.txt)",
    )
    args = parser.parse_args()

    sdir = args.session_dir
    acc_calib_path = sdir / "test_imu_acc.calib"
    gyro_calib_path = sdir / "test_imu_gyro.calib"
    meta_path = sdir / "meta.txt"
    for p in (acc_calib_path, gyro_calib_path, meta_path):
        if not p.is_file():
            sys.exit(f"error: {p} not found")

    T_acc, K_acc, B_acc = parse_calib(acc_calib_path)
    T_gyro, K_gyro, B_gyro = parse_calib(gyro_calib_path)
    meta = parse_meta(meta_path)

    # M = T @ K combines misalignment + scale into a single linear map.
    # Then M_runtime = M / nominal_sensitivity removes the physical unit
    # so on-device we can do `M_runtime @ (raw - bias)` purely in raw LSB
    # and still feed it to the existing gyro_mdps_num pipeline.
    M_acc = T_acc @ K_acc
    M_gyro = T_gyro @ K_gyro

    nom_acc = meta["fsr_xl_g"] * ACCEL_G_TO_MS2 / ACCEL_LSB_FULL_SCALE
    nom_gyro = (
        meta["fsr_gy_dps"]
        * (math.pi / 180.0)
        * GYRO_SENS_MDPS_PER_LSB_PER_DPS
        / 1000.0
    )

    M_acc_runtime = M_acc / nom_acc
    M_gyro_runtime = M_gyro / nom_gyro

    # x1000 fractional fixed-point. Bias is raw LSB so just multiply.
    M_acc_x1000 = np.round(M_acc_runtime * 1000).astype(np.int64)
    M_gyro_x1000 = np.round(M_gyro_runtime * 1000).astype(np.int64)
    bias_acc_x1000 = np.round(B_acc * 1000).astype(np.int64)
    bias_gyro_x1000 = np.round(B_gyro * 1000).astype(np.int64)

    output = args.output or (sdir / "imu_cal.txt")
    with output.open("w") as f:
        f.write(
            "# Auto-generated by tools/imu_cal/imu_tk_output_to_cfg.py\n"
        )
        f.write("# Do not edit by hand — re-run the pipeline instead.\n\n")
        f.write(f"schema_version = {SCHEMA_VERSION}\n")
        f.write(f"nominal_gyro_radps_per_lsb = {nom_gyro:.6e}\n")
        f.write(f"nominal_accel_ms2_per_lsb  = {nom_acc:.6e}\n")
        f.write(f"fsr_gy_dps = {meta['fsr_gy_dps']}\n")
        f.write(f"fsr_xl_g = {meta['fsr_xl_g']}\n")
        f.write(f"odr_hz = {meta['odr_hz']}\n")
        f.write(f"ambient_temp_c = {meta['ambient_temp_c']:.1f}\n\n")
        f.write(
            f"gyro_bias_lsb_x1000  = "
            f"{format_int_row(bias_gyro_x1000)}\n"
        )
        f.write(
            f"accel_bias_lsb_x1000 = "
            f"{format_int_row(bias_acc_x1000)}\n\n"
        )
        f.write(
            f"gyro_M_x1000  = "
            f"{format_int_row(M_gyro_x1000.flatten())}\n"
        )
        f.write(
            f"accel_M_x1000 = "
            f"{format_int_row(M_acc_x1000.flatten())}\n"
        )

    diag_msg = (
        f"  M_gyro diag (x1000): {format_int_row(np.diag(M_gyro_x1000))}\n"
        f"  M_accel diag (x1000): {format_int_row(np.diag(M_acc_x1000))}\n"
        f"  gyro bias  (LSB): {format_int_row(np.round(B_gyro))}\n"
        f"  accel bias (LSB): {format_int_row(np.round(B_acc))}\n"
    )
    print(f"wrote {output}\n{diag_msg}", end="")

    # Sanity: diagonals near 1000 = sensor matches nominal sensitivity.
    # Wide deviations (>5%) suggest a bad calibration session.
    for label, M_x1000 in (("gyro", M_gyro_x1000), ("accel", M_acc_x1000)):
        diag = np.diag(M_x1000)
        for axis, v in zip("XYZ", diag):
            deviation_pct = abs(v - 1000) / 10.0
            if deviation_pct > 5.0:
                print(
                    f"warning: {label} M[{axis}{axis}]={v} "
                    f"(±{deviation_pct:.1f}% off nominal); "
                    f"check calibration session quality",
                    file=sys.stderr,
                )


if __name__ == "__main__":
    main()
