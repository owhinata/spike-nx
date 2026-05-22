#!/usr/bin/env python3
"""
Parse a .bin file of BTSENSOR_FRAME_TYPE_IMU_CAP (0x03) frames captured by
ImuViewer's Capture tab and emit ASCII files in the format imu_tk's
importAsciiData() expects (4 space-separated floats per line:
`ts_sec ax ay az` and `ts_sec gx gy gz`).

Also emits meta.txt with session-invariant ODR/FSR and the mean ambient
temperature (used as the reference temperature in imu_cal.txt).

Frame layout (27 B, envelope 5 + payload 22) — must match
apps/btsensor/btsensor_wire.h:

    +0   magic        uint16 LE   (BTSENSOR_FRAME_MAGIC = 0xB66B)
    +2   frame_type   uint8       (BTSENSOR_FRAME_TYPE_IMU_CAP = 0x03)
    +3   frame_len    uint16 LE   (= 27)
    +5   timestamp_us uint32 LE   (low 32 bits of CLOCK_BOOTTIME us)
    +9   ax,ay,az     int16 LE x3 (raw LSB, Hub body frame)
    +15  gx,gy,gz     int16 LE x3 (raw LSB, Hub body frame)
    +21  temp_raw     int16 LE    (LSM6DSL OUT_TEMP, T_c = 25 + raw/256)
    +23  fsr_xl_idx   uint8       (lsm6dsl_fsr_xl_e: 0=2g 1=16g 2=4g 3=8g)
    +24  fsr_gy_idx   uint8       (lsm6dsl_fsr_gy_e: 0=250 1=125 2=500
                                   4=1000 6=2000 dps)
    +25  seq          uint16 LE   (monotonic wrap-around)
"""

import argparse
import struct
import sys
from pathlib import Path

FRAME_MAGIC = 0xB66B
FRAME_TYPE_IMU_CAP = 0x03
FRAME_SIZE = 27
FRAME_FMT = "<HBHIhhhhhhhBBH"  # 27 B total

# Driver-internal enum -> physical value lookup, mirrors
# boards/spike-prime-hub/src/lsm6dsl_uorb.c.
FSR_XL_TABLE_G = {0: 2, 1: 16, 2: 4, 3: 8}
FSR_GY_TABLE_DPS = {0: 250, 1: 125, 2: 500, 4: 1000, 6: 2000}


def parse_frames(buf):
    """Yield (ts_us, ax, ay, az, gx, gy, gz, temp_raw, xl_idx, gy_idx, seq)
    from the raw .bin buffer. Raises ValueError on any malformed frame.
    """
    n_frames = len(buf) // FRAME_SIZE
    if len(buf) % FRAME_SIZE != 0:
        print(
            f"warning: trailing {len(buf) % FRAME_SIZE} bytes ignored "
            f"(partial frame at EOF)",
            file=sys.stderr,
        )

    for i in range(n_frames):
        off = i * FRAME_SIZE
        chunk = buf[off : off + FRAME_SIZE]
        (
            magic,
            frame_type,
            frame_len,
            ts_us,
            ax, ay, az,
            gx, gy, gz,
            temp_raw,
            xl_idx,
            gy_idx,
            seq,
        ) = struct.unpack(FRAME_FMT, chunk)

        if magic != FRAME_MAGIC:
            raise ValueError(
                f"frame {i} @offset {off}: magic 0x{magic:04x} != "
                f"0x{FRAME_MAGIC:04x}"
            )
        if frame_type != FRAME_TYPE_IMU_CAP:
            raise ValueError(
                f"frame {i} @offset {off}: frame_type 0x{frame_type:02x} != "
                f"0x{FRAME_TYPE_IMU_CAP:02x}"
            )
        if frame_len != FRAME_SIZE:
            raise ValueError(
                f"frame {i} @offset {off}: frame_len {frame_len} != "
                f"{FRAME_SIZE}"
            )

        yield (ts_us, ax, ay, az, gx, gy, gz, temp_raw, xl_idx, gy_idx, seq)


def main():
    parser = argparse.ArgumentParser(
        description=(
            "Decode an IMU_CAP .bin capture into imu_tk ASCII inputs "
            "(accel.txt, gyro.txt) and a session meta.txt."
        )
    )
    parser.add_argument("bin_path", type=Path, help="input .bin file")
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path("."),
        help="output directory (default: cwd)",
    )
    parser.add_argument(
        "--allow-seq-gaps",
        action="store_true",
        help=(
            "convert input even when seq is not monotonic without "
            "wrap-around. Default rejects (ImuViewer should have "
            "already filtered drop-free sessions only)."
        ),
    )
    parser.add_argument(
        "--min-samples",
        type=int,
        default=2000,
        help=(
            "reject sessions with fewer than this many samples "
            "(Tedaldi calibration needs a long enough run; default 2000 "
            "≈ 20 s at 104 Hz)."
        ),
    )
    args = parser.parse_args()

    buf = args.bin_path.read_bytes()
    if len(buf) == 0:
        sys.exit(f"error: {args.bin_path} is empty")

    samples = list(parse_frames(buf))
    if len(samples) < args.min_samples:
        sys.exit(
            f"error: only {len(samples)} samples in {args.bin_path}; "
            f"need at least {args.min_samples} for Tedaldi calibration"
        )

    # Session-invariance + monotonic seq checks.  fsr_*_idx must not
    # change mid-session (any change means an ImuViewer-side race let a
    # SET ODR / drivebase start interleave with the capture — reject).
    first_xl = samples[0][8]
    first_gy = samples[0][9]
    prev_seq = samples[0][10]
    seq_gap_total = 0
    for i, s in enumerate(samples[1:], start=1):
        if s[8] != first_xl:
            sys.exit(
                f"error: fsr_xl_idx changed mid-session "
                f"(frame {i}: {s[8]} vs first {first_xl}); reject"
            )
        if s[9] != first_gy:
            sys.exit(
                f"error: fsr_gy_idx changed mid-session "
                f"(frame {i}: {s[9]} vs first {first_gy}); reject"
            )
        delta = (s[10] - prev_seq) & 0xFFFF
        if delta != 1:
            seq_gap_total += delta - 1
            if not args.allow_seq_gaps:
                sys.exit(
                    f"error: seq gap at frame {i}: prev={prev_seq} "
                    f"cur={s[10]} (delta {delta}); reject "
                    f"(use --allow-seq-gaps to override for debugging)"
                )
        prev_seq = s[10]

    if seq_gap_total > 0:
        print(
            f"warning: {seq_gap_total} samples dropped (seq gaps); "
            f"calibration quality compromised — only continuing because "
            f"--allow-seq-gaps was specified",
            file=sys.stderr,
        )

    # Map enum idx -> physical value for meta.txt and downstream
    # imu_tk_output_to_cfg.py.  Unknown idx is a session error since the
    # driver should never publish an out-of-table value.
    if first_xl not in FSR_XL_TABLE_G:
        sys.exit(f"error: unknown fsr_xl_idx {first_xl}; not in driver table")
    if first_gy not in FSR_GY_TABLE_DPS:
        sys.exit(f"error: unknown fsr_gy_idx {first_gy}; not in driver table")
    fsr_xl_g = FSR_XL_TABLE_G[first_xl]
    fsr_gy_dps = FSR_GY_TABLE_DPS[first_gy]

    # Build relative seconds timeline using uint32 modular arithmetic so a
    # single mid-session timestamp wrap (every 71.6 min of Hub uptime)
    # produces the correct positive delta.  Tedaldi only cares about
    # relative timing.
    t_us_accum = 0
    prev_ts = samples[0][0]
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    accel_path = out_dir / "accel.txt"
    gyro_path = out_dir / "gyro.txt"
    meta_path = out_dir / "meta.txt"

    temp_sum = 0
    odr_intervals_us = []  # for ODR estimation

    with accel_path.open("w") as acc_f, gyro_path.open("w") as gyro_f:
        for i, s in enumerate(samples):
            (ts_us, ax, ay, az, gx, gy, gz, temp_raw, _, _, _) = s
            if i > 0:
                delta_us = (ts_us - prev_ts) & 0xFFFFFFFF
                t_us_accum += delta_us
                odr_intervals_us.append(delta_us)
            prev_ts = ts_us
            t_sec = t_us_accum / 1_000_000.0
            acc_f.write(f"{t_sec:.6f} {ax} {ay} {az}\n")
            gyro_f.write(f"{t_sec:.6f} {gx} {gy} {gz}\n")
            temp_sum += temp_raw

    # Mean ambient temperature.  LSM6DSL OUT_TEMP convention per datasheet:
    # T_c = 25 + raw / 256  (sensitivity 256 LSB/°C, 0 LSB = 25 °C).
    mean_temp_raw = temp_sum / len(samples)
    ambient_temp_c = 25.0 + mean_temp_raw / 256.0

    # Estimated ODR from median interval.  Median is robust against the
    # one large delta that follows a timestamp wrap.
    odr_intervals_us.sort()
    median_interval_us = odr_intervals_us[len(odr_intervals_us) // 2]
    odr_hz_est = round(1_000_000.0 / median_interval_us)

    with meta_path.open("w") as m:
        m.write(f"fsr_xl_g = {fsr_xl_g}\n")
        m.write(f"fsr_gy_dps = {fsr_gy_dps}\n")
        m.write(f"odr_hz = {odr_hz_est}\n")
        m.write(f"ambient_temp_c = {ambient_temp_c:.1f}\n")
        m.write(f"sample_count = {len(samples)}\n")
        m.write(f"duration_sec = {t_us_accum / 1_000_000.0:.3f}\n")

    duration_sec = t_us_accum / 1_000_000.0
    print(
        f"wrote {len(samples)} samples ({duration_sec:.1f} s @ ~{odr_hz_est} Hz, "
        f"fsr_xl=±{fsr_xl_g}g, fsr_gy=±{fsr_gy_dps}dps, "
        f"T~{ambient_temp_c:.1f}°C)"
    )
    print(f"  {accel_path}")
    print(f"  {gyro_path}")
    print(f"  {meta_path}")


if __name__ == "__main__":
    main()
