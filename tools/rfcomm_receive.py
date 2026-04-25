#!/usr/bin/env python3
"""Receive raw RFCOMM frames from the SPIKE Prime Hub btsensor app.

Bypasses the /dev/rfcomm0 tty wrapper (which drops bytes via line
discipline even with `stty raw`) and talks to BlueZ directly via a
BTPROTO_RFCOMM socket.

Prereqs:
  - Hub paired + trusted via bluetoothctl (one-time).
  - No `rfcomm connect` / `rfcomm bind` holding the channel.  If one is
    running, Ctrl-C it or `sudo killall rfcomm` first.
  - Run with sudo (raw BT socket).

Usage:
  sudo python3 tools/rfcomm_receive.py                   # 5s capture, summary
  sudo python3 tools/rfcomm_receive.py --duration 10
  sudo python3 tools/rfcomm_receive.py --out /tmp/imu.bin
  sudo python3 tools/rfcomm_receive.py --decode          # parse frames
"""

import argparse
import socket
import struct
import sys
import time

DEFAULT_ADDR = "F8:2E:0C:A0:3E:64"
DEFAULT_CHANNEL = 1

MAGIC = 0xA55A
HDR_FMT = "<HHIHBB"     # magic, seq, ts_us, rate, count, type
HDR_SIZE = struct.calcsize(HDR_FMT)
SAMPLE_FMT = "<hhhhhh"  # ax ay az gx gy gz (raw LSBs)
SAMPLE_SIZE = struct.calcsize(SAMPLE_FMT)

ACCEL_MG_LSB = 0.244    # LSM6DS3 at ±8 g
GYRO_DPS_LSB = 0.070    # LSM6DS3 at ±2000 dps


def capture(addr, channel, duration):
    s = socket.socket(socket.AF_BLUETOOTH,
                      socket.SOCK_STREAM,
                      socket.BTPROTO_RFCOMM)
    print(f"connecting to {addr} channel {channel} ...", flush=True)
    s.connect((addr, channel))
    print("connected", flush=True)

    s.settimeout(0.5)
    deadline = time.time() + duration
    buf = bytearray()
    reads = 0
    while time.time() < deadline:
        try:
            chunk = s.recv(4096)
        except socket.timeout:
            continue
        if not chunk:
            print("peer closed", flush=True)
            break
        buf += chunk
        reads += 1

    s.close()
    rate = len(buf) / duration if duration else 0
    print(f"received {len(buf)} bytes in {duration}s "
          f"= {rate/1024:.1f} KB/s ({reads} recv() calls)")
    return bytes(buf)


def find_magic(buf):
    idx = buf.find(b"\x5a\xa5")
    if idx < 0:
        print("no 0xA55A magic found — tty / socket corruption likely")
        return
    print(f"first magic at offset {idx}")
    if len(buf) < idx + HDR_SIZE:
        return
    magic, seq, ts, rate, count, typ = struct.unpack(
        HDR_FMT, buf[idx:idx + HDR_SIZE])
    print(f"  seq={seq} ts_us={ts} rate={rate} count={count} type=0x{typ:02x}")


def decode_stream(buf, dump_all=False):
    pos = 0
    frames_meta = []   # list of (seq, ts_us, count, type, first_idx)
    last_seq = None
    seq_gaps = 0
    seq_jump_total = 0
    while pos + HDR_SIZE <= len(buf):
        idx = buf.find(b"\x5a\xa5", pos)
        if idx < 0:
            break
        if idx + HDR_SIZE > len(buf):
            break
        magic, seq, ts, rate, count, typ = struct.unpack(
            HDR_FMT, buf[idx:idx + HDR_SIZE])
        if magic != MAGIC or count == 0 or count > 64:
            pos = idx + 1
            continue
        need = HDR_SIZE + count * SAMPLE_SIZE
        if idx + need > len(buf):
            break
        if last_seq is not None:
            expected = (last_seq + 1) & 0xFFFF
            if seq != expected:
                seq_gaps += 1
                # signed delta mod 2^16
                jump = (seq - last_seq) & 0xFFFF
                seq_jump_total += jump - 1
        last_seq = seq
        frames_meta.append((seq, ts, count, typ, idx))
        pos = idx + need

    if not frames_meta:
        print("no frames decoded")
        return

    print(f"decoded {len(frames_meta)} frames, "
          f"{sum(m[2] for m in frames_meta)} samples, "
          f"{seq_gaps} seq gaps "
          f"(extra missed seqs: {seq_jump_total}), "
          f"rate field (header) = {rate} Hz")

    # Dump every frame: seq, ts_us, dt_ms, expected_dt_ms, count.
    if dump_all:
        prev_ts = None
        prev_seq = None
        print()
        print(f"{'seq':>6} {'ts_us':>12} {'dt_ms':>8} {'exp_dt_ms':>10} "
              f"{'cnt':>4} {'seq_jump':>9}")
        for seq, ts, count, typ, idx in frames_meta:
            if prev_ts is None:
                dt_ms = 0.0
                exp = 0.0
                jump = 0
            else:
                dt_ms = (ts - prev_ts) / 1000.0
                # Expected dt assumes the previous frame's sample count.
                # All frames are normally `count` samples, so use that.
                seq_jump_local = (seq - prev_seq) & 0xFFFF
                exp = seq_jump_local * count * (1000.0 / 833.0)
                jump = seq_jump_local
            print(f"{seq:>6} {ts:>12} {dt_ms:>8.2f} {exp:>10.2f} "
                  f"{count:>4} {jump:>9}")
            prev_ts = ts
            prev_seq = seq

    # Sample-rate sanity from timestamps:
    # total samples == sum(count_i) for received frames.
    # wall-time spanned == ts[-1] - ts[0].
    # If Hub was producing samples at full ODR and we received every frame,
    # wall-time / (total samples - count_first) == 1/ODR_actual.
    # If we missed seq numbers, wall-time still spans the same ODR window
    # because timestamps are sourced from the sensor stream, but total
    # received samples becomes a fraction of expected.
    first_seq, first_ts, _, _, _ = frames_meta[0]
    last_seq, last_ts, last_count, _, _ = frames_meta[-1]
    seq_span = (last_seq - first_seq) & 0xFFFF
    total_received = sum(m[2] for m in frames_meta)
    expected_samples_in_span = (seq_span + 1) * frames_meta[0][2]
    wall_us = last_ts - first_ts
    if seq_span > 0 and wall_us > 0:
        odr_seqbased = expected_samples_in_span / (wall_us / 1e6)
        odr_received = total_received / (wall_us / 1e6)
        print()
        print(f"first frame: seq={first_seq} ts_us={first_ts}")
        print(f"last  frame: seq={last_seq} ts_us={last_ts}")
        print(f"wall_us span: {wall_us}  ({wall_us/1e3:.1f} ms)")
        print(f"seq span: {seq_span+1} frames "
              f"(expected {expected_samples_in_span} samples at sensor rate)")
        print(f"received: {total_received} samples "
              f"({total_received/expected_samples_in_span*100:.1f} %)")
        print(f"sensor ODR (from seq+ts):  {odr_seqbased:7.1f} Hz "
              f"(Hub-side actual sampling rate)")
        print(f"link ODR  (received only): {odr_received:7.1f} Hz "
              f"(samples actually delivered to PC)")


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--addr", default=DEFAULT_ADDR)
    p.add_argument("--channel", type=int, default=DEFAULT_CHANNEL)
    p.add_argument("--duration", type=float, default=5.0)
    p.add_argument("--out")
    p.add_argument("--decode", action="store_true",
                   help="parse frames + sample-rate sanity")
    p.add_argument("--dump-all", action="store_true",
                   help="dump every frame's seq / ts_us / dt (implies --decode)")
    args = p.parse_args()

    try:
        buf = capture(args.addr, args.channel, args.duration)
    except OSError as e:
        print(f"connect/recv failed: {e}", file=sys.stderr)
        print("  - is the channel already held by `rfcomm connect`?",
              file=sys.stderr)
        print("  - is the Hub paired+trusted and btsensor running?",
              file=sys.stderr)
        return 1

    if args.out:
        with open(args.out, "wb") as f:
            f.write(buf)
        print(f"wrote {args.out}")

    find_magic(buf)
    if args.decode or args.dump_all:
        decode_stream(buf, dump_all=args.dump_all)
    return 0


if __name__ == "__main__":
    sys.exit(main())
