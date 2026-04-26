# Receiving the SPP stream on a PC

The SPIKE Prime Hub `btsensor` app emits IMU samples over Classic
Bluetooth SPP (RFCOMM channel 1) as a little-endian binary stream.
This page walks through the host-side receive procedure for Linux and
macOS, the on-wire frame format, the ASCII command protocol, and a
reusable Python parser.

!!! warning "Wire-format change in Issue #56 Commit E"
    The frame magic moved from `0xA55A` to `0xB66B` and the per-sample
    layout grew from 12 to 16 bytes (added a `ts_delta_us` field).
    Pre-Commit-E PC scripts that hard-coded `0xA55A` will not parse
    the new stream — refresh the parser per the section below.

## Linux (BlueZ)

### Required packages

```bash
# Ubuntu/Debian
sudo apt install bluez bluez-tools

# Fedora
sudo dnf install bluez bluez-tools
```

The `rfcomm` CLI lives in `bluez-tools` on most distros (or
`bluez-compat` on older ones).  Confirm with `which rfcomm`.

### Pairing

With the daemon running on the Hub (`btsensor start [batch]`; the BT
button on the Hub or any future control surface enables advertising —
see the [Bluetooth driver page](../drivers/bluetooth.md) for the BT
state machine):

```bash
bluetoothctl
[bluetooth]# power on
[bluetooth]# scan on
# wait 5-10 s until SPIKE-BT-Sensor appears
[bluetooth]# pair F8:2E:0C:A0:3E:64
# expect "Pairing successful"
[bluetooth]# trust F8:2E:0C:A0:3E:64
[bluetooth]# quit
```

The BD address is logged on the Hub console as part of `HCI working,
BD_ADDR ...` or via `bluetoothctl info`.

### Verify the SPP SDP record

```bash
sdptool browse F8:2E:0C:A0:3E:64
```

You should see `Service Name: SPIKE IMU Stream`, `Channel: 1`,
`Profile: SPP (v1.2)`.

### Open RFCOMM + read the stream

```bash
# bind a tty to the RFCOMM destination (minor 0)
sudo rfcomm bind 0 F8:2E:0C:A0:3E:64 1
ls -l /dev/rfcomm0

# lazy open — Linux initiates the RFCOMM session on first I/O.  Send
# `IMU ON\n` first so the Hub starts streaming (sampling is OFF by
# default at daemon start; see the ASCII command section below).
printf 'IMU ON\n' | sudo tee /dev/rfcomm0 > /dev/null
sudo cat /dev/rfcomm0 | xxd | head
```

The first bytes should be `6b b6 ...` (magic `0xB66B` in little-endian)
followed by type / sample_count / rate / FSR / seq / first-sample
timestamp / frame_len + samples.

Throughput probe (default batch=8 at 833 Hz → 18 + 8×16 = 146 B/frame
× 104 Hz ≈ 15 KB/s ≈ 121 kbps):

```bash
sudo cat /dev/rfcomm0 | pv > /dev/null
```

Release when done: `sudo rfcomm release 0`.

### Fallback: `rfcomm connect` (foreground)

If `rfcomm bind`'s lazy open fails with
`cat: /dev/rfcomm0: Cannot allocate memory`, use the explicit
foreground variant:

```bash
# holds the session open until Ctrl-C
sudo rfcomm connect 0 F8:2E:0C:A0:3E:64 1
# in another terminal
sudo cat /dev/rfcomm0 | xxd | head
```

A lingering `Permission denied` usually means the host has a stale
link key from an earlier session.  Re-pair: `bluetoothctl remove`
then `pair` again.

## macOS IOBluetooth

macOS's Classic BT SPP support has a few quirks:

1. System Settings → Bluetooth will *not* show a "Connect" button for
   generic SPP devices with a Class of Device of Uncategorized — the
   device appears under "Nearby Devices" but the UI action is
   suppressed.
2. Pairing itself can be driven from the CLI via
   [blueutil](https://github.com/toy/blueutil).
3. `/dev/tty.SPIKE-BT-Sensor-*` is created after pairing, but reading
   it through `cat` / `screen` will not drive IOBluetooth into opening
   an RFCOMM session.  You must use `IOBluetoothRFCOMMChannel` through
   PyObjC (or a native Swift helper).

### Tools

```bash
brew install blueutil
python3 -m pip install pyobjc-framework-IOBluetooth
```

### Pairing

With the daemon running (`btsensor start`):

```bash
blueutil --inquiry
blueutil --pair f8-2e-0c-a0-3e-64
# Hub console prints "SSP confirm" and
# "SSP pairing with ... status 0x00"
```

### Open RFCOMM + read (Python)

`/dev/tty.SPIKE-BT-Sensor-*` exists after pairing but isn't actively
bound to RFCOMM until something asks the framework to open the
channel.  Use PyObjC:

```python
# pc_receive_spp.py
import IOBluetooth
from Foundation import NSObject, NSRunLoop, NSDate

ADDR = "f8-2e-0c-a0-3e-64"


class RFCOMMDelegate(NSObject):
    def rfcommChannelOpenComplete_status_(self, channel, status):
        print(f"channel open status={status}", flush=True)

    def rfcommChannelData_data_length_(self, channel, data, length):
        buf = bytes(data)[:length]
        print(buf.hex(), flush=True)

    def rfcommChannelClosed_(self, channel):
        print("channel closed", flush=True)


def main():
    dev = IOBluetooth.IOBluetoothDevice.deviceWithAddressString_(ADDR)
    delegate = RFCOMMDelegate.alloc().init()
    result = dev.openRFCOMMChannelSync_withChannelID_delegate_(None, 1,
                                                               delegate)
    print("open result:", result)
    deadline = NSDate.dateWithTimeIntervalSinceNow_(60.0)
    NSRunLoop.currentRunLoop().runUntilDate_(deadline)


if __name__ == "__main__":
    main()
```

Run with `python3 pc_receive_spp.py`.  The Hub console should print
`RFCOMM incoming` → `RFCOMM open cid=...` as the channel comes up.

## Wire format (Commit E layout)

All fields are little-endian.  Frame = 18-byte header + N × 16-byte
samples, where N = `sample_count` (1..80, default 8).

### Header (18 bytes)

| Offset | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 2 | `magic` | always `0xB66B` |
| 2 | 1 | `type` | `0x01` = IMU |
| 3 | 1 | `sample_count` | 1..80 |
| 4 | 2 | `sample_rate_hz` | current driver ODR |
| 6 | 2 | `accel_fsr_g` | 2 / 4 / 8 / 16 |
| 8 | 2 | `gyro_fsr_dps` | 125 / 250 / 500 / 1000 / 2000 |
| 10 | 2 | `seq` | monotonic per frame |
| 12 | 4 | `first_sample_ts_us` | low 32 bits of `CLOCK_BOOTTIME` µs (mod 2³², ≈71m35s wrap — PC must unwrap for long captures) |
| 16 | 2 | `frame_len` | `= 18 + sample_count × 16` |

### Per sample (16 bytes)

| Offset (within sample) | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 2 | `ax` | int16, chip frame |
| 2 | 2 | `ay` | int16, chip frame |
| 4 | 2 | `az` | int16, chip frame |
| 6 | 2 | `gx` | int16, chip frame |
| 8 | 2 | `gy` | int16, chip frame |
| 10 | 2 | `gz` | int16, chip frame |
| 12 | 4 | `ts_delta_us` | uint32, sample timestamp − `first_sample_ts_us`; sample[0] = 0 |

**Axis convention**: raw chip frame as published by the LSM6DSL.  No
hub-body axis remap is applied on the Hub; if you need a hub frame,
apply the rotation on the PC.

### Resync after byte loss

Use `frame_len` to skip past truncated / corrupted frames:

1. Find the next `0xB66B` magic.
2. Read 18 bytes; sanity-check `type == 0x01`,
   `1 ≤ sample_count ≤ 80`, and `frame_len == 18 + sample_count × 16`.
3. If `frame_len` looks bogus, advance by 1 and rescan from step 1.
4. Otherwise wait for `frame_len` bytes total and parse.

### Converting raw to physical units

Multiply each int16 LSB by the per-FSR sensitivity from the LSM6DSL
datasheet:

| FSR | Accel sensitivity (mg/LSB) | Gyro sensitivity (mdps/LSB) |
|---|---|---|
| ±2 g  / ±125 dps  | 0.061 | 4.375 |
| ±4 g  / ±250 dps  | 0.122 | 8.75  |
| ±8 g  / ±500 dps  | 0.244 | 17.5  |
| ±16 g / ±1000 dps | 0.488 | 35.0  |
| (–)   / ±2000 dps | (–)   | 70.0  |

Or programmatically, given the header's FSR fields:

```python
accel_mg_lsb  = (accel_fsr_g  * 1000.0) / 32768.0
gyro_dps_lsb  = (gyro_fsr_dps * 0.035) / 1000.0
# accel_mps2 = raw * accel_mg_lsb * 9.80665 / 1000
# gyro_dps   = raw * gyro_dps_lsb
```

(Accel scales are exact int16 full-scale; gyro carries ~14.7%
headroom over the nominal FSR — see datasheet Table 3.)

## Shared Python parser

Works the same way on Linux (`/dev/rfcomm0`) and macOS (once you've
piped the IOBluetooth delegate's bytes into a stream):

```python
# btsensor_parser.py
import struct, sys, glob

MAGIC       = 0xB66B
HDR_FMT     = "<HBBHHHHIH"   # magic, type, count, rate, accel_fsr,
                             # gyro_fsr, seq, first_ts_us, frame_len
HDR_SIZE    = 18
SAMPLE_FMT  = "<hhhhhhI"     # ax ay az gx gy gz ts_delta_us
SAMPLE_SIZE = 16

GRAVITY_MS2 = 9.80665


def parse_frame(buf, offs, accel_mg_lsb, gyro_dps_lsb,
                seq, first_ts_us, sample_count):
    for i in range(sample_count):
        ax, ay, az, gx, gy, gz, dt = struct.unpack(
            SAMPLE_FMT,
            buf[offs + i * SAMPLE_SIZE : offs + (i + 1) * SAMPLE_SIZE])
        ts_us = (first_ts_us + dt) & 0xffffffff
        print(f"seq={seq} ts_us={ts_us} dt_us={dt} "
              f"accel_mg=({ax*accel_mg_lsb:7.2f},{ay*accel_mg_lsb:7.2f},"
              f"{az*accel_mg_lsb:7.2f}) "
              f"gyro_dps=({gx*gyro_dps_lsb:7.3f},{gy*gyro_dps_lsb:7.3f},"
              f"{gz*gyro_dps_lsb:7.3f})")


def parse_stream(stream):
    buf = b""
    while True:
        chunk = stream.read(256)
        if not chunk:
            break
        buf += chunk
        while len(buf) >= HDR_SIZE:
            idx = buf.find(b"\x6b\xb6")
            if idx < 0:
                buf = buf[-1:]
                break
            if idx > 0:
                buf = buf[idx:]
            if len(buf) < HDR_SIZE:
                break
            (magic, typ, count, rate, accel_fsr, gyro_fsr,
             seq, first_ts, frame_len) = struct.unpack(HDR_FMT,
                                                       buf[:HDR_SIZE])
            if magic != MAGIC or typ != 0x01 or not 1 <= count <= 80 \
                    or frame_len != HDR_SIZE + count * SAMPLE_SIZE:
                buf = buf[1:]
                continue
            if len(buf) < frame_len:
                break
            accel_mg_lsb = (accel_fsr * 1000.0) / 32768.0
            gyro_dps_lsb = (gyro_fsr  * 0.035) / 1000.0
            parse_frame(buf, HDR_SIZE, accel_mg_lsb, gyro_dps_lsb,
                        seq, first_ts, count)
            buf = buf[frame_len:]


def default_port():
    macos = glob.glob("/dev/tty.SPIKE-BT-Sensor*")
    if macos:
        return macos[0]
    return "/dev/rfcomm0"


if __name__ == "__main__":
    port = sys.argv[1] if len(sys.argv) > 1 else default_port()
    with open(port, "rb") as f:
        parse_stream(f)
```

Run on Linux:

```bash
# Make sure to send `IMU ON\n` to /dev/rfcomm0 first.
sudo python3 btsensor_parser.py /dev/rfcomm0
```

A motionless Hub at the default ±8 g / ±2000 dps reports
`accel_z ≈ ±1000 mg` (±1 g) with the other axes near zero; shake it to
see peaks.  `gyro_dps` is ≈ 0 at rest after a brief warm-up.

## ASCII command protocol

After RFCOMM is open the Hub accepts ASCII command lines (`\n`
terminated, `\r` ignored) on the same channel.  Commands and replies
share the channel with the binary IMU stream — replies always queue
ahead of the next telemetry frame.

| Command | Action | Constraint |
|---|---|---|
| `IMU ON\n`  | Start sampling (open the driver, auto-activate) | — |
| `IMU OFF\n` | Stop sampling (close the driver, auto-deactivate) | — |
| `SET ODR <hz>\n` | Set ODR (13/26/52/104/208/416/833/1660/3330/6660 Hz) | only while IMU OFF |
| `SET BATCH <n>\n` | Samples per RFCOMM frame (1..80) | only while IMU OFF |
| `SET ACCEL_FSR <g>\n` | Accel FSR (2/4/8/16) | only while IMU OFF |
| `SET GYRO_FSR <dps>\n` | Gyro FSR (125/250/500/1000/2000) | only while IMU OFF |

Reply patterns:
- `OK\n` on success
- `ERR busy\n` if a `SET *` arrives while sampling is on
- `ERR invalid <token>\n` for malformed values / unknown subcommands
- `ERR overflow\n` for lines longer than 64 bytes
- `ERR unknown <cmd>\n` for an unrecognised first token

Sampling is **off** at daemon start, so a typical session looks like:

```text
PC -> Hub:  IMU OFF\n              (idempotent)
Hub -> PC:  OK\n
PC -> Hub:  SET ODR 416\n
Hub -> PC:  OK\n
PC -> Hub:  SET BATCH 16\n
Hub -> PC:  OK\n
PC -> Hub:  IMU ON\n
Hub -> PC:  OK\n
            ... binary IMU frames flow ...
PC -> Hub:  IMU OFF\n
Hub -> PC:  OK\n
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Linux: `cat: /dev/rfcomm0: Cannot allocate memory` | lazy RFCOMM bind failed to open | Switch to `rfcomm connect` (foreground).  A persistent `Permission denied` instead usually means stale link keys — remove + re-pair in `bluetoothctl` |
| Linux: `Permission denied` | Hub's in-RAM link-key DB was wiped by the latest flash but the host still holds the stale key | `bluetoothctl remove <MAC>` then pair again |
| macOS: `cat` returns nothing | macOS tty open alone does not establish RFCOMM | Use the PyObjC `IOBluetoothRFCOMMChannel` path above |
| macOS: no "Connect" button in Settings | CoD 0x001F00 is filtered by macOS UI | Drive pairing with `blueutil --pair` from the CLI |
| Frame drop counter rising | RFCOMM send can't keep up (e.g. PC disconnected) | Hub ring is 8 frames deep; drops oldest on overflow by design |

## See also

- `host/ImuViewer/` — desktop visualizer (.NET 10 + Avalonia + Silk.NET) that
  consumes this stream, runs a Madgwick filter, and renders a 3D Cube whose
  orientation tracks the Hub. Linux PoC; macOS / Windows are stubbed.
- btstack `example/spp_counter.c` — minimal SPP server reference
- BlueZ `rfcomm(1)` — Linux CLI reference
- [IOBluetoothRFCOMMChannel Class Reference](https://developer.apple.com/documentation/iobluetooth/iobluetoothrfcommchannel) — macOS API
