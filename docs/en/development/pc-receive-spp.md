# Receiving the SPP stream on a PC

The SPIKE Prime Hub `btsensor` app emits **100 Hz BUNDLE** binary frames
over Classic Bluetooth SPP (RFCOMM channel 1).  Each bundle carries the
IMU samples that arrived during the 10 ms window plus a status entry
for every one of the six LEGO Powered Up sensor classes
(color/ultrasonic/force/motor_m/motor_r/motor_l).  This page walks
through the host-side receive procedure for Linux and macOS, the
on-wire BUNDLE layout, the ASCII command protocol, and a reusable
Python parser.

!!! warning "Wire-format change in Issue #88"
    The frame magic stays at `0xB66B` but `type` flips from `0x01`
    (legacy IMU-only) to `0x02` (BUNDLE), `frame_len` moved from
    offset 16 to offset 3, and the bundle now ends with six TLV
    entries (one per LEGO sensor class).  Pre-#88 scripts cannot
    parse the new stream — refresh the parser per the section below.
    `SET BATCH` is removed and `SET ODR` now caps at 833 Hz.  New
    commands `SENSOR ON | OFF` toggle the LEGO TLV section.

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

With the daemon running on the Hub (`btsensor start`; the BT button on
the Hub or any future control surface enables advertising — see the
[Bluetooth driver page](../drivers/bluetooth.md) for the BT state
machine):

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

# lazy open — Linux initiates the RFCOMM session on first I/O.  IMU and
# SENSOR are both off at daemon start, so send `IMU ON\n` and/or
# `SENSOR ON\n` first.
printf 'IMU ON\n' | sudo tee /dev/rfcomm0 > /dev/null
printf 'SENSOR ON\n' | sudo tee /dev/rfcomm0 > /dev/null
sudo cat /dev/rfcomm0 | xxd | head
```

The first bytes should be `6b b6 02 LL LL ...` (magic `0xB66B` + BUNDLE
type 0x02 + frame_len LE).

Throughput probe (10 ms tick × ~233 B average ≈ 22 KB/s ≈ 186 kbps):

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

## Wire format (BUNDLE / Issue #88)

All fields are little-endian.  Each 100 Hz tick emits one frame:
**envelope (5 B) + bundle header (16 B) + IMU subsection (0..8 samples
× 16 B) + TLV subsection (always 6 entries, variable length)**.  All
six LEGO sensor classes (color / ultrasonic / force / motor_m /
motor_r / motor_l) emit a TLV every tick; the `flags` field's `BOUND`
and `FRESH` bits report liveness and per-tick novelty.

### Envelope (5 B)

| Offset | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 2 | `magic` | always `0xB66B` |
| 2 | 1 | `type` | `0x02` = BUNDLE |
| 3 | 2 | `frame_len` | total frame length including envelope |

### Bundle header (16 B, offsets 5..20)

| Offset | Size | Field | Notes |
|---:|---:|---|---|
| 5  | 2 | `seq` | monotonic per bundle |
| 7  | 4 | `tick_ts_us` | absolute timestamp of this bundle's oldest IMU sample (low 32 bits of `CLOCK_BOOTTIME` µs; run-loop now() if no IMU samples). Per-sample `ts_delta_us` is therefore always non-negative |
| 11 | 2 | `imu_section_len` | `= imu_sample_count × 16` |
| 13 | 1 | `imu_sample_count` | 0..8 |
| 14 | 1 | `tlv_count` | always 6 |
| 15 | 2 | `imu_sample_rate_hz` | current ODR (≤833) |
| 17 | 1 | `imu_accel_fsr_g` | 2 / 4 / 8 / 16 |
| 18 | 2 | `imu_gyro_fsr_dps` | 125 / 250 / 500 / 1000 / 2000 |
| 20 | 1 | `flags` | bit0=IMU_ON, bit1=SENSOR_ON |

### IMU sample (16 B each)

| Offset (within sample) | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 2 | `ax` | int16, Hub body frame |
| 2 | 2 | `ay` | int16, Hub body frame |
| 4 | 2 | `az` | int16, Hub body frame |
| 6 | 2 | `gx` | int16, Hub body frame |
| 8 | 2 | `gy` | int16, Hub body frame |
| 10 | 2 | `gz` | int16, Hub body frame |
| 12 | 4 | `ts_delta_us` | uint32, sample timestamp − `tick_ts_us`; sample[0] = 0 |

**Axis convention**: SPIKE Prime Hub body frame.  The LSM6DSL is
mounted with chip Y/Z anti-parallel to the Hub body Y/Z, so the
driver negates Y and Z on the Hub before publishing.  X is unchanged.
With the Hub flat on a desk and its Z axis pointing up, accel reads
≈ (0, 0, +1 g) at rest.  No further rotation is needed on the PC
side.

### TLV (10 B header + 0..32 B payload, six entries concatenated)

Emitted in fixed class order (color → ultrasonic → force → motor_m →
motor_r → motor_l) every tick.

| Offset (within TLV) | Size | Field | Notes |
|---:|---:|---|---|
| 0 | 1 | `class_id` | 0..5 (`enum legosensor_class_e`) |
| 1 | 1 | `port_id` | 0..5 when BOUND, 0xFF otherwise |
| 2 | 1 | `mode_id` | LUMP mode index (0 when not bound) |
| 3 | 1 | `data_type` | 0:INT8 1:INT16 2:INT32 3:FLOAT |
| 4 | 1 | `num_values` | INFO_FORMAT[2] |
| 5 | 1 | `payload_len` | 0..32; 0 unless FRESH |
| 6 | 1 | `flags` | bit0=BOUND, bit1=FRESH (this tick has a new sample) |
| 7 | 1 | `age_10ms` | 10 ms units since last publish, 0xFF saturated |
| 8 | 2 | `seq` | `lump_sample_s.seq & 0xFFFF` |
| 10 | N | `payload` | only when FRESH=1 |

A TLV with `FRESH=0` still carries a header — the host keeps the
last-known payload from the most recent FRESH tick.  Hot-plug events
are observed via the `BOUND` flag.

### TLV payload decoding examples

Combine `data_type` and `num_values` to decode the payload.  Common
class × mode pairs:

| class       | mode | label  | data_type | num_values | unit / scale |
|-------------|---:|--------|-----------|---:|-----|
| color       |  0 | COLOR  | INT8      | 1 | colour index |
| color       |  1 | REFLT  | INT8      | 1 | reflected light % |
| color       |  2 | AMBI   | INT8      | 1 | ambient light % |
| color       |  5 | RGB I  | INT16     | 4 | R, G, B, IR (raw 0..1024) |
| color       |  6 | HSV    | INT16     | 3 | H°, S, V |
| ultrasonic  |  0 | DISTL  | INT16     | 1 | distance mm |
| ultrasonic  |  1 | DISTS  | INT16     | 1 | distance mm (short range) |
| force       |  0 | FORCE  | INT8      | 1 | force % |
| force       |  1 | TOUCH  | INT8      | 1 | touched (0/1) |
| motor_m/r/l |  1 | SPEED  | INT8      | 1 | speed -100..+100 % |
| motor_m/r/l |  2 | POS    | INT32     | 1 | angle ° (relative) |
| motor_m/r/l |  3 | APOS   | INT16     | 1 | angle ° (0..359) |

Example: a color sensor TLV with mode 6 (HSV) and
`payload = "B4 00 32 00 4B 00"` decodes (INT16 LE) to
`H=180°, S=50, V=75`.

The host implementation lives in
`host/ImuViewer/src/ImuViewer.Core/LegoSensor/ScaleTables.cs` (hard-
coded table).  Modes outside the table fall back to raw integer values.

### Resync after byte loss

1. Find the next `0xB66B` magic.
2. Read the 5-byte envelope; sanity-check `type == 0x02` and that
   `frame_len` is at least 21 bytes and within the worst-case bound.
3. If anything looks bogus, advance by 1 and rescan from step 1.
4. Wait for `frame_len` bytes total, parse the bundle header, the IMU
   subsection, and the six TLVs in order.
5. If the cumulative TLV size + IMU section size + headers does not
   equal `frame_len`, advance by 1 and resync.

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

MAGIC          = 0xB66B
ENVELOPE_FMT   = "<HBH"          # magic, type, frame_len
ENVELOPE_SIZE  = 5
HDR_FMT        = "<HIHBBHBHB"    # seq, tick_ts_us, imu_section_len,
                                  # imu_sample_count, tlv_count, sample_rate,
                                  # accel_fsr_g, gyro_fsr_dps, flags
HDR_SIZE       = 16
SAMPLE_FMT     = "<hhhhhhI"      # ax ay az gx gy gz ts_delta_us
SAMPLE_SIZE    = 16
TLV_HDR_FMT    = "<BBBBBBBBH"    # class_id, port_id, mode_id, data_type,
                                  # num_values, payload_len, flags, age_10ms, seq
TLV_HDR_SIZE   = 10
TLV_COUNT      = 6
CLASS_NAMES    = ["color", "ultrasonic", "force",
                  "motor_m", "motor_r", "motor_l"]


def parse_bundle(buf, offset, length):
    end = offset + length
    (magic, typ, frame_len) = struct.unpack_from(ENVELOPE_FMT, buf, offset)
    assert magic == MAGIC and typ == 0x02
    p = offset + ENVELOPE_SIZE

    (seq, tick_ts_us, imu_section_len, imu_sample_count, tlv_count,
     sample_rate, accel_fsr_g, gyro_fsr_dps, flags) = \
        struct.unpack_from(HDR_FMT, buf, p)
    p += HDR_SIZE

    accel_mg_lsb = (accel_fsr_g * 1000.0) / 32768.0
    gyro_dps_lsb = (gyro_fsr_dps * 0.035) / 1000.0

    print(f"seq={seq} tick_ts_us={tick_ts_us} imu_n={imu_sample_count} "
          f"odr={sample_rate} accel_fsr={accel_fsr_g}g "
          f"gyro_fsr={gyro_fsr_dps}dps flags=0x{flags:02x}")

    for i in range(imu_sample_count):
        ax, ay, az, gx, gy, gz, dt = struct.unpack_from(SAMPLE_FMT, buf, p)
        ts_us = (tick_ts_us + dt) & 0xffffffff
        print(f"  imu[{i}] ts_us={ts_us} dt_us={dt} "
              f"accel_mg=({ax*accel_mg_lsb:7.2f},{ay*accel_mg_lsb:7.2f},"
              f"{az*accel_mg_lsb:7.2f}) "
              f"gyro_dps=({gx*gyro_dps_lsb:7.3f},{gy*gyro_dps_lsb:7.3f},"
              f"{gz*gyro_dps_lsb:7.3f})")
        p += SAMPLE_SIZE

    for i in range(tlv_count):
        (class_id, port_id, mode_id, data_type, num_values,
         payload_len, tlv_flags, age_10ms, tlv_seq) = \
            struct.unpack_from(TLV_HDR_FMT, buf, p)
        p += TLV_HDR_SIZE
        payload = bytes(buf[p:p + payload_len])
        p += payload_len
        cname = CLASS_NAMES[class_id] if class_id < len(CLASS_NAMES) \
                else f"#{class_id}"
        bound = "bound" if (tlv_flags & 0x01) else "unbound"
        fresh = "FRESH" if (tlv_flags & 0x02) else ""
        port = "-" if port_id == 0xFF else f"{port_id}"
        print(f"  tlv {cname:<10} port={port} mode={mode_id} "
              f"{bound} {fresh} age={age_10ms*10}ms "
              f"payload({payload_len})={payload.hex()}")

    assert p == end


def parse_stream(stream):
    buf = b""
    while True:
        chunk = stream.read(256)
        if not chunk:
            break
        buf += chunk
        while len(buf) >= ENVELOPE_SIZE:
            idx = buf.find(b"\x6b\xb6")
            if idx < 0:
                buf = buf[-1:]
                break
            if idx > 0:
                buf = buf[idx:]
            if len(buf) < ENVELOPE_SIZE:
                break
            (magic, typ, frame_len) = struct.unpack_from(ENVELOPE_FMT, buf, 0)
            if magic != MAGIC or typ != 0x02 or frame_len < ENVELOPE_SIZE + HDR_SIZE:
                buf = buf[1:]
                continue
            if len(buf) < frame_len:
                break
            try:
                parse_bundle(buf, 0, frame_len)
            except (struct.error, AssertionError):
                buf = buf[1:]
                continue
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
| `IMU ON\n`  | Include the IMU subsection in the BUNDLE (auto-activates the uORB driver) | — |
| `IMU OFF\n` | Drop the IMU subsection | — |
| `SENSOR ON\n`  | Stream all 6 LEGO sensor TLVs | — |
| `SENSOR OFF\n` | Freeze TLVs to unbound/empty | — |
| `SENSOR MODE <class> <mode>\n` | Switch the bound device to the given LUMP mode | only while SENSOR ON |
| `SENSOR SEND <class> <mode> <hex>\n` | Write hex bytes to a writable mode (LEDs etc.) | only while SENSOR ON |
| `SENSOR PWM <class> <ch0> [ch1..ch3]\n` | LED brightness / motor duty (-100..100; color=3ch, ultrasonic=4ch, motor_*=1ch) | only while SENSOR ON |
| `SET ODR <hz>\n` | Set ODR (13/26/52/104/208/416/833 Hz) | only while IMU OFF.  **`> 833` returns `ERR invalid_odr`** |
| `SET ACCEL_FSR <g>\n` | Accel FSR (2/4/8/16) | only while IMU OFF |
| `SET GYRO_FSR <dps>\n` | Gyro FSR (125/250/500/1000/2000) | only while IMU OFF |

Reply patterns:
- `OK\n` on success
- `ERR busy\n` if a `SET *` arrives while sampling is on
- `ERR invalid_odr\n` for ODR > 833
- `ERR invalid <token>\n` for malformed values / unknown subcommands
- `ERR overflow\n` for lines longer than 64 bytes
- `ERR unknown <cmd>\n` for an unrecognised first token

IMU and SENSOR are both **off** at daemon start; while both are off the
100 Hz BUNDLE timer is parked so no bytes flow.  A typical session:

```text
PC -> Hub:  IMU OFF\n              (idempotent)
Hub -> PC:  OK\n
PC -> Hub:  SET ODR 416\n
Hub -> PC:  OK\n
PC -> Hub:  IMU ON\n
Hub -> PC:  OK\n
PC -> Hub:  SENSOR ON\n
Hub -> PC:  OK\n
            ... 100 Hz BUNDLE frames flow ...
PC -> Hub:  IMU OFF\n
PC -> Hub:  SENSOR OFF\n
Hub -> PC:  OK\n
Hub -> PC:  OK\n
```

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Linux: `cat: /dev/rfcomm0: Cannot allocate memory` | lazy RFCOMM bind failed to open | Switch to `rfcomm connect` (foreground).  A persistent `Permission denied` instead usually means stale link keys — remove + re-pair in `bluetoothctl` |
| Linux: `Permission denied` | Hub's in-RAM link-key DB was wiped by the latest flash but the host still holds the stale key | `bluetoothctl remove <MAC>` then pair again |
| macOS: `cat` returns nothing | macOS tty open alone does not establish RFCOMM | Use the PyObjC `IOBluetoothRFCOMMChannel` path above |
| macOS: no "Connect" button in Settings | CoD 0x001F00 is filtered by macOS UI | Drive pairing with `blueutil --pair` from the CLI |
| `dropped_oldest` counter rising | RFCOMM send can't keep up (e.g. PC disconnected) | Hub ring is 8 frames deep; drops oldest on overflow by design (drop-oldest) |

## See also

- `host/ImuViewer/` — desktop visualizer (.NET 10 + Avalonia + Silk.NET) that
  consumes this stream, runs a Madgwick filter, and renders a 3D Cube whose
  orientation tracks the Hub. Linux PoC; macOS / Windows are stubbed.
- btstack `example/spp_counter.c` — minimal SPP server reference
- BlueZ `rfcomm(1)` — Linux CLI reference
- [IOBluetoothRFCOMMChannel Class Reference](https://developer.apple.com/documentation/iobluetooth/iobluetoothrfcommchannel) — macOS API
