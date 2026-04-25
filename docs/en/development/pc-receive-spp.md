# Receiving the SPP stream on a PC

The SPIKE Prime Hub `btsensor` app (Issue #52) emits IMU samples over
Classic Bluetooth SPP (RFCOMM channel 1) as a little-endian binary
stream.  This page walks through the host-side receive procedure for
Linux and macOS, plus a reusable Python parser.

The on-wire format lives under
[CC2564C Bluetooth driver — RFCOMM payload](../drivers/bluetooth.md).

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

With `btsensor &` running on the Hub:

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

# lazy open — Linux initiates the RFCOMM session on first I/O
sudo cat /dev/rfcomm0 | xxd | head
```

The first bytes should be `5a a5 ...` (magic `0xA55A` in little-endian)
followed by seq / timestamp / rate / sample_count / type + samples.

Throughput probe:

```bash
sudo cat /dev/rfcomm0 | pv > /dev/null
# expect ~10 KB/s (833 Hz × 12 bytes)
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

With `btsensor &` running:

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

## Shared Python parser

Works the same way on Linux (`/dev/rfcomm0`) and macOS (once you've
piped the IOBluetooth delegate's bytes into a stream):

```python
# btsensor_parser.py
import struct, sys, glob

MAGIC = 0xA55A
HDR_FMT = "<HHIHBB"     # magic, seq, ts_us, rate, count, type
HDR_SIZE = 12
SAMPLE_FMT = "<hhhhhh"  # ax ay az gx gy gz
SAMPLE_SIZE = 12

ACCEL_MG_LSB = 0.244    # LSM6DS3 ±8 g
GYRO_DPS_LSB = 0.070    # LSM6DS3 ±2000 dps


def parse_stream(stream):
    buf = b""
    while True:
        chunk = stream.read(256)
        if not chunk:
            break
        buf += chunk
        while len(buf) >= HDR_SIZE:
            idx = buf.find(b"\x5a\xa5")
            if idx < 0:
                buf = buf[-1:]
                break
            if idx > 0:
                buf = buf[idx:]
            if len(buf) < HDR_SIZE:
                break
            magic, seq, ts, rate, count, typ = struct.unpack(HDR_FMT,
                                                             buf[:HDR_SIZE])
            if magic != MAGIC:
                buf = buf[1:]
                continue
            need = HDR_SIZE + count * SAMPLE_SIZE
            if len(buf) < need:
                break
            for i in range(count):
                offs = HDR_SIZE + i * SAMPLE_SIZE
                ax, ay, az, gx, gy, gz = struct.unpack(
                    SAMPLE_FMT, buf[offs:offs + SAMPLE_SIZE])
                print(f"seq={seq} ts_us={ts} "
                      f"accel_mg=({ax*ACCEL_MG_LSB:.1f},"
                      f"{ay*ACCEL_MG_LSB:.1f},{az*ACCEL_MG_LSB:.1f}) "
                      f"gyro_dps=({gx*GYRO_DPS_LSB:.2f},"
                      f"{gy*GYRO_DPS_LSB:.2f},{gz*GYRO_DPS_LSB:.2f})")
            buf = buf[need:]


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
sudo python3 btsensor_parser.py /dev/rfcomm0
```

A motionless Hub reports `accel_z ≈ ±1000 mg` (±1 g) with the other
axes near zero; shake it to see peaks.  `gyro_dps` is ≈ 0 at rest.

## Troubleshooting

| Symptom | Cause | Fix |
|---|---|---|
| Linux: `cat: /dev/rfcomm0: Cannot allocate memory` | lazy RFCOMM bind failed to open | Switch to `rfcomm connect` (foreground).  A persistent `Permission denied` instead usually means stale link keys — remove + re-pair in `bluetoothctl` |
| Linux: `Permission denied` | Hub's in-RAM link-key DB was wiped by the latest flash but the host still holds the stale key | `bluetoothctl remove <MAC>` then pair again |
| macOS: `cat` returns nothing | macOS tty open alone does not establish RFCOMM | Use the PyObjC `IOBluetoothRFCOMMChannel` path above |
| macOS: no "Connect" button in Settings | CoD 0x001F00 is filtered by macOS UI | Drive pairing with `blueutil --pair` from the CLI |
| Frame drop counter rising | RFCOMM send can't keep up (e.g. PC disconnected) | Hub ring is 8 frames deep; drops oldest on overflow by design |

## See also

- btstack `example/spp_counter.c` — minimal SPP server reference
- BlueZ `rfcomm(1)` — Linux CLI reference
- [IOBluetoothRFCOMMChannel Class Reference](https://developer.apple.com/documentation/iobluetooth/iobluetoothrfcommchannel) — macOS API
