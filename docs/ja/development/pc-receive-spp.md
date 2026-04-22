# PC で SPP ストリームを受信する

SPIKE Prime Hub (`btsensor` アプリ、Issue #52) は Classic Bluetooth SPP (RFCOMM channel 1) で IMU サンプルを Little-endian バイナリで送出する。本稿では Linux / macOS から受信する具体手順と、パース用 Python スクリプトを示す。

フレーム構造は [CC2564C Bluetooth ドライバ](../drivers/bluetooth.md) の「RFCOMM ペイロード」節を参照。

## Linux (BlueZ) で受信

### 必要パッケージ

```bash
# Ubuntu/Debian
sudo apt install bluez bluez-tools

# Fedora
sudo dnf install bluez bluez-tools
```

`rfcomm` コマンドが含まれる `bluez-tools` (または配布によって `bluez-compat`) が必要。`which rfcomm` で存在確認。

### ペアリング

Hub で `btsensor &` が起動している状態で:

```bash
bluetoothctl
[bluetooth]# power on
[bluetooth]# scan on
# SPIKE-BT-Sensor が見えるまで 5〜10 秒待つ
[bluetooth]# pair F8:2E:0C:A0:3E:64
# "Pairing successful" が出ること
[bluetooth]# trust F8:2E:0C:A0:3E:64
[bluetooth]# quit
```

MAC アドレスは Hub 起動時の `HCI working, BD_ADDR ...` メッセージか `bluetoothctl info` で確認。

### SDP で SPP が見えるか確認

```bash
sdptool browse F8:2E:0C:A0:3E:64
```

`Service Name: SPIKE IMU Stream` / `Channel: 1` / `Profile: SPP (v1.2)` が出力されれば OK。

### RFCOMM 接続 + 読み出し

```bash
# tty を RFCOMM 上に bind (device 番号 0)
sudo rfcomm bind 0 F8:2E:0C:A0:3E:64 1
ls -l /dev/rfcomm0

# Hub 側で RFCOMM 接続を開かせる (bind は lazy なのでここで初めて dial)
sudo cat /dev/rfcomm0 | xxd | head
```

先頭に `5a a5 ...` (magic 0xA55A の little-endian) + 続く 4 byte の seq / timestamp + 2 byte sample_rate + sample_count + type があれば成功。

スループット測定:

```bash
sudo cat /dev/rfcomm0 | pv > /dev/null
# 833 Hz × 12 byte = 10 KB/s 前後を想定
```

終了時は `sudo rfcomm release 0`。

### 代替: `rfcomm connect` (foreground)

`rfcomm bind` の lazy open で `cat: /dev/rfcomm0: メモリを確保できません` が出るときは、`rfcomm connect` で明示的に session を張って tty を保持させる:

```bash
# foreground で動き続ける
sudo rfcomm connect 0 F8:2E:0C:A0:3E:64 1
# 別端末で
sudo cat /dev/rfcomm0 | xxd | head
```

`Permission denied` が出る場合は stale link key が残っている可能性。`bluetoothctl remove` → `pair` をやり直す。

## macOS IOBluetooth で受信

macOS の Classic BT SPP はややクセがある:

1. System Settings → Bluetooth の GUI は Class of Device = "Uncategorized" の汎用 SPP デバイスに "接続" ボタンを出さない (近くのデバイスリストには見える)。
2. ペアリング自体は CLI の [blueutil](https://github.com/toy/blueutil) で完結する。
3. 生 tty (`/dev/tty.SPIKE-BT-Sensor-*`) は pair 後に自動生成されるが、`cat`/`screen` で開いても macOS は RFCOMM session を張らない。`IOBluetoothRFCOMMChannel` API (pyobjc 経由の IOBluetooth フレームワーク) で明示的に接続する必要がある。

### 必要ツール

```bash
brew install blueutil
python3 -m pip install pyobjc-framework-IOBluetooth
```

### ペアリング

Hub で `btsensor &` が起動している状態で:

```bash
# 検索 (数秒)
blueutil --inquiry

# pair (sudo 不要、BLUEUTIL_ALLOW_ROOT=1 が必要なら env で)
blueutil --pair f8-2e-0c-a0-3e-64
# Hub 側の NSH に "SSP confirm" → "SSP pairing with ... status 0x00" が出る
```

### RFCOMM 接続 + 読み出し (Python)

`/dev/tty.SPIKE-BT-Sensor` は存在するが RFCOMM 非活性なので、IOBluetoothRFCOMMChannel を直接叩く必要がある。

```python
# pc_receive_spp.py
import struct, sys, time
import objc
from Foundation import NSObject, NSRunLoop, NSDate
import IOBluetooth

ADDR = "f8-2e-0c-a0-3e-64"


class RFCOMMDelegate(NSObject):
    def rfcommChannelOpenComplete_status_(self, channel, status):
        print(f"channel open status={status}", flush=True)

    def rfcommChannelData_data_length_(self, channel, data, length):
        buf = bytes(data.bytes().tobytes()[:length]) if hasattr(data, 'bytes') else bytes(data)[:length]
        print(buf.hex(), flush=True)

    def rfcommChannelClosed_(self, channel):
        print("channel closed", flush=True)


def main():
    dev = IOBluetooth.IOBluetoothDevice.deviceWithAddressString_(ADDR)
    delegate = RFCOMMDelegate.alloc().init()
    result = dev.openRFCOMMChannelSync_withChannelID_delegate_(None, 1, delegate)
    print("open result:", result)

    # run loop
    deadline = NSDate.dateWithTimeIntervalSinceNow_(60.0)
    NSRunLoop.currentRunLoop().runUntilDate_(deadline)


if __name__ == "__main__":
    main()
```

実行:

```bash
python3 pc_receive_spp.py
```

Hub 側に `RFCOMM incoming` → `RFCOMM open cid=...` が出ればデータが流れる。

## フレームパーサ (Python 共通)

Linux / macOS 共通で使えるバイナリパーサ:

```python
# btsensor_parser.py
import struct, sys, glob

MAGIC = 0xA55A
HDR_FMT = "<HHIHBB"     # magic, seq, ts_us, rate, count, type
HDR_SIZE = 12
SAMPLE_FMT = "<hhhhhh"  # ax ay az gx gy gz
SAMPLE_SIZE = 12

# LSM6DS3 scales
ACCEL_MG_LSB = 0.244    # ±8 g
GYRO_DPS_LSB = 0.070    # ±2000 dps


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
                ax_mg = ax * ACCEL_MG_LSB
                ay_mg = ay * ACCEL_MG_LSB
                az_mg = az * ACCEL_MG_LSB
                gx_dps = gx * GYRO_DPS_LSB
                gy_dps = gy * GYRO_DPS_LSB
                gz_dps = gz * GYRO_DPS_LSB
                print(f"seq={seq} ts_us={ts} "
                      f"accel_mg=({ax_mg:.1f},{ay_mg:.1f},{az_mg:.1f}) "
                      f"gyro_dps=({gx_dps:.2f},{gy_dps:.2f},{gz_dps:.2f})")
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

実行:

```bash
# Linux
sudo python3 btsensor_parser.py /dev/rfcomm0

# macOS: Python IOBluetooth で RFCOMM を開いた delegate から生バイトを
# このパーサに流し込む
```

Hub 本体を静置すると accel_z ≈ ±1000 mg (±1 g)、振ると peak が立つ。gyro は静置で ≈ 0 dps。

## よくあるトラブル

| 症状 | 原因 | 対処 |
|------|------|------|
| Linux: `cat: /dev/rfcomm0: メモリを確保できません` | `rfcomm bind` の lazy open で RFCOMM session 失敗 | `rfcomm connect` (foreground) に切替。なお `Permission denied` が続く場合は stale link key: `bluetoothctl remove` で unpair → 再 pair |
| Linux: `Permission denied` | 再起動後で Hub 側 link key DB (in-memory) が空 | Linux 側を一度 `remove` → 再 pair |
| macOS: cat で何も流れない | IOBluetooth tty の open だけでは RFCOMM を張らない仕様 | pyobjc の `IOBluetoothRFCOMMChannel` で明示的に開く (上記 Python) |
| macOS: "接続" ボタンが出ない | Uncategorized CoD に対する macOS の UI ポリシー | blueutil CLI 経由で pair 完結 |
| drop counter 増加 | RFCOMM 送信が追いつかない (PC 側切断など) | Hub 側 ring は 8 frame でオーバーフロー時に古いフレームから破棄。正常動作 |

## 参考

- btstack `example/spp_counter.c` — SPP server 最小例
- BlueZ `rfcomm(1)` man — Linux 側 CLI 仕様
- [IOBluetoothRFCOMMChannel Class Reference](https://developer.apple.com/documentation/iobluetooth/iobluetoothrfcommchannel) — macOS API
