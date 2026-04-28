# PC で SPP ストリームを受信する

SPIKE Prime Hub (`btsensor` アプリ) は Classic Bluetooth SPP (RFCOMM channel 1) で IMU サンプルを Little-endian バイナリで送出する。本稿では Linux / macOS から受信する具体手順、フレーム形式、ASCII コマンドプロトコル、パース用 Python スクリプトを示す。

!!! warning "Issue #56 Commit E でフレーム形式変更"
    magic が `0xA55A` から `0xB66B` に変わり、サンプル長が 12 → 16 byte (`ts_delta_us` 追加) になった。Commit E より前の `0xA55A` をハードコードした PC スクリプトでは新ストリームを parse できないので、下記の最新パーサに置き換えること。

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

Hub で daemon が起動し BT advertising が出ている状態で
(`btsensor start [batch]` → BT ボタン短押し、または PC から `btsensor bt on`):

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

# Hub 側 daemon は IMU OFF で起動するので、まず IMU ON を送ってから読み出す
# (lazy open なのでここで初めて RFCOMM session が張られる)
printf 'IMU ON\n' | sudo tee /dev/rfcomm0 > /dev/null
sudo cat /dev/rfcomm0 | xxd | head
```

先頭が `6b b6 ...` (magic 0xB66B の little-endian) + type / sample_count / rate / FSR / seq / first_sample_ts / frame_len と続けば成功。

スループット測定 (default batch=8 / 833 Hz → 18 + 8×16 = 146 B/frame × 104 Hz ≈ 15 KB/s ≈ 121 kbps):

```bash
sudo cat /dev/rfcomm0 | pv > /dev/null
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

Hub で daemon が起動している状態で (`btsensor start`):

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

## フレーム形式 (Commit E layout)

すべて little-endian。1 フレーム = 18 byte ヘッダ + N × 16 byte サンプル (N = `sample_count`、1〜80、デフォルト 8)。

### ヘッダ (18 byte)

| Offset | Size | Field | 備考 |
|---:|---:|---|---|
| 0 | 2 | `magic` | 常に `0xB66B` |
| 2 | 1 | `type` | `0x01` = IMU |
| 3 | 1 | `sample_count` | 1〜80 |
| 4 | 2 | `sample_rate_hz` | 現 ODR |
| 6 | 2 | `accel_fsr_g` | 2 / 4 / 8 / 16 |
| 8 | 2 | `gyro_fsr_dps` | 125 / 250 / 500 / 1000 / 2000 |
| 10 | 2 | `seq` | フレーム単調連番 |
| 12 | 4 | `first_sample_ts_us` | `CLOCK_BOOTTIME` µs の low 32 bit (mod 2³²、約 71m35s で wrap — 長時間キャプチャ時は PC 側で unwrap) |
| 16 | 2 | `frame_len` | `= 18 + sample_count × 16` |

### サンプル (16 byte/個)

| Offset (内) | Size | Field | 備考 |
|---:|---:|---|---|
| 0 | 2 | `ax` | int16 Hub body frame |
| 2 | 2 | `ay` | int16 Hub body frame |
| 4 | 2 | `az` | int16 Hub body frame |
| 6 | 2 | `gx` | int16 Hub body frame |
| 8 | 2 | `gy` | int16 Hub body frame |
| 10 | 2 | `gz` | int16 Hub body frame |
| 12 | 4 | `ts_delta_us` | uint32、サンプルの timestamp − `first_sample_ts_us`、sample[0] = 0 |

**軸方向**: SPIKE Prime Hub body frame。LSM6DSL は Y/Z が Hub 筐体軸と逆向きにマウントされているため、Hub 側ドライバが Y/Z を反転させてから publish する (X はそのまま)。Hub を机に置いて Z 軸を上に向けた状態で accel ≒ (0, 0, +1 g) になる。PC 側で追加回転を当てる必要は無い。

### 部分ロス時の resync

`frame_len` を使えばフレーム途中切れに耐性を持たせられる:

1. 次の `0xB66B` magic を探す
2. 18 byte 読んで `type == 0x01` / `1 ≤ sample_count ≤ 80` / `frame_len == 18 + sample_count × 16` を sanity check
3. `frame_len` がおかしければ +1 して step 1 に戻る
4. 以降 `frame_len` byte 揃うまで待ち、揃ったら parse

### 物理量への換算

LSM6DSL データシートの per-FSR sensitivity を生値に乗算:

| FSR | accel sensitivity (mg/LSB) | gyro sensitivity (mdps/LSB) |
|---|---|---|
| ±2 g  / ±125 dps  | 0.061 | 4.375 |
| ±4 g  / ±250 dps  | 0.122 | 8.75  |
| ±8 g  / ±500 dps  | 0.244 | 17.5  |
| ±16 g / ±1000 dps | 0.488 | 35.0  |
| (–)   / ±2000 dps | (–)   | 70.0  |

ヘッダ FSR フィールドから動的に算出する場合:

```python
accel_mg_lsb = (accel_fsr_g  * 1000.0) / 32768.0
gyro_dps_lsb = (gyro_fsr_dps * 0.035) / 1000.0
# accel_mps2 = raw * accel_mg_lsb * 9.80665 / 1000
# gyro_dps   = raw * gyro_dps_lsb
```

(accel は int16 full-scale が FSR に正確一致。gyro は仕様の都合で約 14.7% headroom — datasheet Table 3 参照。)

## フレームパーサ (Python 共通)

Linux / macOS 共通で使えるバイナリパーサ:

```python
# btsensor_parser.py
import struct, sys, glob

MAGIC       = 0xB66B
HDR_FMT     = "<HBBHHHHIH"   # magic, type, count, rate, accel_fsr,
                             # gyro_fsr, seq, first_ts_us, frame_len
HDR_SIZE    = 18
SAMPLE_FMT  = "<hhhhhhI"     # ax ay az gx gy gz ts_delta_us
SAMPLE_SIZE = 16


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

実行:

```bash
# Linux: 先に `printf 'IMU ON\n' | sudo tee /dev/rfcomm0` で sampling 開始
sudo python3 btsensor_parser.py /dev/rfcomm0

# macOS: IOBluetoothRFCOMMChannel delegate から流れる生バイトを
# parse_stream に渡す
```

Hub 本体を静置すると `accel_z ≈ ±1000 mg` (±1g、デフォルト ±8g 設定時)、振ると peak が立つ。gyro は静置 + warm-up 後 ≈ 0 dps。

## ASCII コマンドプロトコル

RFCOMM open 後、同じチャネル上で 1 行 ASCII (`\n` 終端、`\r` 無視) のコマンドを送れる。コマンド応答は IMU バイナリストリームと同じチャネルを共有するが、応答は arbiter 側で次の telemetry フレームより必ず先に送出される。

| コマンド | 動作 | 制約 |
|---|---|---|
| `IMU ON\n`  | サンプリング開始 (driver 自動 activate) | — |
| `IMU OFF\n` | サンプリング停止 (driver 自動 deactivate) | — |
| `SET ODR <hz>\n` | ODR 変更 (13/26/52/104/208/416/833/1660/3330/6660 Hz) | IMU OFF 時のみ |
| `SET BATCH <n>\n` | フレームあたりサンプル数 (1〜80) | IMU OFF 時のみ |
| `SET ACCEL_FSR <g>\n` | 加速度 FSR (2/4/8/16) | IMU OFF 時のみ |
| `SET GYRO_FSR <dps>\n` | ジャイロ FSR (125/250/500/1000/2000) | IMU OFF 時のみ |

応答パターン:
- 成功: `OK\n`
- IMU ON 中の `SET *`: `ERR busy\n`
- 不正な値 / トークン: `ERR invalid <token>\n`
- 行が 64 byte 超: `ERR overflow\n`
- 未知のコマンド: `ERR unknown <cmd>\n`

daemon 起動直後はサンプリング **off** なので、典型的なセッションは:

```text
PC -> Hub:  IMU OFF\n              (idempotent)
Hub -> PC:  OK\n
PC -> Hub:  SET ODR 416\n
Hub -> PC:  OK\n
PC -> Hub:  SET BATCH 16\n
Hub -> PC:  OK\n
PC -> Hub:  IMU ON\n
Hub -> PC:  OK\n
            ... バイナリ IMU フレームが流れる ...
PC -> Hub:  IMU OFF\n
Hub -> PC:  OK\n
```

## よくあるトラブル

| 症状 | 原因 | 対処 |
|------|------|------|
| Linux: `cat: /dev/rfcomm0: メモリを確保できません` | `rfcomm bind` の lazy open で RFCOMM session 失敗 | `rfcomm connect` (foreground) に切替。なお `Permission denied` が続く場合は stale link key: `bluetoothctl remove` で unpair → 再 pair |
| Linux: `Permission denied` | 再起動後で Hub 側 link key DB (in-memory) が空 | Linux 側を一度 `remove` → 再 pair |
| macOS: cat で何も流れない | IOBluetooth tty の open だけでは RFCOMM を張らない仕様 | pyobjc の `IOBluetoothRFCOMMChannel` で明示的に開く (上記 Python) |
| macOS: "接続" ボタンが出ない | Uncategorized CoD に対する macOS の UI ポリシー | blueutil CLI 経由で pair 完結 |
| drop counter 増加 | RFCOMM 送信が追いつかない (PC 側切断など) | Hub 側 ring は 8 frame でオーバーフロー時に古いフレームから破棄。正常動作 |

## 参考

- `host/ImuViewer/` — このストリームを受信し Madgwick filter で姿勢推定して
  Cube を 3D 表示するデスクトップアプリ (.NET 10 + Avalonia + Silk.NET)。
  Linux PoC、macOS / Windows は stub
- btstack `example/spp_counter.c` — SPP server 最小例
- BlueZ `rfcomm(1)` man — Linux 側 CLI 仕様
- [IOBluetoothRFCOMMChannel Class Reference](https://developer.apple.com/documentation/iobluetooth/iobluetoothrfcommchannel) — macOS API
