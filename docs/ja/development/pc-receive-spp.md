# PC で SPP ストリームを受信する

SPIKE Prime Hub (`btsensor` アプリ) は Classic Bluetooth SPP (RFCOMM channel 1) で **100 Hz BUNDLE** バイナリフレームを送出する。1 フレームに IMU サンプル + 6 種 LEGO Powered Up センサー (color/ultrasonic/force/motor_m/motor_r/motor_l) のステータスがまとまっている。本稿では Linux / macOS から受信する具体手順、BUNDLE フレーム形式、ASCII コマンドプロトコル、パース用 Python スクリプトを示す。

!!! warning "Issue #88 でフレーム形式変更"
    magic は `0xB66B` のまま、type が `0x01` (IMU 単独) → `0x02` (BUNDLE) に変わった。frame_len の位置がオフセット 16 → 3 へ移動し、ヘッダ全体のレイアウトが刷新されている。Issue #88 より前のスクリプトは新ストリームを parse できないので、下記の最新パーサに置き換えること。`SET BATCH` コマンドは廃止。`SET ODR` の上限は 833 Hz に制限 (>833 は `ERR invalid_odr`)。新コマンド `SENSOR ON|OFF` を追加。

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
(`btsensor start` → BT ボタン短押し、または PC から `btsensor bt on`):

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

# Hub 側 daemon は IMU OFF / SENSOR OFF で起動するので、まず IMU ON / SENSOR ON
# を送ってから読み出す (lazy open なので初めて RFCOMM session が張られる)
printf 'IMU ON\n' | sudo tee /dev/rfcomm0 > /dev/null
printf 'SENSOR ON\n' | sudo tee /dev/rfcomm0 > /dev/null
sudo cat /dev/rfcomm0 | xxd | head
```

先頭が `6b b6 02 LL LL ...` (magic 0xB66B + type 0x02 BUNDLE + frame_len LE) と続けば成功。

スループット測定 (10 ms tick × 平均 ~233 B = 約 22 KB/s ≈ 186 kbps):

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

## フレーム形式 (BUNDLE / Issue #88)

すべて little-endian。100 Hz tick ごとに 1 フレーム emit。1 フレーム =
**envelope (5 B) + bundle header (16 B) + IMU サブセクション (0..8 サンプル × 16 B) + TLV サブセクション (常に 6 個、可変長)**。
全 6 LEGO センサークラス (color / ultrasonic / force / motor_m / motor_r / motor_l) は毎 tick TLV を送出し、`flags` の `BOUND` / `FRESH` ビットで生存と新着を表現する。

### Envelope (5 B)

| Offset | Size | Field | 備考 |
|---:|---:|---|---|
| 0 | 2 | `magic` | 常に `0xB66B` |
| 2 | 1 | `type` | `0x02` = BUNDLE |
| 3 | 2 | `frame_len` | envelope 含む全長 |

### Bundle header (16 B, offset 5..20)

| Offset | Size | Field | 備考 |
|---:|---:|---|---|
| 5  | 2 | `seq` | バンドル単調連番 |
| 7  | 4 | `tick_ts_us` | この tick の最古 IMU sample 絶対時刻 (`CLOCK_BOOTTIME` µs の low 32 bit、サンプル 0 個なら現在時刻)。`ts_delta_us` は常に ≥ 0 |
| 11 | 2 | `imu_section_len` | = `imu_sample_count × 16` |
| 13 | 1 | `imu_sample_count` | 0..8 |
| 14 | 1 | `tlv_count` | 常に 6 |
| 15 | 2 | `imu_sample_rate_hz` | 現 ODR (≤833) |
| 17 | 1 | `imu_accel_fsr_g` | 2 / 4 / 8 / 16 |
| 18 | 2 | `imu_gyro_fsr_dps` | 125 / 250 / 500 / 1000 / 2000 |
| 20 | 1 | `flags` | bit0=IMU_ON, bit1=SENSOR_ON |

### IMU サブセクション (16 byte / サンプル)

| Offset (内) | Size | Field | 備考 |
|---:|---:|---|---|
| 0 | 2 | `ax` | int16 Hub body frame |
| 2 | 2 | `ay` | int16 Hub body frame |
| 4 | 2 | `az` | int16 Hub body frame |
| 6 | 2 | `gx` | int16 Hub body frame |
| 8 | 2 | `gy` | int16 Hub body frame |
| 10 | 2 | `gz` | int16 Hub body frame |
| 12 | 4 | `ts_delta_us` | uint32、サンプルの timestamp − `tick_ts_us`、sample[0] = 0 |

**軸方向**: SPIKE Prime Hub body frame。LSM6DSL は Y/Z が Hub 筐体軸と逆向きにマウントされているため、Hub 側ドライバが Y/Z を反転させてから publish する (X はそのまま)。Hub を机に置いて Z 軸を上に向けた状態で accel ≒ (0, 0, +1 g) になる。PC 側で追加回転を当てる必要は無い。

### TLV サブフレーム (10 byte ヘッダ + 0..32 byte payload, 6 個固定連結)

クラス順 (color → ultrasonic → force → motor_m → motor_r → motor_l) で毎 tick 送出。

| Offset (内) | Size | Field | 備考 |
|---:|---:|---|---|
| 0 | 1 | `class_id` | 0..5 (`enum legosensor_class_e`) |
| 1 | 1 | `port_id` | 0..5 (BOUND 時) / 0xFF (未バインド) |
| 2 | 1 | `mode_id` | LUMP モード番号 (BOUND=false なら 0) |
| 3 | 1 | `data_type` | 0:INT8 1:INT16 2:INT32 3:FLOAT |
| 4 | 1 | `num_values` | INFO_FORMAT[2] |
| 5 | 1 | `payload_len` | 0..32 (FRESH=0 のときは 0) |
| 6 | 1 | `flags` | bit0=BOUND, bit1=FRESH |
| 7 | 1 | `age_10ms` | 最終 publish からの経過 10ms 単位、0xFF saturated |
| 8 | 2 | `seq` | `lump_sample_s.seq & 0xFFFF` |
| 10 | N | `payload` | FRESH=1 のときのみ |

`FRESH=0` の tick も TLV header は出る (Viewer 側で last-known を hold する)。デバイス挿抜は `BOUND` フラグの遷移で観測。

### TLV ペイロード decode 例

`data_type` と `num_values` を組み合わせて payload を decode する。代表的な class × mode の組合せ:

| class       | mode | label  | data_type | num_values | unit / scale |
|-------------|---:|--------|-----------|---:|-----|
| color       |  0 | COLOR  | INT8      | 1 | カラーインデックス |
| color       |  1 | REFLT  | INT8      | 1 | 反射光 % |
| color       |  2 | AMBI   | INT8      | 1 | 環境光 % |
| color       |  5 | RGB I  | INT16     | 4 | R, G, B, IR (raw 0..1024) |
| color       |  6 | HSV    | INT16     | 3 | H°, S, V |
| ultrasonic  |  0 | DISTL  | INT16     | 1 | 距離 mm |
| ultrasonic  |  1 | DISTS  | INT16     | 1 | 距離 mm (短距離精度) |
| force       |  0 | FORCE  | INT8      | 1 | 力 % |
| force       |  1 | TOUCH  | INT8      | 1 | 接触 (0/1) |
| motor_m/r/l |  1 | SPEED  | INT8      | 1 | 速度 -100..+100 % |
| motor_m/r/l |  2 | POS    | INT32     | 1 | 角度 ° (relative) |
| motor_m/r/l |  3 | APOS   | INT16     | 1 | 角度 ° (0..359) |

例: color sensor mode 6 (HSV) で `payload = "B4 00 32 00 4B 00"` を受信した場合、INT16 LE で decode して `H=180°, S=50, V=75`。

実装は `host/ImuViewer/src/ImuViewer.Core/LegoSensor/ScaleTables.cs` のハードコード表を参照。未対応のモードは raw int 値で表示される。

### 部分ロス時の resync

1. 次の `0xB66B` magic を探す
2. envelope 5 byte 読んで `type == 0x02` / `frame_len` が妥当 (>= 21、上限以下) を sanity check
3. 不正なら +1 byte ずらして step 1 に戻る
4. `frame_len` byte 揃うまで待ち、bundle header + IMU セクション + 6 TLV を順次 parse
5. 6 TLV 全部 + IMU サブセクション長が `frame_len` と一致しなければ +1 byte ずらして再同期

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

Linux / macOS 共通で使える BUNDLE バイナリパーサ:

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
    """Parse a single BUNDLE frame.  buf[offset:offset+length] must be the
    complete frame (envelope + header + IMU + 6 TLVs)."""
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

    # IMU subsection
    for i in range(imu_sample_count):
        ax, ay, az, gx, gy, gz, dt = struct.unpack_from(
            SAMPLE_FMT, buf, p)
        ts_us = (tick_ts_us + dt) & 0xffffffff
        print(f"  imu[{i}] ts_us={ts_us} dt_us={dt} "
              f"accel_mg=({ax*accel_mg_lsb:7.2f},{ay*accel_mg_lsb:7.2f},"
              f"{az*accel_mg_lsb:7.2f}) "
              f"gyro_dps=({gx*gyro_dps_lsb:7.3f},{gy*gyro_dps_lsb:7.3f},"
              f"{gz*gyro_dps_lsb:7.3f})")
        p += SAMPLE_SIZE

    # TLV subsection
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
| `IMU ON\n`  | IMU を BUNDLE に含める (driver 自動 activate) | — |
| `IMU OFF\n` | IMU を BUNDLE から外す | — |
| `SENSOR ON\n`  | 全 6 LEGO センサークラスの TLV streaming on | — |
| `SENSOR OFF\n` | LEGO センサー TLV を未バインド表示に固定 | — |
| `SENSOR MODE <class> <mode>\n` | バインド中のデバイスをモード切替 | SENSOR ON 時のみ |
| `SENSOR SEND <class> <mode> <hex>\n` | writable mode に hex bytes 書込み (LED 等) | SENSOR ON 時のみ |
| `SENSOR PWM <class> <ch0> [ch1..ch3]\n` | PWM/LED 強度設定 (-100..100、color=3ch / ultrasonic=4ch / motor=1ch) | SENSOR ON 時のみ |
| `SET ODR <hz>\n` | ODR 変更 (13/26/52/104/208/416/833 Hz) | IMU OFF 時のみ。**>833 は `ERR invalid_odr`** |
| `SET ACCEL_FSR <g>\n` | 加速度 FSR (2/4/8/16) | IMU OFF 時のみ |
| `SET GYRO_FSR <dps>\n` | ジャイロ FSR (125/250/500/1000/2000) | IMU OFF 時のみ |

応答パターン:
- 成功: `OK\n`
- IMU ON 中の `SET *`: `ERR busy\n`
- ODR > 833: `ERR invalid_odr\n`
- 不正な値 / トークン: `ERR invalid <token>\n`
- 行が 64 byte 超: `ERR overflow\n`
- 未知のコマンド: `ERR unknown <cmd>\n`

daemon 起動直後は IMU/SENSOR とも **off** で、両方 OFF の間は 100 Hz timer 自体が止まっているので 1 byte も流れない。典型的なセッション:

```text
PC -> Hub:  IMU OFF\n              (idempotent)
Hub -> PC:  OK\n
PC -> Hub:  SET ODR 416\n
Hub -> PC:  OK\n
PC -> Hub:  IMU ON\n
Hub -> PC:  OK\n
PC -> Hub:  SENSOR ON\n
Hub -> PC:  OK\n
            ... 100 Hz BUNDLE フレームが流れる ...
PC -> Hub:  IMU OFF\n
PC -> Hub:  SENSOR OFF\n
Hub -> PC:  OK\n
Hub -> PC:  OK\n
```

## よくあるトラブル

| 症状 | 原因 | 対処 |
|------|------|------|
| Linux: `cat: /dev/rfcomm0: メモリを確保できません` | `rfcomm bind` の lazy open で RFCOMM session 失敗 | `rfcomm connect` (foreground) に切替。なお `Permission denied` が続く場合は stale link key: `bluetoothctl remove` で unpair → 再 pair |
| Linux: `Permission denied` | 再起動後で Hub 側 link key DB (in-memory) が空 | Linux 側を一度 `remove` → 再 pair |
| macOS: cat で何も流れない | IOBluetooth tty の open だけでは RFCOMM を張らない仕様 | pyobjc の `IOBluetoothRFCOMMChannel` で明示的に開く (上記 Python) |
| macOS: "接続" ボタンが出ない | Uncategorized CoD に対する macOS の UI ポリシー | blueutil CLI 経由で pair 完結 |
| `dropped_oldest` カウンタ増加 | RFCOMM 送信が追いつかない (PC 側切断など) | Hub 側 ring は 8 frame でオーバーフロー時に古いフレームから破棄 (drop-oldest)。正常動作 |

## 参考

- `host/ImuViewer/` — このストリームを受信し Madgwick filter で姿勢推定して
  Cube を 3D 表示するデスクトップアプリ (.NET 10 + Avalonia + Silk.NET)。
  Linux PoC、macOS / Windows は stub
- btstack `example/spp_counter.c` — SPP server 最小例
- BlueZ `rfcomm(1)` man — Linux 側 CLI 仕様
- [IOBluetoothRFCOMMChannel Class Reference](https://developer.apple.com/documentation/iobluetooth/iobluetoothrfcommchannel) — macOS API
