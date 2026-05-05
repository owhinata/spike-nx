# spike-nx

## Benchmark Results

### CoreMark

| Board | MCU | CoreMark Score | CoreMark/MHz | Compiler | Flags |
|-------|-----|---------------|-------------|----------|-------|
| SPIKE Prime Hub | STM32F413VG (96MHz) | 171.19 | 1.78 | GCC 13.2.1 | `-Os` |
| B-L4S5I-IOT01A | STM32L4R5VI (80MHz) | 143.16 | 1.79 | GCC 13.2.1 | `-Os` |

<details>
<summary>SPIKE Prime Hub raw output</summary>

```
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 1168261
Total time (secs): 11.682610
Iterations/Sec   : 171.194622
Iterations       : 2000
Compiler version : GCC13.2.1 20231009
Compiler flags   : -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g
Memory location  : HEAP
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x4983
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 171.194622 / GCC13.2.1 20231009 -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g / HEAP
```

</details>

<details>
<summary>B-L4S5I-IOT01A raw output</summary>

```
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 1396995
Total time (secs): 13.969950
Iterations/Sec   : 143.164435
Iterations       : 2000
Compiler version : GCC13.2.1 20231009
Compiler flags   : -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g
Memory location  : HEAP
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x4983
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 143.164435 / GCC13.2.1 20231009 -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g / HEAP
```

</details>

## Quick Start

### ビルド

```bash
make
```

### フラッシュ (DFU)

```bash
brew install dfu-util  # 初回のみ
```

1. Hub の USB を抜く
2. Bluetooth ボタンを押したまま USB 接続、5秒待って離す（DFU モード）
3. 書き込み (`usbnsh` 既定構成は BUILD_PROTECTED なので kernel + user の 2 段書き込み):

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
```

### シリアル接続

```bash
picocom /dev/tty.usbmodem01
```

### ファイル転送 (picocom + Zmodem)

外付け W25Q256 (32 MB SPI NOR) を `/mnt/flash` にマウント済み。`lrzsz` (`brew install lrzsz`) と picocom で PC ⇔ Hub のファイル転送が行える。

#### PC → Hub (アップロード)

```bash
picocom --send-cmd 'sz -vv -L 256' /dev/tty.usbmodem01
```

picocom 接続後:
1. `nsh> rz` 入力 (Hub を Zmodem 受信モードに)
2. `Ctrl-A Ctrl-S` → ローカルファイルパス入力 → Enter
3. "Transfer complete" まで待つ (1 MB ≒ 45 秒)
4. `Ctrl-A Ctrl-X` で picocom 終了

ファイルは `/mnt/flash/<basename>` に保存される。

#### Hub → PC (ダウンロード)

```bash
cd <受信先ディレクトリ>
picocom --receive-cmd 'rz -vv -y' /dev/tty.usbmodem01
```

picocom 接続後:
1. `nsh> sz /mnt/flash/file.bin` 入力 (Hub を Zmodem 送信モードに)
2. `Ctrl-A Ctrl-R` → 引数なしで Enter (`rz` がプロトコルから受信ファイル名を取得)
3. "Transfer complete" まで待つ
4. `Ctrl-A Ctrl-X` で picocom 終了

#### 整合性確認 (md5)

```
nsh> md5 -f /mnt/flash/file.bin     # Hub 側 (32-char hex, 改行なし)
$ md5sum file.bin                    # PC 側
```

#### 注意

- `sz -L 256` は **必須** — USB CDC は HW フロー制御がなく、デフォルト 1024 byte の subpacket では取りこぼしで ZNAK retry になる
- `-e` (escape control chars) は **付けない** — 8-bit clean な USB CDC で escape は不要かつ ZNAK の原因になる

詳細とトラブルシューティングは [docs/ja/development/file-transfer.md](docs/ja/development/file-transfer.md) と [docs/ja/usage/01-flash-storage.md](docs/ja/usage/01-flash-storage.md) を参照。

### BT 経由 NSH シェル (Issue #108)

USB ケーブルを外したまま (バッテリ駆動の走行ロボ等) PC から Hub の NSH を叩く。SPP RFCOMM 上で `MODE SHELL` / `MODE TELEMETRY` を排他切替する。

#### 初回のみ — ペアリング (Linux + BlueZ)

```bash
sudo apt install bluez bluez-tools                # 必要なら
```

Hub 側 (USB NSH):

```
nsh> btsensor start                # daemon 起動 (LED 消灯のまま)
nsh> btsensor bt on                # BT 可視化 (LED 青の slow-blink)
```

`dmesg` の `HCI working, BD_ADDR XX:XX:...` で Hub の BD アドレスを控える。

PC 側:

```bash
bluetoothctl
[bluetooth]# scan on
[bluetooth]# pair E0:FF:F1:5A:30:35
# Hub 側 dmesg に "SSP pairing with ... status 0x00" を確認
[bluetooth]# trust E0:FF:F1:5A:30:35
[bluetooth]# scan off
[bluetooth]# quit
```

#### 接続 (端末 2 枚)

**Terminal 1** — RFCOMM チャネル保持 (foreground):

```bash
sudo rfcomm connect 0 E0:FF:F1:5A:30:35 1
```

`Connected /dev/rfcomm0 to ...` のまま待機。`Ctrl-C` で切断。

**Terminal 2** — picocom:

```bash
sudo picocom -l --echo --omap crlf --imap lfcrlf /dev/rfcomm0
```

picocom の入力で:

```
MODE SHELL <Enter>          ← Hub に "MODE SHELL\n" 送信
OK                          ← 直後 NSH banner 表示
nsh> ls /dev <Enter>
nsh> dmesg <Enter>
nsh> exit <Enter>
READY                       ← telemetry mode 復帰
```

抜ける時: picocom 側 `Ctrl-A Ctrl-X` → Terminal 1 で `Ctrl-C`。

#### 注意

- `MODE SHELL` を送る前は telemetry mode (ASCII コマンド parser)。`MODE SHELL` 送信前のキー入力は `ERR unknown <cmd>` が返る。
- `MODE SHELL` 受信時に IMU/SENSOR pump は OFF にされる (auto-resume なし)。telemetry を続行したい場合は `IMU ON\n` / `SENSOR ON\n` を再送。
- Ctrl-C / job control なし (FIFO は tty 非対応)。長時間コマンドを止めたい時は RFCOMM 切断 (Terminal 1 の `Ctrl-C`)。
- `btsensor stop` で Hub 側 in-RAM link key DB が消える。次回接続時に SSP authentication failure が出る場合は `bluetoothctl remove ...` + `btsensor unpair` の上で再ペア。

プロトコル仕様・既知制約・トラブルシュートの詳細は [docs/ja/development/bt-nsh-shell.md](docs/ja/development/bt-nsh-shell.md)。SPP/RFCOMM ペアリング全般は [docs/ja/development/pc-receive-spp.md](docs/ja/development/pc-receive-spp.md)。
