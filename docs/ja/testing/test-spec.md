# テスト仕様書

## 概要

SPIKE Prime Hub NuttX プロジェクトの自動テスト環境。pexpect + pyserial + pytest を使用し、シリアル経由で NSH コマンドを実行して出力を検証する。

### 前提条件

- SPIKE Prime Hub が USB 接続されている
- NuttX (usbnsh) がフラッシュ済み
- Python 仮想環境 (`.venv/`) が利用可能

### テスト依存のインストール

```bash
.venv/bin/pip install -r requirements.txt
```

## 実行方法

シリアルデバイスは `-D` オプションまたは環境変数 `NUTTX_DEVICE` で指定する。
デフォルトは `/dev/tty.usbmodem01`。

```bash
# 環境変数で設定（推奨: .zshrc 等に追加）
export NUTTX_DEVICE=/dev/tty.usbmodem01

# 自動テストのみ（推奨、普段はこれ）
.venv/bin/pytest tests/ -m "not slow and not interactive"

# 自動 + interactive（目視・操作あり）
.venv/bin/pytest tests/ -m "not slow"

# 全テスト（slow 含む、カーネル CONFIG 変更時）
.venv/bin/pytest tests/

# 特定カテゴリのみ
.venv/bin/pytest tests/test_drivers.py

# 特定テストのみ
.venv/bin/pytest tests/test_drivers.py::test_battery_gauge
```

## テスト一覧

| カテゴリ | 件数 | 完全自動 | 操作待ち | スキップ |
|----------|------|----------|----------|----------|
| A. 起動・初期化 | 4 | 4 | 0 | 0 |
| B. ペリフェラル | 9 | 8 | 1 | 0 |
| C. システム | 6 | 4 | 2 | 0 |
| D. クラッシュ | 4 | 4 | 0 | 0 |
| E. OSテスト | 2 | 1 | 0 | 1 ([#26](https://github.com/owhinata/spike-nx/issues/26)) |
| F. サウンド | 13 | 9 | 4 | 0 |
| G. Flash / LittleFS | 7 | 4 | 3 | 0 |
| H. Bluetooth (CC2564C) | 5 | 4 | 1 | 0 |
| **合計** | **50** | **39** | **11** | **1** |

## A. 起動・初期化 (`test_boot.py`)

### A-1: test_nsh_prompt

- **目的**: NSH プロンプトが応答すること
- **コマンド**: Enter 送信
- **判定**: `nsh> ` が返る

### A-2: test_dmesg_no_error

- **目的**: ブートログにエラーがないこと
- **コマンド**: `dmesg`
- **判定**: 出力に `ERROR` を含む行がない

### A-3: test_procfs_version

- **目的**: NuttX バージョン情報が取得できること
- **コマンド**: `cat /proc/version`
- **判定**: 出力に `NuttX` を含む

### A-4: test_procfs_uptime

- **目的**: アップタイムが取得できること
- **コマンド**: `cat /proc/uptime`
- **判定**: 数値（`\d+\.\d+`）を含む

## B. ペリフェラルドライバ (`test_drivers.py`)

### B-1: test_battery_gauge

- **目的**: バッテリーゲージの電圧読み取り
- **コマンド**: `battery gauge`
- **判定**: `Voltage:` と `mV` を含む

### B-2: test_battery_charger

- **目的**: 充電器の状態読み取り
- **コマンド**: `battery charger`
- **判定**: `State:` と `Health:` を含む

### B-3: test_battery_monitor

- **目的**: バッテリーモニターの連続サンプリング
- **コマンド**: `battery monitor 3`
- **判定**: 3 行以上のデータ行、`FAULT` なし

### B-4: test_imu_accel

- **目的**: 加速度センサーの動作確認
- **コマンド**: `sensortest -n 3 accel0`
- **判定**: `number:3/3` を含む

### B-5: test_imu_gyro

- **目的**: ジャイロスコープの動作確認
- **コマンド**: `sensortest -n 3 gyro0`
- **判定**: `number:3/3` を含む

### B-6: test_imu_fusion

- **目的**: IMU fusion デーモンの起動・データ取得・停止
- **手順**:
    1. CPU 負荷記録（fusion 前）
    2. `imu start` — デーモン起動
    3. 3 秒待機
    4. `imu status` — `running:` + `yes` 確認
    5. `imu accel` — Z 軸の絶対値 8000-12000 mm/s² 確認
    6. `imu gyro` — `gyro:` を含む
    7. `imu upside` — `up side:` を含む
    8. CPU 負荷記録（fusion 中、約 15%）
    9. `imu stop` — デーモン停止（try/finally で確実に停止）
- **判定**: ステータス・加速度 Z 軸・停止成功

### B-7: test_i2c_scan

- **目的**: I2C バス上の IMU 検出
- **コマンド**: `i2c dev -b 2 0x03 0x77`
- **判定**: `6a` を含む（LSM6DS3 アドレス）
- **備考**: `i2c` コマンド未登録時は自動スキップ

### B-8: test_led_smoke

- **目的**: 各 LED グループ (status / battery / bluetooth / matrix) を一瞬ずつ点灯して、LED ドライバ経路と NSH 応答を確認する smoke テスト
- **コマンド**: `led smoke`
- **判定**: 出力に `smoke: status`, `smoke: battery`, `smoke: bluetooth`, `smoke: matrix`, `smoke: done` を含む
- **所要時間**: 約 0.5 秒 (各 LED 100 ms 点灯)

### B-9: test_led_all `@interactive`

- **目的**: LED の全パターン点灯 (RGB サイクル / rainbow / breathe / matrix)
- **コマンド**: `led all`
- **判定**: `All tests done` + 目視確認
- **タイムアウト**: 120 秒（Matrix LED シーケンスを含む）

## C. システムサービス (`test_system.py`)

### C-1: test_watchdog_device

- **目的**: ウォッチドッグデバイスの存在
- **コマンド**: `ls /dev/watchdog0`
- **判定**: `watchdog0` を含む

### C-2: test_cpuload

- **目的**: CPU 負荷情報の取得
- **コマンド**: `cat /proc/cpuload`
- **判定**: `%` を含む

### C-3: test_stackmonitor

- **目的**: スタックモニターデーモンの起動・停止
- **コマンド**: `stackmonitor_start` / `stackmonitor_stop`
- **判定**: コマンドが見つかること、`/proc/0` に `stack` エントリが存在

### C-4: test_help_builtins

- **目的**: 組み込みアプリの登録確認
- **コマンド**: `help`
- **判定**: `battery`, `led`, `imu` を含む

### C-5: test_power_off `@interactive`

- **目的**: 電源オフ → リセット復帰
- **手順**: センターボタン長押し → リセット → シリアル再接続
- **判定**: `nsh> ` プロンプト復帰

### C-6: test_usb_reconnect `@interactive`

- **目的**: USB 抜挿後の復帰
- **手順**: USB 抜き → 再接続 → シリアル再接続
- **判定**: `nsh> ` プロンプト復帰 + `help` 正常動作

## D. クラッシュハンドリング (`test_crash.py`)

各テストはクラッシュ → ウォッチドッグリセット（約 3 秒） → NSH 再接続のサイクル。`_reboot_before_crash` autouse fixture で各テスト前にも明示的にボードをリブートし、常にクリーンなヒープから開始する。ヒープ比較は `@pytest.mark.no_memcheck` で無効化。

### D-1: test_crash_assert

- **コマンド**: `crash assert`
- **判定**: `up_assert` → リセット → `nsh> ` 復帰

### D-2: test_crash_null

- **コマンド**: `crash null`
- **判定**: `Hard Fault` → IWDG リセット → `nsh> ` 復帰
- **備考**: `CONFIG_WATCHDOG_AUTOMONITOR=y` ([#31](https://github.com/owhinata/spike-nx/issues/31)) により IWDG が起動・ソフトキックされるため、ハードフォルトで割込無効化された後も約 3 秒で IWDG リセットが入り復帰する

### D-3: test_crash_divzero

- **コマンド**: `crash divzero`
- **判定**: `Fault` → IWDG リセット → `nsh> ` 復帰

### D-4: test_crash_stackoverflow

- **コマンド**: `crash stackoverflow`
- **判定**: `assert|Fault` → IWDG リセット → `nsh> ` 復帰

## E. OS テスト (`test_ostest.py`)

`test_ostest` は `@slow` マーク付きでカーネル CONFIG 変更時のみ実行。通常は `-m "not slow"` で除外される。`test_coremark` は 12 秒程度で完走するため通常実行に含まれる。

### E-1: test_ostest `@slow` `@skip`

- **コマンド**: `ostest`
- **判定**: `Exiting with status 0`
- **タイムアウト**: 900 秒
- **スキップ理由**: signest_test（nested signal handler test）でハング ([#26](https://github.com/owhinata/spike-nx/issues/26))

### E-2: test_coremark

- **コマンド**: `coremark`
- **判定**: `CoreMark 1.0 :` を含む
- **タイムアウト**: 300 秒
- **実測結果**: 170.82 iterations/sec (STM32F413, Cortex-M4)

## F. サウンドドライバ (`test_sound.py`)

`/dev/tone0`, `/dev/pcm0`, `apps/sound` NSH ビルトインの smoke テスト。自動テスト側は発音を最小限に抑え (short tone + short beep の 2 本のみ)、可聴確認は interactive テストで行う。

### F-1: test_sound_devices_present

- **目的**: 起動時に `/dev/tone0` と `/dev/pcm0` が登録されていること
- **コマンド**: `ls /dev`
- **判定**: `tone0` と `pcm0` を含む

### F-2: test_sound_dmesg_banner

- **目的**: bringup syslog にサウンド初期化ログが 3 行あること
- **コマンド**: `dmesg`
- **判定**: `sound: initialized`, `tone: /dev/tone0 registered`, `pcm: /dev/pcm0 registered` を含む

### F-3: test_sound_usage

- **目的**: `sound` コマンドの usage 表示
- **コマンド**: `sound`
- **判定**: `Usage`, `beep`, `notes` を含む

### F-4: test_tone_single_note

- **目的**: `/dev/tone0` に 4 分音符 1 つを書き込み、期待時間内にコンソールが戻ること (tone 経路の smoke)
- **コマンド**: `echo "C4/4" > /dev/tone0`
- **判定**: 所要時間 400-900 ms (120 BPM quarter = 500 ms + release gap)

### F-5: test_sound_beep_default

- **目的**: `sound beep` のデフォルト (500 Hz / 200 ms) が期待時間内に返ること (PCM 経路の smoke)
- **コマンド**: `sound beep`
- **判定**: 所要時間 150-800 ms

### F-6: test_sound_volume_roundtrip

- **目的**: ボリュームの SET/GET が `TONEIOC_VOLUME_*` 経由で一貫すること (無音)
- **コマンド**: `sound volume` / `sound volume 30` / `sound volume 75` / 復帰
- **判定**: SET した値が GET で読める

### F-7: test_sound_off

- **目的**: `sound off` が何も再生していない状態でも正常終了すること
- **コマンド**: `sound off`
- **判定**: `failed` を含まない

### F-8: test_pcm_short_write_rejected

- **目的**: PCM ヘッダより短い書き込みでドライバが `-EINVAL` を返し、後続コマンドが実行可能なこと (panic 検出)
- **コマンド**: `echo "abcd" > /dev/pcm0` → `echo alive`
- **判定**: 後続コマンドで `alive` が返る

### F-9: test_pcm_bad_magic_rejected

- **目的**: 不正な magic の 20 バイトヘッダを拒否し panic しないこと
- **コマンド**: `echo 'XXXXXXXXXXXXXXXXXXXX' > /dev/pcm0` → `echo alive`
- **判定**: 後続コマンドで `alive` が返る

### F-I1: test_sound_audible_tone_dev `@interactive`

- **目的**: `/dev/tone0` の可聴確認
- **コマンド**: `echo "C4/4 E4/4 G4/4 C5/2" > /dev/tone0`
- **判定**: C/E/G/C の上昇アルペジオが聞こえる

### F-I2: test_sound_audible_pcm_dev `@interactive`

- **目的**: `/dev/pcm0` 経由の可聴確認
- **コマンド**: `sound beep 440 400` → `sound beep 880 400`
- **判定**: 440 Hz と 880 Hz の 2 音が聞こえる

### F-I3: test_sound_audible_notes_app `@interactive`

- **目的**: `apps/sound` の `sound notes` 経路の可聴確認
- **コマンド**: `sound notes "T240 C4/4 E4/4 G4/4 C5/4 G4/4 E4/4 C4/2"`
- **判定**: C メジャーのアップダウンアルペジオが聞こえる

### F-I4: test_sound_audible_volume `@interactive`

- **目的**: ボリューム変更が実際に音量に反映されること
- **コマンド**: volume 100 → beep → volume 20 → beep → 元に戻す
- **判定**: 2 音目が 1 音目より明らかに小さく聞こえる

## G. Flash / LittleFS (`test_flash.py`)

W25Q256 + LittleFS の回帰テストと、picocom + lrzsz 経由の Zmodem 転送 interactive テスト。詳細は [W25Q256 ドライバ](../drivers/w25q256.md) と [Zmodem ファイル転送](../development/file-transfer.md) を参照。

### G-1: test_flash_mount

- **目的**: `/mnt/flash` が LittleFS で起動時にマウント済みであること
- **コマンド**: `mount`
- **判定**: 出力に `/mnt/flash` と `littlefs` を含む

### G-2: test_flash_mtdblock_visible

- **目的**: `/dev/mtdblock0` のみが公開され、full chip raw `/dev/mtd0` は非公開であること
- **コマンド**: `ls /dev/mtdblock0` / `ls /dev/mtd0`
- **判定**: 前者は名前を返す、後者は ENOENT で返る (LEGO 領域保護)

### G-3: test_flash_write_read

- **目的**: ファイル書込・読出のラウンドトリップ
- **コマンド**: `echo "regression-<ts>" > /mnt/flash/regress.txt` → `cat /mnt/flash/regress.txt`
- **判定**: 書き込んだ payload が cat で取得できる
- **タイムアウト**: 120 秒 (LittleFS 初回 metadata 更新は数秒かかることがある)

### G-4: test_flash_persist_across_reboot

- **目的**: NSH `reboot` 後にファイル内容が残ること (LittleFS 永続性)
- **コマンド**: `echo "persist-<ts>" > /mnt/flash/persist.txt` → `reboot` → `cat /mnt/flash/persist.txt`
- **判定**: reboot 後にファイル内容が一致する

### G-I1: test_flash_zmodem_send `@interactive`

- **目的**: PC → Hub の Zmodem アップロード + Hub 側 md5 検証
- **手順**:
    1. pytest がランダム 1 MB ファイルを `/tmp/spike_zmtest_1mb.bin` に生成 (毎回新規)
    2. シリアルポートを解放、operator に `picocom --send-cmd 'sz -vv -L 256' /dev/tty.usbmodem01` 実行を促す
    3. operator が picocom 内で `nsh> rz` → `C-a C-s` → ファイルパス入力で送信
    4. operator が picocom 終了後 Enter → pytest がシリアル再接続
    5. ファイルサイズ (1048576 byte) と Hub 側 `md5 -f /mnt/flash/spike_zmtest_1mb.bin` が PC 側 md5 と一致することを検証
- **依存**: lrzsz (`brew install lrzsz`)

### G-I2: test_flash_zmodem_recv `@interactive`

- **目的**: Hub → PC の Zmodem ダウンロード + PC 側 md5 ラウンドトリップ検証
- **前提**: `test_flash_zmodem_send` が同セッション内で実行済み (Hub に対象ファイルが存在)
- **手順**:
    1. pytest が Hub 側 `md5 -f` で ground-truth md5 を取得
    2. 既存の同名ローカルファイルを削除し、シリアルポートを解放
    3. operator に `cd <pytest cwd> && picocom --receive-cmd 'rz -vv -y' /dev/tty.usbmodem01` 実行を促す
    4. operator が picocom 内で `nsh> sz /mnt/flash/spike_zmtest_1mb.bin` → `C-a C-r` → 引数なし Enter で受信
    5. operator が picocom 終了後 Enter → pytest がシリアル再接続
    6. PC 側に保存されたファイルの md5sum が Hub 側 md5 と一致することを検証
    7. 検証成功時はローカルおよび Hub 側のテストファイルを削除
- **依存**: lrzsz (`brew install lrzsz`)

### G-S1: test_flash_stress_concurrent `@interactive`

- **目的**: Sound DMA1_S5 + IMU I2C2 daemon + Flash SPI2/DMA1_S3/S4 + USB CDC を同時稼働させて、W25Q256 driver が他ペリフェラルを乱さないことを確認 (Codex 必須ゲート)
- **手順**:
    1. `imu start` で IMU daemon 起動 (継続的な I2C2 読出 + EXTI4 + HPWORK 負荷)
    2. `dmesg` を baseline として記録
    3. `sound volume 15` で控えめな音量に下げて `sound beep 440 4000 &` をバックグラウンド再生 (4 秒, 440 Hz)
    4. `dd if=/dev/zero of=/mnt/flash/stress.bin bs=1024 count=64` で 64 KB 書込 (16 × 4 KB sector erase + 256 page program ≒ 3-5 秒、Sound 再生と完全に重なる)
    5. `dd if=/mnt/flash/stress.bin of=/dev/null bs=1024 count=64` で読み戻し
    6. 2 秒待って tail-end の PCM DMA を completion
    7. `ls -l /mnt/flash/stress.bin` でサイズ 65536 byte 確認
    8. `imu status` で IMU daemon 生存確認 (`running: yes`)
    9. `dmesg` 差分に `ERROR / FAULT / underrun / overflow / ASSERT / panic` の語が含まれていないことを assert
    10. operator が音にクリック・ドロップアウト・ピッチ揺れがなかったか judge (DAC ドライバが underrun を syslog しないため可聴チェックは耳だけが頼り)
    11. cleanup: `imu stop`, `rm /mnt/flash/stress.bin`, 元の音量に復元

### 実行コマンド例

```bash
# 回帰テストのみ (interactive を除外)
.venv/bin/pytest tests/test_flash.py -m "not interactive" -D /dev/tty.usbmodem01

# 送信のみ
.venv/bin/pytest tests/test_flash.py::test_flash_zmodem_send -D /dev/tty.usbmodem01

# ストレステストのみ
.venv/bin/pytest tests/test_flash.py::test_flash_stress_concurrent -D /dev/tty.usbmodem01

# 全テスト (interactive 含む)
.venv/bin/pytest tests/test_flash.py -D /dev/tty.usbmodem01
```

## H. Bluetooth (`test_bt_spp.py`)

Issue #52 で NuttX 標準 BT スタックを撤去し btstack + Classic BT SPP 構成に切り替えたため、テスト群もそれに合わせて全面書き直し。Hub 側の chardev・bring-up・SPP サービス起動の健全性のみを自動化し、PC 受信の pair + RFCOMM 接続は手動手順 (`docs/development/pc-receive-spp.md`) として残す。

### H-1: test_bt_chardev_exists

- **目的**: カーネル側 chardev が登録されていること
- **コマンド**: `ls /dev/ttyBT`
- **判定**: `/dev/ttyBT` を含み、`No such file` を含まない

### H-2: test_bt_powered_banner

- **目的**: CC2564C への給電+slow clock 起動が完了していること
- **コマンド**: `dmesg`
- **判定**: `BT: CC2564C powered, /dev/ttyBT ready` を含む

### H-3: test_bt_btsensor_builtin

- **目的**: btsensor NSH builtin が登録されていること
- **コマンド**: `help | grep btsensor`
- **判定**: 出力に `btsensor` を含む

### H-4: test_bt_btsensor_hci_working

- **目的**: `btsensor &` で btstack が HCI_STATE_WORKING に到達し、正当な BD アドレスが得られること
- **コマンド**: `btsensor &`
- **判定**: `btsensor: HCI working, BD_ADDR <xx:xx:xx:xx:xx:xx>` を expect し、BD アドレスがオール 0 / オール F でない
- **副作用**: btstack が起動しっぱなしになる (NSH を `reboot` するか電源 off しない限り停止しない)

### H-5: test_bt_pc_pair_and_stream `@interactive`

- **目的**: Linux または macOS の PC から SPP 接続 + フレーム受信までの手動検証
- **前提**: Hub が H-4 まで合格している、PC (Linux or macOS) と blueutil/bluetoothctl が利用可能
- **手順**: `docs/development/pc-receive-spp.md` の pair → RFCOMM open → magic `5a a5` 確認
- **判定**: operator が目視で magic 付きストリームを確認

### 実行コマンド例

```bash
# 自動テストのみ (interactive 除外)
.venv/bin/pytest tests/test_bt_spp.py -m "not interactive" -D /dev/tty.usbmodem01

# PC 対話テストのみ
.venv/bin/pytest tests/test_bt_spp.py::test_bt_pc_pair_and_stream -D /dev/tty.usbmodem01
```

## テスト同期方式 (sendCommand)

`tests/conftest.py` の `NuttxSerial.sendCommand()` はユニークな per-call センチネルでシリアル出力の同期を取る。

1. Phase 1 — PRE マーカー: `echo MKPRE<nonce>` を送信し、`<pre>\r\nnsh> ` パターンを expect してクリーンなベースラインを確立する。stale buffer を吸収する役割。
2. Phase 2 — 本コマンド: `sendline(cmd)` の後、行頭アンカー付きプロンプト `\r\nnsh> ` を expect して出力を取得する。`nsh> ` 単体マッチではなく `\r\n` をアンカーにすることで、コマンド出力中の文字列誤マッチを防ぐ。

この方式により、以前の「先頭単語 echo を expect する」戦略で発生していた不安定マッチ (issue [#33](https://github.com/owhinata/spike-nx/issues/33)) を解消している。重いコマンドの途中で 2 本目の sendline が drop されないよう、各 phase の sendline は 1 本のみ。

## メモリリーク検出

各テストの前後で `free` コマンドを実行し、ヒープメモリの空き容量を比較する。テスト後に 1KB 以上の減少がある場合は警告を出力する。

個別テストで free 測定を無効化したい場合は `@pytest.mark.no_memcheck` マーカーを付ける (`conftest.py` の `check_memory_leak` fixture が skip する)。

!!! note
    `test_power_off` 等リセットを伴うテストでは、リセット前後のヒープ初期状態の差分で警告が出ることがあるが、メモリリークではない。
