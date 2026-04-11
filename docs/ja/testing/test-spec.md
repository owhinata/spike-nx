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
| B. ペリフェラル | 8 | 7 | 1 | 0 |
| C. システム | 6 | 4 | 2 | 0 |
| D. クラッシュ | 4 | 1 | 0 | 3 ([#25](https://github.com/owhinata/spike-nx/issues/25)) |
| E. OSテスト | 2 | 1 | 0 | 1 ([#26](https://github.com/owhinata/spike-nx/issues/26)) |
| **合計** | **24** | **17** | **3** | **4** |

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

### B-8: test_led_all `@interactive`

- **目的**: LED の全パターン点灯
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

各テストはクラッシュ → ウォッチドッグリセット（約 3 秒） → NSH 再接続のサイクル。

### D-1: test_crash_assert

- **コマンド**: `crash assert`
- **判定**: `up_assert` → リセット → `nsh> ` 復帰

### D-2: test_crash_null `@skip`

- **コマンド**: `crash null`
- **判定**: `Hard Fault` → リセット → `nsh> ` 復帰
- **スキップ理由**: ハードフォルト時にウォッチドッグリセットが効かず実機ハング ([#25](https://github.com/owhinata/spike-nx/issues/25))

### D-3: test_crash_divzero `@skip`

- **コマンド**: `crash divzero`
- **判定**: `Fault` → リセット → `nsh> ` 復帰
- **スキップ理由**: 同上 ([#25](https://github.com/owhinata/spike-nx/issues/25))

### D-4: test_crash_stackoverflow `@skip`

- **コマンド**: `crash stackoverflow`
- **判定**: `assert|Fault` → リセット → `nsh> ` 復帰
- **スキップ理由**: 同上 ([#25](https://github.com/owhinata/spike-nx/issues/25))

## E. OS テスト (`test_ostest.py`) `@slow`

カーネル CONFIG 変更時のみ実行。通常は `-m "not slow"` で除外。

### E-1: test_ostest `@skip`

- **コマンド**: `ostest`
- **判定**: `Exiting with status 0`
- **タイムアウト**: 900 秒
- **スキップ理由**: signest_test（nested signal handler test）でハング ([#26](https://github.com/owhinata/spike-nx/issues/26))

### E-2: test_coremark

- **コマンド**: `coremark`
- **判定**: `CoreMark 1.0 :` を含む
- **タイムアウト**: 300 秒
- **実測結果**: 170.82 iterations/sec (STM32F413, Cortex-M4)

## メモリリーク検出

各テストの前後で `free` コマンドを実行し、ヒープメモリの空き容量を比較する。テスト後に 1KB 以上の減少がある場合は警告を出力する。

!!! note
    `test_power_off` 等リセットを伴うテストでは、リセット前後のヒープ初期状態の差分で警告が出ることがあるが、メモリリークではない。
