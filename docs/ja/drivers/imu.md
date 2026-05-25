# IMU ドライバ設計

## 1. 概要

SPIKE Prime Hub 搭載の LSM6DS3TR-C (6 軸 IMU) を NuttX の uORB センサードライバで制御する。LSM6DSL と完全なレジスタ互換があり、NuttX の LSM6DSL uORB ドライバをそのまま使用する。

IMU 処理ライブラリ (`apps/imu/`) はセンサー非依存で、uORB トピック経由でデータを消費する。

## 2. デバイス仕様

LSM6DS3TR-C は 3 軸加速度計と 3 軸ジャイロスコープを統合したシングルダイ 6 軸 IMU。

| 項目 | 値 |
|---|---|
| I2C アドレス | 0x6A (SDO/SA0 = GND) |
| WHO_AM_I | 0x6A |
| 起動時 ODR | 833 Hz (ioctl で変更可) |
| 起動時 加速度 FSR | ±2 g (ioctl で 2/4/8/16 g に変更可) — Phase 2.5 (#145) で ±8 g から narrowing、tilt 分解能 4× 向上 |
| 起動時 ジャイロ FSR | ±1000 dps (ioctl で 125/250/500/1000/2000 dps に変更可) — Phase 2.5 (#145) で ±2000 dps から narrowing、角速度分解能 2× 向上 (drivebase 最大 565 dps に対し 1.8× マージン) |

## 3. ボード配線

| ピン | 機能 | 説明 |
|---|---|---|
| PB10 | I2C2_SCL | I2C クロック (AF4) |
| PB3 | I2C2_SDA | I2C データ (AF9, F413 固有) |
| PB4 | INT1 | ジャイロ DRDY 割り込み (EXTI4) |

## 4. レジスタ設定

| レジスタ | 設定 | 目的 |
|---|---|---|
| CTRL1_XL (0x10) | ODR=833Hz, FS=±2g | 加速度: ODR とフルスケール (Phase 2.5 default) |
| CTRL2_G (0x11) | ODR=833Hz, FS=±1000dps | ジャイロ: ODR とフルスケール (Phase 2.5 default) |
| CTRL3_C (0x12) | BDU=1, IF_INC=1 | データ更新ブロック、アドレス自動インクリメント |
| CTRL5_C (0x14) | ROUNDING=011 | バースト読み出し用ラウンディング |
| DRDY_PULSE_CFG (0x0B) | DRDY_PULSED=1 | パルスモード DRDY |
| INT1_CTRL (0x0D) | INT1_DRDY_G=1 | ジャイロ DRDY を INT1 にルーティング |

## 5. データ取得

INT1 (gyro DRDY) 割り込みで 12 バイト一括読み出し:

- バイト 0-5: ジャイロ X/Y/Z (OUTX_L_G 0x22 〜 OUTZ_H_G 0x27)
- バイト 6-11: 加速度 X/Y/Z (OUTX_L_A 0x28 〜 OUTZ_H_A 0x2D)

両センサーが同一 ODR (833Hz) で動作するため、gyro DRDY 時に accel データも更新済み。

### データフロー

```
INT1 (gyro DRDY) 発火
  → ISR: timestamp (CLOCK_BOOTTIME us, low 32bit) を捕捉
  → HPWORK: 0x22 から 12 バイト一括読み出し
  → 16 サンプル毎に OUT_TEMP_L/H (0x20) も読み出し（間は前回値を使い回し）
  → struct sensor_imu (timestamp + accel/gyro 生 int16 + temperature_raw)
  → push_event で /dev/uorb/sensor_imu0 に配信
```

物理量変換はドライバでは行わず、消費側で FSR を見て LSB を換算する。

## 6. NuttX ドライバアーキテクチャ

### ボード層

```
boards/spike-prime-hub/src/lsm6dsl_uorb.c        - I2C 制御、バースト読み出し、uORB 登録
boards/spike-prime-hub/src/lsm6dsl_uorb.h        - config 構造体、登録 API
boards/spike-prime-hub/src/stm32_lsm6dsl.c       - I2C2 初期化、INT1 割り込み設定、ドライバ登録
boards/spike-prime-hub/include/board_lsm6dsl.h   - ボードローカル ioctl (FSR 変更)
```

### uORB トピック

```
/dev/uorb/sensor_imu0   (struct sensor_imu)
```

`sensor_custom_register()` で登録した SENSOR_TYPE_CUSTOM トピック。1 read で accel + gyro + temperature_raw + ISR timestamp を 1 構造体で取得できる。

### struct sensor_imu

`boards/spike-prime-hub/include/board_lsm6dsl.h` で定義 (24 B 固定、`_Static_assert` で size / offset を強制):

| オフセット | フィールド | 型 | 内容 |
|---|---|---|---|
| +0 | timestamp | uint32_t | CLOCK_BOOTTIME us の low 32bit (~71m35s で wrap)。ARMv7-M の 4-byte aligned word は single-copy atomic ⇒ ISR と worker 間で tearing 無し |
| +4 | ax / ay / az | int16_t | 加速度 生 LSB、Hub body frame (chip frame の Y/Z をドライバで反転) |
| +10 | gx / gy / gz | int16_t | ジャイロ 生 LSB、Hub body frame (chip frame の Y/Z をドライバで反転) |
| +16 | temperature_raw | int16_t | OUT_TEMP 生値（16 サンプルごとに更新、間は前回値 stale） |
| +18 | odr_idx | uint8_t | `enum lsm6dsl_odr_e` 値 (0..0xA) — そのサンプル取得時の HW ODR を埋め込み |
| +19 | fsr_xl_idx | uint8_t | `enum lsm6dsl_fsr_xl_e` 値 (sparse: 0=2g, 1=16g, 2=4g, 3=8g) |
| +20 | fsr_gy_idx | uint8_t | `enum lsm6dsl_fsr_gy_e` 値 (sparse: 0=250, 1=125, 2=500, 4=1000, 6=2000 dps) |
| +21..+23 | reserved[3] | uint8_t × 3 | パディング (`push_data()` で `memset` ゼロ化) |

Issue #139 で sensor_imu に per-sample `odr_idx` / `fsr_xl_idx` / `fsr_gy_idx` を埋め込むことで、active 中の live ODR/FSR 変更があっても consumer (drivebase / imu daemon / btsensor BUNDLE) が各サンプルの正しい物理単位を後追いで計算できるようになっている。

### ioctl

| ioctl | 引数 | 動作 |
|---|---|---|
| `SNIOC_SET_INTERVAL` | uint32 (period_us) | ODR を period_us 以上の最近接 ODR に設定 |
| `SNIOC_SETSAMPLERATE` | uint32 (Hz: 13/26/52/104/208/416/833/1660/3330/6660) | ODR を Hz で指定 |
| `LSM6DSL_IOC_SETACCELFSR` | uint32 (g: 2/4/8/16) | 加速度 FSR を変更 |
| `LSM6DSL_IOC_SETGYROFSR` | uint32 (dps: 125/250/500/1000/2000) | ジャイロ FSR を変更 |
| `SNIOC_GETSAMPLERATE` | uint32* (out) | 現在の ODR を `enum lsm6dsl_odr_e` 値 (idx) で返す |
| `LSM6DSL_IOC_GETACCELFSR` | uint32* (out) | 現在の加速度 FSR を `enum lsm6dsl_fsr_xl_e` 値 (idx) で返す |
| `LSM6DSL_IOC_GETGYROFSR` | uint32* (out) | 現在のジャイロ FSR を `enum lsm6dsl_fsr_gy_e` 値 (idx) で返す |

SET ioctl は物理値 (Hz / g / dps) を受け取って driver-internal な enum idx に変換する。GET ioctl は **driver-internal な enum idx をそのまま返す** (sample 埋め込み idx と対称、consumer 側に `idx → 物理値` lookup table を持たせる設計)。

Issue #139 以前は active 中 (`SNIOC_ACTIVATE(true)` 後) の SET は `-EBUSY` で拒否していたが、現在は **active 中でも live SET を受理する**。SET ハンドラは `devlock` 配下で `push_data()` と直列化されているため、register R-M-W と publish path のレースは発生しない。SET 直後 1 サンプルだけ chip-internal pipeline の影響で「旧 register 値で取得されて新 idx でタグ付けされる」過渡があり得るが、consumer (drivebase / imu daemon) は idx 変化検出時に自動 recalibrate する仕組みで吸収される。

### defconfig

```
CONFIG_STM32_I2C2=y
CONFIG_I2C=y
CONFIG_I2C_RESET=y
CONFIG_SCHED_HPWORK=y
CONFIG_SENSORS=y
CONFIG_APP_IMU=y
```

### 起動時の I2C バスリカバリ

WDOG / assert / crash で MCU が soft-reset した直後、LSM6DS3TR-C が前回トランザクションの途中で残り SDA を low に握ったまま wedge することがある。SPIKE Prime Hub には IMU 専用の電源制御 GPIO が無く (LSM6DSL VDD/VDDIO は常時給電)、ハードウェア電源リセットは不可能なため、`stm32_lsm6dsl.c` で起動時に毎回 I2C2 のバスリカバリを実行している。

シーケンス (1 attempt 分):

1. `stm32_i2cbus_initialize(2)` で I2C2 ハンドル取得
2. `I2C_RESET(i2c)` (NuttX `stm32_i2c_reset`) で SCL/SDA を一時 GPIO open-drain 化 → SCL を最大 10 clock toggle (SDA HIGH まで) → START + STOP で slave state machine リセット → I2C ペリフェラル再 init
   - 失敗時 (SDA が解放されない / clock stretch が無限) は bus が deinit + GPIO 化のまま残るため、`stm32_i2cbus_uninitialize` → `stm32_i2cbus_initialize` で確実に復元する
3. `lsm6dsl_register_uorb()` を呼び出し
   - `lsm6dsl_hw_init()` 冒頭で WHO_AM_I (`0x0F` の値が `0x6A`) を pre-flight check し、不一致なら早期に `-ENODEV` を返してリトライへ早期遷移
   - SW_RESET 後の poll は 50 回 (50 ms) 上限、完了後 10 ms の保守的 settle delay
4. 失敗したら 10 ms sleep して 1. に戻る (最大 3 attempts)

成功時は `IMU: LSM6DS3TR-C registered on I2C2 addr=0x6a (attempt N)` が syslog に出る (`N` は 0 オリジン、通常 0)。リトライ中の失敗は `snwarn`、3 attempts 全て失敗で `snerr` ログを残して諦め、`/dev/uorb/sensor_imu0` は登録されない。実機運用上、3 attempt 内で復旧しない場合は battery 完全 OFF が必要 (chip 内部 state も腐るケース)。

参考: pybricks も同等の bus recovery を I2C2 init 時に毎回実行している (`HAL_I2C_MspInit()`、SCL 10 clock toggle)。NuttX 標準 `stm32_i2c_reset()` は open-drain 化と START/STOP まで自動で行うため、pybricks より丁寧な実装。

## 7. センサーフュージョン

pybricks から移植したクォータニオンベースの相補フィルタを使用。

### 初期化

最初の重力ベクトル測定からクォータニオンを生成。

### サンプルごとの処理

1. **キャリブレーション適用**: 加速度のオフセット+スケール補正、ジャイロのバイアス+スケール補正
2. **重力推定**: 回転行列の第 3 行から推定重力ベクトルを抽出
3. **誤差信号**: 推定重力と測定重力の外積を計算
4. **静止度**: ブレンディング係数 (0..1) を計算
5. **融合角速度**: `omega_fused = omega_gyro + correction * fusion_gain`
6. **クォータニオン積分**: 前進オイラー法 + 正規化

### ヘディング

- **1D ヘディング**: 軸ごとの回転積分
- **3D ヘディング**: x 軸の水平面への atan2 射影

## 8. キャリブレーション

Phase 2.5 (Issue #145 / #146) で導入された 2 段構成:

- **オフライン (Tedaldi)** — `imu_tk` (Pretto 2014) で約 5 分の捕捉セッションから bias / scale / misalignment を一括推定。結果を `/mnt/flash/imu_cal.txt` にプロパティ形式で保存。
- **オンライン (EMA)** — `drivebase_imu` がブート後の温度ドリフトを α=0.1 の指数平均で追従。オフライン値をシードとして使う。

両者は協調する: オフラインで個体の根本的な bias / scale / 軸非直交を取り除き、オンラインで残った温度依存ドリフトを補正する。

### キャリブレーションファイル形式

`/mnt/flash/imu_cal.txt` (schema_version=1, properties 形式):

```
schema_version = 1
nominal_gyro_radps_per_lsb = 6.108652e-04
nominal_accel_ms2_per_lsb  = 5.985504e-04
fsr_gy_dps = 1000
fsr_xl_g = 2
odr_hz = 107
ambient_temp_c = 28.1

gyro_bias_lsb_x1000  = 22406 65567 11412
accel_bias_lsb_x1000 = -390870 12782 80597
gyro_M_x1000  = 997 -1 18 5 988 -3 -1 -10 1002
accel_M_x1000 = 1008 -2 9 0 1002 3 0 0 1002
```

- `*_bias_lsb_x1000`: 3 軸 bias (raw LSB × 1000)
- `*_M_x1000`: 3×3 行列 (misalignment × scale を統合、row-major、要素 × 1000)
- 適用式 (LSB ドメイン): `corrected[i] = sum_j(M[i][j] × (raw[j] − bias[j])) / 1000`
- nominal 感度で割って M_runtime を出すと M 対角は ≈ 1000 (= 個体差 1% 以内なら nominal 通り)

### ホスト側 pipeline (cal 生成)

詳細は `tools/imu_cal/README.md` を参照。3 ステップ:

1. **キャプチャ** — ImuViewer の "IMU Capture (Tedaldi)" expander で 5 分の静止+回転セッションを 27 B/sample frame として `.bin` に記録 (frame_type 0x03 / 104 Hz / FSR ±2g/±1000dps)。Tedaldi は **最初の 10 秒の完全静止** を noise floor 採取窓に使うので、Start 直後は触らないこと。約 12 ポーズ以上 × 各 5 秒以上の静止 + 2〜3 秒の手回し回転を挟む。
2. **Tedaldi 実行** — `tools/imu_cal/run_imu_tk.sh <session_dir>` で Docker (`ghcr.io/owhinata/ubuntu-imu_tk`) を呼び bias / scale / misalignment を最適化。`.calib` ファイル 2 つを生成。
3. **cfg 生成** — `tools/imu_cal/imu_tk_output_to_cfg.py --session-dir <session_dir>` で `imu_cal.txt` を出力。M 対角が 1000 から ±5% を超えると warning。

### Hub へのデプロイ

```text
nsh> rz                                  # zmodem 受信開始
(host) sx -k imu_cal.txt > /dev/ttyACM0
nsh> mv received_file /mnt/flash/imu_cal.txt
nsh> reboot
```

起動後 `dmesg | grep imu_cal` で読込み確認:

```text
drivebase: imu_cal: loaded /mnt/flash/imu_cal.txt (FSR=±1000 dps, ODR=104 Hz, T=23°C)
```

### Hub 側適用

`drivebase_imu` が `integrate()` 内で各サンプルに matmul + bias 減算を適用 (LSB ドメイン)。x1000 固定小数点で整数演算のみ。EMA は cal-loaded bias をシードに静止区間で更新。`drivebase _imu show` で cal + runtime 双方を確認可。

### ImuViewer 側適用 (Issue #146)

ホストからも同じ cal を Telemetry stream (frame_type 0x02) に適用可能:

1. ImuViewer の **Telemetry** expander を開く
2. **Apply offline calibration** チェックボックス ON
3. **Browse...** で `imu_cal.txt` を選択
4. status 行に `loaded · FSR ±1000dps/±2g · ODR 107Hz · T 28.1°C` 表示

`SensorAggregator.OnBundle()` で FSR スケーリング前に matmul を適用する Hub と同じ数式。Madgwick filter にも cal-corrected 値が入る。BUNDLE ヘッダの FSR が cal と不一致なら warning + cal 無効化。

### 検証 (acceptance)

| verb | 用途 | 受入基準 |
|---|---|---|
| `drivebase _imu show` | 現在の cal + runtime bias / temperature 表示 | `cal.loaded=1`, M 対角 ≈ 1000 |
| `drivebase _imu drift <sec>` | 静止時 drift rate 測定 | `drift_mdegpm` < 500 (実機 Tedaldi で 3 mdeg/min 達成) |
| `drivebase _imu verify <deg>` | 任意角回転と gyro 積分結果の対照 | Phase 2.5 までは raw-Z 積分なので `target × cos(tilt)`。Phase 3a (Section 13) で Madgwick による world-vertical yaw 抽出に切替え後は `actual_mdeg ≈ target_deg × 1000 ± 4°` (取付け角に依存しない) |

## 9. 静止判定

### アルゴリズム

1. 125 サンプルの遅い移動平均をベースラインとして計算
2. 各サンプルをジャイロ・加速度閾値と比較
3. 約 1 秒間連続して静止と判定された場合:
   - バイアス推定コールバックを呼び出し
   - 実際のサンプル時間を測定

## 10. 軸符号補正

Hub PCB の実装方向に合わせて軸符号を補正:

| 軸 | 符号 |
|---|---|
| X | -1 |
| Y | +1 |
| Z | -1 |

!!! note
    現時点では軸符号補正は未適用。生データがそのまま出力される。

## 11. IMU 処理ライブラリ (apps/imu/)

uORB データを消費するセンサー非依存の処理層。

| ファイル | 説明 |
|---|---|
| `imu_main.c` | NSH コマンド `imu` + デーモン |
| `imu_fusion.c/h` | センサーフュージョン (相補フィルタ) |
| `imu_stationary.c/h` | 静止判定 |
| `imu_calibration.c/h` | キャリブレーション永続化 |
| `imu_geometry.c/h` | クォータニオン、ベクトル、行列演算 |
| `imu_types.h` | 型定義 |

### NSH コマンド

```
nsh> imu start    # デーモン起動
nsh> imu accel    # 加速度表示
nsh> imu gyro     # 角速度表示
nsh> imu status   # ステータス表示
nsh> imu stop     # デーモン停止
```

## 12. pybricks 機能対応表

| pybricks 機能 | NuttX 実装 | 備考 |
|---|---|---|
| `imu.up()` | `imu_fusion` 重力ベクトル分類 | 6 面検出 |
| `imu.tilt()` | `imu_fusion` 重力ベクトルから導出 | ピッチ/ロール |
| `imu.acceleration()` | uORB `sensor_imu0` から ax/ay/az 読み出し → FSR ベースで mm/s² 換算 | apps/imu 側で変換 |
| `imu.angular_velocity()` | uORB `sensor_imu0` から gx/gy/gz 読み出し → FSR ベースで deg/s 換算 | apps/imu 側で変換 |
| `imu.heading()` | `imu_fusion` 3D ヘディング | z 軸回転 |
| `imu.rotation()` | `imu_fusion` 1D ヘディング | 軸ごと |
| `imu.orientation()` | `imu_fusion` クォータニオン→オイラー角 | ヨー/ピッチ/ロール |
| `imu.settings()` | `imu_calibration` + NSH コマンド | 閾値設定 |
| `imu.stationary()` | `imu_stationary` 状態照会 | bool |
| `imu.reset_heading()` | `imu_fusion` ヘディングリセット | 積分値ゼロ化 |

## 13. Drivebase IMU Madgwick fusion (Phase 3a, Issue #147)

`apps/drivebase/drivebase_imu` は daemon が任意で gyro-locked heading source として使うユーザ空間 IMU 統合レイヤ。Phase 3a で、従来の素の `∫gz_corr dt` heading を Madgwick 6-DOF IMU-only fusion に差し替え、Hub の取付けが傾いていても heading が `cos(tilt)` だけ目減りしない設計にする。

### なぜ raw 積分でなく Madgwick か

Phase 2.5 verify を ~51° 傾いたベンチで実施した結果、物理 360° 旋回に対し 228.7° しか積分されなかった (= 360 × cos(51°) ≈ 226.5°)。素の Z 軸 gyro 積分は IMU の物理 Z 軸まわりの回転を見ているだけで、ロボット本体の実回転軸まわりではない。LSM6DSL を取り付けた向きが垂直からずれていれば射影誤差は構造的に発生する。Madgwick は accel (重力) を quaternion 推定にフュージョンし、その後 world-vertical yaw を抽出するので、PID が本当に欲しいロボット heading が直接得られる。

### Per-sample パイプライン (LSM6DSL ODR 833 Hz)

`apps/drivebase/drivebase_imu.c::integrate()`:

1. **FSR 遷移ガード** (Issue #139) — driver が live FSR 切替したときに gyro bias / idle threshold の物理意味を保つよう rescale
2. **Gyro Tedaldi matmul** — `g_corr_x1000[i] = Σⱼ M[i][j] × (g_raw[j] × 1000 − bias[j]) / 1000` (Phase 2.5、x1000 fixed-point)
3. `gz_corr` を idle EMA に流し込み温度ドリフトを追跡
4. **Accel FSR 整合チェック** — `fsr_xl_idx_to_g(batch[i].fsr_xl_idx)` と `cal.fsr_xl_g` を比較。不一致なら identity 補正に fallback + `accel_fsr_match=0` を `_imu show` で診断可能に。Madgwick には scale が違うが向きの正しい重力ベクトルが入り、filter 側の re-normalise で yaw drift は構造的に有界
5. **Accel Tedaldi matmul** (一致時) / **identity** (不一致時)
6. **float 変換**:
   - `ω_f[rad/s] = corr_x1000 × (gyro_mdps_num/1000) × π/180 / 1e6`
   - `a_f[g] = corr_x1000 × (fsr_xl_g / 32768) / 1000`
7. **Stationary-gated β** (pybricks `pbio/src/imu.c:327` 形式、rad/s+g 単位に換算):
   ```
   stationary = min(1, ACCL_MIN_G/max(|‖a‖-1|, ACCL_MIN_G))
              × min(1, GYRO_MIN_RADPS/max(‖ω‖, GYRO_MIN_RADPS))
   β_eff = 0.05 × stationary
   ```
   静止時は β=0.05 で full accel 補正、車輪インパクト/急回転で accel error が増えると β→0 になり vibration が accel 項経由で yaw を引きずらない
8. **Quaternion bootstrap** — 初回サンプルで accel の正規化ベクトルから (0,0,1) への最短弧 quaternion を seed。tilt 推定が即収束する (β-blend で数秒待たない)
9. **Madgwick update** — host `ImuViewer.Core/Filters/MadgwickFilter.cs` の C 直訳。β=0.05 で Hub/host parity を担保

### Per-drain yaw 抽出

batch 処理後 1 回:

```
ψ_curr = atan2f(2(q0 q3 + q1 q2), 1 - 2(q2² + q3²))
dψ     = wrap_pi(ψ_curr - ψ_prev)
heading_mdeg += dψ × 180000/π
ψ_prev = ψ_curr
```

`atan2f` は per-drain (~500 Hz) で per-sample (833 Hz) ではない。最悪 2000 dps × 2 ms RT tick = 4° で ±π wrap には届かないので unwrap 精度は確保される。

### FPU と CPU 見積もり

- `CONFIG_ARCH_FPU=y` + `CONFIG_LIBM_NEWLIB=y` が必須 (`drivebase_imu.c` 冒頭で `#error` ガード)
- NuttX の lazy FPU は OFF (`arch/arm/src/armv7-m/arm_fpuconfig.c:63`) で context switch 時に S16–S31 が既に保存対象。Phase 3a では新たな switch コストは発生しない
- 833 Hz 時の見積もり: Madgwick ~0.2 % CPU、`atan2f` per-drain ~0.05 %
- `integrate()` は task context (RT tick work-queue / `_imu` CLI) からのみ呼び出し可。ISR からは禁止 (libm 経路の cycle が乗らないため)

### Timestamp wrap 対応

`sensor_imu.timestamp` は `CLOCK_BOOTTIME` µs の下位 32 ビット (~71 分 wrap)。Phase 3a で旧来の `last_sample_ts_us != 0 && ts > last_sample_ts_us` ガードを `last_sample_valid` フラグ + `(uint32_t)(ts - last)` のモジュラ減算に置換、wrap 越えでも負の dt にならない。

### 診断

`drivebase _imu show` の Phase 3a 追加出力:

```
madgwick.q_x1000=<w> <x> <y> <z>    # quaternion × 1000 (起動直後 ≈ 1000 0 0 0)
madgwick.tilt_mdeg=<value>           # world vertical からの傾き (mdeg)
madgwick.beta_x1000=50               # base fusion gain × 1000 (default)
madgwick.initialized=1               # quaternion seed 済み
madgwick.accel_fsr_match=1           # 1 = accel Tedaldi cal をこのサンプルで
                                     #     適用 (cal loaded かつ live FSR が
                                     #     cal.fsr_xl_g と一致)
                                     # 0 = identity fallback (cal 未 load も
                                     #     しくは live FSR が cal.fsr_xl_g と
                                     #     乖離)
```

`drivebase _imu verify <deg>` (Section 8 acceptance) は world-vertical yaw を返すので、51° 傾いたベンチで物理 360° 旋回 → ≈ 360 000 mdeg。

### Stale 検出 API

`bool db_imu_is_stale(im, now_us, threshold_us)` は `threshold_us` (既定 `DB_IMU_DEFAULT_STALE_THRESHOLD_US = 50 ms`、25 RT tick) 以内にサンプル取得した drain がなければ true を返す。Phase 3b の heading PID 注入で encoder fallback ガードとして使い、Issue #102 の I²C 復旧中などで stale サンプルを積分しないようにする。

## 14. Drivebase heading PID への IMU 注入 (Phase 3b, Issue [#148](https://github.com/owhinata/spike-nx/issues/148))

Phase 3a で `db_imu_get_heading_mdeg()` は world-vertical robot heading を返すようになり、`drivebase _imu show` の手回し 360° は ±0.003° まで一致した。Phase 3b ではこれを drivebase aggregate heading PID の **state 入力** に注入する。encoder 由来 `(sR-sL)/2` のままだとホイールスリップ・バックラッシュ・モーター個体差を closed-loop で吸収できないため。

この world-vertical heading は body forward 軸を world 水平面へ射影したもの (= §7 の **3D ヘディング**)。Issue #157 で `set-gyro 3d` を唯一の gyro mode とし、同じ射影を計算していた旧 `1d` 名は廃止した。`set-gyro 1d` は `-EINVAL` で拒否される。

### 構造

| 概念 | データソース | 場所 |
|---|---|---|
| `use_gyro_requested` | user-requested (`set-gyro` ioctl / cfg `use_gyro_plus1`) | `db_drivebase_s.use_gyro_requested` |
| `use_gyro_latched` | motion 開始時に capturable なら requested を写す | `db_drivebase_s.use_gyro_latched` |
| `gyro_origin_mdeg` | `set_origin` / `reset` / `set-gyro` / latch で `raw - angle_mdeg` を保存 | `db_drivebase_s.gyro_origin_mdeg` |
| `gyro_origin_valid` | origin が信頼できる baseline か | `db_drivebase_s.gyro_origin_valid` |

`db_drivebase_capture_start_heading(db, state, now_us, do_latch)` (`drivebase_drivebase.c`) が 4 entry point (`setup_position_move` / `drive_curve` / `drive_forever` / `db_drivebase_stop`) と per-tick の両方から呼ばれ、do_latch=true なら latched + origin in-place snapshot を取り、do_latch=false でも latched が 3D かつ guard 一式 OK なら `state->heading_x_mdeg` を `(raw - gyro_origin_mdeg)` から逆変換して注入する。

### 構造的 race 防止

- ioctl は `requested` だけ更新、`latched` には触れない。走行中の `set-gyro` は `-EBUSY` で拒否
- 走行中の IMU 一時 stale は per-tick guard が encoder fallback に切替えるだけで latched は変えない。次 tick で復帰したら gyro 注入に戻る。soft race だがモーター制御は止まらない
- in-flight retarget (`drive_forever` 再 arm) では origin in-place snapshot を `!gyro_origin_valid` のみに限定し、既存 origin を上書きしない (publish basis が走行中にジャンプしない)

### RT tick 順序

```
chardev → db_imu_drain_and_update → db_drivebase_update → publish overwrite
```

Phase 3a の元の順序 (drivebase_update → IMU drain) では heading PID が前 tick の IMU sample を見ていた。Phase 3b で IMU drain を前に出すことで同 tick の sample が PID に届く。

### Publish overwrite

`drivebase_daemon.c` の publish overwrite が `latched == 3D` (走行中) または `requested == 3D && origin_valid` (未走行で `set-gyro` 直後) なら `st.angle_mdeg = (int32_t) CLAMP(raw - gyro_origin_mdeg, INT32_MIN, INT32_MAX)` で IMU 由来値に差し替え。PID 入力と user-visible publish が同 origin を引くため SSOT 化される。

### Out of scope (別 Issue)

- X/Y bias EMA 温度ドリフト追従 — Phase 3a Codex CONCERN 2 領分
- 1D mode (`set-gyro 1d`) — Issue #157 で廃止。`3d` (fused forward-axis 射影) が唯一の gyro heading mode
- 走行中 IMU stale 復帰時の I-term spike 評価 — bench で stale を起こさない acceptance 前提
