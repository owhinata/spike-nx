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
| 起動時 加速度 FSR | ±8 g (ioctl で 2/4/8/16 g に変更可) |
| 起動時 ジャイロ FSR | 2000 dps (ioctl で 125/250/500/1000/2000 dps に変更可) |

## 3. ボード配線

| ピン | 機能 | 説明 |
|---|---|---|
| PB10 | I2C2_SCL | I2C クロック (AF4) |
| PB3 | I2C2_SDA | I2C データ (AF9, F413 固有) |
| PB4 | INT1 | ジャイロ DRDY 割り込み (EXTI4) |

## 4. レジスタ設定

| レジスタ | 設定 | 目的 |
|---|---|---|
| CTRL1_XL (0x10) | ODR=833Hz, FS=±8g | 加速度: ODR とフルスケール |
| CTRL2_G (0x11) | ODR=833Hz, FS=2000dps | ジャイロ: ODR とフルスケール |
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

`nuttx/include/nuttx/uorb.h` で定義:

| フィールド | 型 | 内容 |
|---|---|---|
| timestamp | uint32_t | CLOCK_BOOTTIME us の low 32bit (~71m35s で wrap)。ARMv7-M の 4-byte aligned word は single-copy atomic ⇒ ISR と worker 間で tearing 無し |
| ax / ay / az | int16_t | 加速度 生 LSB、Hub body frame (chip frame の Y/Z をドライバで反転) |
| gx / gy / gz | int16_t | ジャイロ 生 LSB、Hub body frame (chip frame の Y/Z をドライバで反転) |
| temperature_raw | int16_t | OUT_TEMP 生値（16 サンプルごとに更新、間は前回値 stale） |
| reserved | int16_t | アラインメント用パディング |

### ioctl

| ioctl | 引数 | 動作 |
|---|---|---|
| `SNIOC_SET_INTERVAL` | uint32 (period_us) | ODR を period_us 以上の最近接 ODR に設定 |
| `SNIOC_SETSAMPLERATE` | uint32 (Hz: 13/26/52/104/208/416/833/1660/3330/6660) | ODR を Hz で指定 |
| `LSM6DSL_IOC_SETACCELFSR` | uint32 (g: 2/4/8/16) | 加速度 FSR を変更 |
| `LSM6DSL_IOC_SETGYROFSR` | uint32 (dps: 125/250/500/1000/2000) | ジャイロ FSR を変更 |

サンプリング有効中（`SNIOC_ACTIVATE` 後）に上記 ioctl を呼ぶと `-EBUSY` を返す。設定変更したい場合は先に deactivate してから ioctl すること。

### defconfig

```
CONFIG_STM32_I2C2=y
CONFIG_I2C=y
CONFIG_SCHED_HPWORK=y
CONFIG_SENSORS=y
CONFIG_APP_IMU=y
```

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

### ジャイロバイアス

静止状態時の指数平滑法で推定。

### 加速度キャリブレーション

6 姿勢重力測定 (各軸の正負方向) でオフセットとスケールを算出。

### ジャイロスケール

軸ごとの補正係数。1 回転がおよそ 360 度になるよう調整。

### 保存形式

設定は `/data/imu_cal.bin` にバイナリファイルとして永続化。

### デフォルト値

| パラメータ | デフォルト値 |
|---|---|
| 重力 | ±9806.65 mm/s² |
| スケール | 360 |
| ジャイロ閾値 | 2 deg/s |
| 加速度閾値 | 2500 mm/s² |

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
