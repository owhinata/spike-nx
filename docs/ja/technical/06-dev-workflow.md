# 開発ワークフロー

## 1. DFU フラッシュ手順

### DFU モード進入

1. USB ケーブルを抜く
2. Hub 中央の Bluetooth ボタンを 5 秒間長押し
3. ボタンを押したまま USB ケーブルを接続
4. ステータス LED が点滅 → DFU モード
5. ボタンを離す

### デバイス確認

```bash
dfu-util -l
# Found DFU: [0694:0008] ... name="Internal Flash"
```

### ファームウェア書込み

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

| オプション | 意味 |
|---|---|
| `-d 0694:0008` | LEGO VID/PID 指定 |
| `-a 0` | Alternate interface 0 (内蔵フラッシュ) |
| `-s 0x08008000:leave` | 書込み開始アドレス + DFU 終了 |
| `-D nuttx.bin` | ダウンロードするバイナリ |

`:leave` を指定すると、書込み完了後に自動的にファームウェアを起動する。

### ワンコマンド化

```bash
make -f scripts/nuttx.mk flash
```

---

## 2. デバッグ方針

### SWD

SWD ピン (PA13/PA14) が電源制御に転用されており、ピンを外部に引き出すのが困難なため、**SWD は使用しない前提**とする。

| ピン | SWD 機能 | Hub での用途 |
|---|---|---|
| PA13 | SWDIO | BAT_PWR_EN (バッテリー電源維持) |
| PA14 | SWCLK | PORT_3V3_EN (I/O ポート 3.3V 電源) |

### デバッグ手段

| 手段 | 用途 | 詳細 |
|---|---|---|
| **USB CDC/ACM NSH** | 日常開発の主要手段 | NSH コンソール + syslog + `dmesg` |
| **RAMLOG + dmesg** | USB 切断時のログ保持 | RAM リングバッファ。再接続後に確認 |
| **coredump** | クラッシュ後分析 | バックアップ SRAM に永続化 → GDB で分析 |
| **NSH コマンド** | ランタイム監視 | `ps`, `free`, `top`, `/proc` |

詳細は [13-debugging-strategy.md](13-debugging-strategy.md) を参照。

---

## 3. コンソール

**USB CDC/ACM 固定。** I/O ポートはデバッグ用に確保しない (全ポートを Powered Up デバイス接続用に使用)。

```
CONFIG_STM32_OTGFS=y
CONFIG_USBDEV=y
CONFIG_CDCACM=y
CONFIG_CDCACM_CONSOLE=y
CONFIG_NSH_USBCONSOLE=y
CONFIG_NSH_USBCONDEV="/dev/ttyACM0"
```

ホスト PC で `/dev/ttyACM0` (Linux) または `/dev/cu.usbmodemXXXX` (macOS) として認識。

### 初期ブリングアップ時の注意

USB CDC/ACM ドライバが動作するまでシリアル出力が得られない。初期段階で USB が動作しない場合の対処:

1. **RAMLOG**: ブートメッセージが RAM に蓄積される。USB 動作後に `dmesg` で確認
2. **LED**: TLC5955 ドライバ実装後、ブート進捗を LED で表示
3. **やむを得ない場合**: 一時的に I/O ポート UART (UART7 等) をデバッグ出力に使用

---

## 4. 最小ブリングアップ順序

### ステップ 1: クロック設定 + 電源維持

**目標**: Hub が電源 ON を維持し、ハードフォルトしない

- 16 MHz HSE → 96 MHz SYSCLK の PLL 設定
- BAT_PWR_EN (PA13) を HIGH に設定
- 成功判定: Hub がシャットダウンせず電源 ON を維持

### ステップ 2: USB CDC/ACM

**目標**: コンソール接続を確立

- USB OTG FS ドライバ有効化
- CDC/ACM デバイスクラス設定
- 成功判定: ホスト PC で `/dev/ttyACM0` として認識

### ステップ 3: NSH シェル

**目標**: NuttX OS が動作し、NSH コマンド実行可能

- NuttX カーネル起動 → NSH シェル
- USB CDC/ACM コンソールでコマンド入力/出力
- 成功判定: `nsh>` プロンプトが表示され、`help` コマンドが動作

### ステップ 4: TLC5955 LED

**目標**: 視覚的フィードバック

- SPI1 + TLC5955 ドライバ実装
- ステータス LED / 5×5 マトリクス制御
- 成功判定: LED の点灯/消灯が制御可能
- 注: 全 LED は TLC5955 経由。GPIO 直接制御可能な LED は存在しない

### ステップ 5: 追加ペリフェラル

以下を順次有効化:
1. SPI2 (W25Q256 フラッシュ → ファイルシステム)
2. I2C2 (LSM6DS3TR-C IMU)
3. ADC1 (バッテリー監視)
4. PWM (モーター制御)
5. UART (I/O ポート Powered Up デバイス通信)

---

## 5. 開発サイクル

### 標準サイクル

```
コード編集
  ↓
make -f scripts/nuttx.mk          # Docker 内ビルド (~30秒)
  ↓
DFU モード進入                     # ボタン操作 (~5秒)
  ↓
make -f scripts/nuttx.mk flash    # DFU フラッシュ (~10秒)
  ↓
シリアルモニタで動作確認            # screen /dev/ttyACM0 115200
```

推定サイクルタイム: 約 1 分

### ビルドのみ (フラッシュなし)

```bash
make -f scripts/nuttx.mk build
```

### 設定変更

```bash
make -f scripts/nuttx.mk menuconfig   # 対話的設定
make -f scripts/nuttx.mk savedefconfig # defconfig 更新
```

### クリーンビルド

```bash
make -f scripts/nuttx.mk clean        # ビルド成果物のみ
make -f scripts/nuttx.mk distclean    # 完全クリーン
```
