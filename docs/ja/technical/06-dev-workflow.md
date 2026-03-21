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

## 2. SWD デバッグ可否

### 制約

SPIKE Prime Hub では SWD デバッグピンが電源制御に転用されている:

| ピン | SWD 機能 | Hub での用途 |
|---|---|---|
| PA13 | SWDIO | BAT_PWR_EN (バッテリー電源維持) |
| PA14 | SWCLK | PORT_3V3_EN (I/O ポート 3.3V 電源) |

**PA13 を GPIO 出力に設定した時点で SWD 接続は切断される。**

### 可能性

1. **USB 電源での動作時**: PA13 を HIGH にしなくても電源は維持される (USB バスパワー)。デバッグビルドでは PA13 の再設定を遅延させ、SWD 接続を維持できる可能性がある
2. **ハードウェア改造**: Hub 基板上に SWD テストパッドがあれば、外部プローブ (J-Link, ST-Link) を接続可能。ただし Hub の分解が必要
3. **現実的には困難**: 通常の開発では SWD は使用不可と想定する

### 代替デバッグ手段

| 方法 | 段階 | 詳細 |
|---|---|---|
| I/O ポート UART | 初期ブリングアップ | 6 ポートのいずれかに USB-UART アダプタを接続。最もシンプルなデバッグ手段 |
| USB CDC/ACM | USB ドライバ動作後 | NSH コンソール。追加ハードウェア不要 |
| LED 表示 | 常時 | TLC5955 ドライバ動作前でも GPIO 直接制御で一部 LED を点灯可能か要確認 |
| ハードフォルト出力 | 常時 | NuttX のハードフォルトハンドラがレジスタダンプを UART/USB に出力 |

---

## 3. シリアルコンソール戦略

### フェーズ 1: 初期ブリングアップ (USB 未動作)

**I/O ポート UART を使用。**

SPIKE Hub の I/O ポートはそれぞれ UART を持つ。USB-UART アダプタ (3.3V) を接続:

| ポート | UART | TX ピン | RX ピン | 推奨理由 |
|---|---|---|---|---|
| A | UART7 | PE8 | PE7 | NuttX で UART7 対応済み |
| B | UART4 | PD1 | PD0 | NuttX で UART4 対応済み |

NuttX defconfig でコンソール UART を設定:
```
CONFIG_STM32_UART7=y
CONFIG_UART7_SERIALDRIVER=y
CONFIG_UART7_SERIAL_CONSOLE=y
CONFIG_UART7_BAUD=115200
CONFIG_UART7_BITS=8
CONFIG_UART7_PARITY=0
CONFIG_UART7_2STOP=0
```

**注意**: I/O ポートコネクタの TX/RX ピン配置は pybricks のピンマップを参照。LEGO 独自コネクタのため、適切なブレークアウトケーブルが必要。

### フェーズ 2: USB 動作後

**USB CDC/ACM に切り替え。**

```
CONFIG_STM32_OTGFS=y
CONFIG_USBDEV=y
CONFIG_CDCACM=y
CONFIG_CDCACM_CONSOLE=y
CONFIG_NSH_USBCONSOLE=y
```

ホスト PC で `/dev/ttyACM0` (Linux) または `/dev/cu.usbmodemXXXX` (macOS) として認識。

---

## 4. 最小ブリングアップ順序

### ステップ 1: クロック設定 + 電源維持

**目標**: Hub が電源 ON を維持し、ハードフォルトしない

- 16 MHz HSE → 96 MHz SYSCLK の PLL 設定
- BAT_PWR_EN (PA13) を HIGH に設定
- 成功判定: Hub がシャットダウンせず電源 ON を維持

### ステップ 2: UART 出力

**目標**: シリアル出力でデバッグ可能にする

- I/O ポートの UART (UART7 推奨) を有効化
- `stm32_lowsetup()` で早期 UART 初期化
- 成功判定: USB-UART アダプタ経由でブートメッセージが表示される

### ステップ 3: NSH シェル (UART)

**目標**: NuttX OS が動作し、NSH コマンド実行可能

- NuttX カーネル起動 → NSH シェル
- UART コンソールでコマンド入力/出力
- 成功判定: `nsh>` プロンプトが表示され、`help` コマンドが動作

### ステップ 4: USB CDC/ACM

**目標**: USB 経由でコンソール接続

- USB OTG FS ドライバ有効化
- CDC/ACM デバイスクラス設定
- 成功判定: ホスト PC で `/dev/ttyACM0` として認識、NSH 操作可能

### ステップ 5: GPIO / LED

**目標**: 視覚的フィードバック

- ステータス LED の GPIO 制御
- TLC5955 ドライバは後回し。まずは GPIO で制御可能な LED があるか確認
- 成功判定: LED の点灯/消灯が制御可能

### ステップ 6: 追加ペリフェラル

以下を順次有効化:
1. SPI (W25Q256 フラッシュ → ファイルシステム)
2. I2C (LSM6DS3TR-C IMU)
3. ADC (バッテリー監視)
4. PWM (モーター制御)
5. SPI (TLC5955 LED マトリクス)
6. 追加 UART (I/O ポート通信)

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
