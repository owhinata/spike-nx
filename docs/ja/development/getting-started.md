# Getting Started

SPIKE Prime Hub 向け NuttX ファームウェアのビルド・書込み・接続手順。

## ビルド

Docker コンテナがすべてを処理する (submodule init, Docker image build, configure, make)。

```bash
make
```

成果物は `nuttx/nuttx.bin`。

### Kconfig 設定

```bash
# 対話的メニュー設定
make nuttx-menuconfig

# 現在の .config から最小 defconfig を保存
make nuttx-savedefconfig
```

### クリーン

```bash
# ビルド成果物のみ削除 (.config は残る)
make nuttx-clean

# .config も含めて完全クリーン
make nuttx-distclean

# Docker image 削除 + submodule deinit
make distclean
```

## DFU フラッシュ

### DFU モードへの進入

1. USB ケーブルを抜く
2. Bluetooth ボタンを 5 秒間長押し
3. ボタンを押したまま USB ケーブルを接続
4. ボタンを離す

### 書込みコマンド

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

| オプション | 意味 |
|---|---|
| `-d 0694:0008` | SPIKE Prime Hub の VID/PID |
| `-a 0` | alternate interface 0 (内蔵フラッシュ) |
| `-s 0x08008000:leave` | 開始アドレス + DFU モード終了 |
| `-D nuttx/nuttx.bin` | ダウンロードするバイナリ |

macOS では `brew install dfu-util` でインストール。

## シリアル接続

```bash
picocom /dev/tty.usbmodem01
```

USB CDC/ACM 経由で NSH コンソールに接続する。

## Out-of-tree ボード定義

NuttX ソースツリーを変更せずにボードを定義する。ボード定義は `boards/spike-prime-hub/` に配置:

```
boards/spike-prime-hub/
├── configs/nsh/defconfig     # NSH ビルド用最小設定
├── include/board.h           # クロック設定, ピン定義
├── scripts/
│   ├── Make.defs             # ツールチェーン, LDSCRIPT, コンパイラフラグ
│   └── ld.script             # リンカスクリプト (MEMORY, SECTIONS)
├── src/
│   ├── Make.defs             # CSRCS = stm32_boot.c ...
│   ├── stm32_boot.c          # stm32_boardinitialize() 実装
│   ├── stm32_bringup.c       # ペリフェラル初期化
│   └── spike_prime_hub.h     # ボード内部ヘッダ
└── Kconfig                   # ボード固有 Kconfig
```

defconfig でカスタムボードを指定:

```
CONFIG_ARCH="arm"
CONFIG_ARCH_ARM=y
CONFIG_ARCH_CHIP="stm32"
CONFIG_ARCH_CHIP_STM32F413VG=y
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR_RELPATH=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../boards/spike-prime-hub"
CONFIG_ARCH_BOARD_CUSTOM_NAME="spike-prime-hub"
CONFIG_APPS_DIR="../nuttx-apps"
```

## Docker イメージ

`docker/Dockerfile.nuttx` で定義。Ubuntu 24.04 ベース:

| パッケージ | 用途 |
|---|---|
| `gcc-arm-none-eabi`, `libnewlib-arm-none-eabi` | ARM クロスコンパイラ |
| `python3-kconfiglib` | Kconfig ツール (NuttX 公式推奨の純 Python 実装) |
| `bison`, `flex`, `gperf`, `xxd` | ビルド依存 |
| `genromfs` | ROMFS イメージ生成 |
| `dfu-util` | DFU フラッシュ |

> **注意**: C ベースの `kconfig-frontends` は新しい NuttX Kconfig で構文エラーが発生するため使用しない。
