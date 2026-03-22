# ビルドシステム統合

## 1. ツールチェーン要件

### 既存 Docker イメージのパッケージ

現在の `docker/Dockerfile` (Ubuntu 24.04) に含まれるもの:
- `build-essential`, `gcc-arm-none-eabi`, `libnewlib-arm-none-eabi`
- `git`, `python3`, `python3-pip`, `python3-venv`, `pipx`, `unzip`, `zip`
- pip: `pycryptodomex`
- pipx: `poetry`

### NuttX ビルドに追加で必要なパッケージ

```bash
apt-get install -y --no-install-recommends \
    bison flex gettext texinfo \
    libncurses5-dev libncursesw5-dev xxd \
    gperf automake libtool pkg-config genromfs \
    libgmp-dev libmpc-dev libmpfr-dev libisl-dev \
    python3-kconfiglib \
    u-boot-tools util-linux
```

| パッケージ | 用途 |
|---|---|
| `python3-kconfiglib` | Kconfig ツール (menuconfig 等)。NuttX 公式推奨の純 Python 実装 |
| `genromfs` | ROMFS イメージ生成 (オプション) |
| `bison`, `flex` | パーサー生成 |
| `gperf` | ハッシュ関数生成 |
| `xxd` | バイナリ→Cヘッダ変換 |
| その他 | ビルド依存 |

> **注**: C ベースの `kconfig-frontends` は新しい NuttX Kconfig で構文エラーが発生する場合があるため使用しない ([issue #2405](https://github.com/apache/nuttx/issues/2405))。

### DFU フラッシュ用

```bash
apt-get install -y dfu-util
```

---

## 2. ソース統合戦略

### 決定: git submodule

pybricks と同様の submodule パターンを採用:

```
spike-nx/
├── nuttx/              # git submodule: owhinata/nuttx (f413-support ブランチ)
├── nuttx-apps/         # git submodule: owhinata/nuttx-apps (タグ nuttx-12.12.0)
├── boards/             # カスタムボード定義 (out-of-tree)
│   └── spike-prime-hub/
│       ├── configs/nsh/defconfig
│       ├── include/board.h
│       ├── scripts/
│       │   ├── Make.defs
│       │   └── ld.script
│       ├── src/
│       │   ├── Make.defs
│       │   ├── stm32_boot.c
│       │   ├── stm32_bringup.c
│       │   └── spike_prime_hub.h
│       └── Kconfig
├── docker/
├── scripts/
└── pybricks/
```

### 理由

- NuttX カーネルコードの変更が不要 (out-of-tree カスタムボード)
- upstream の更新をタグ切り替えで取り込み可能
- フォークはメンテナンスコストが高い
- ただし F413 チップサポート追加が必要な場合はフォークが必要になる可能性あり

### Out-of-tree カスタムボード設定

defconfig に以下を設定:

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

### configure コマンド

```bash
cd nuttx
./tools/configure.sh -l ../boards/spike-prime-hub/configs/nsh
```

### 注意事項

- `make distclean` は `.config` を削除するため、再度 `configure.sh` の実行が必要
- apps ディレクトリのデフォルトは `../apps` だが、submodule 名が `nuttx-apps` なので `CONFIG_APPS_DIR` で明示的に指定
- F413 チップサポートが NuttX 本体に無いため、初期段階では F412 設定で代用するか、NuttX のフォークが必要

---

## 3. ビルドスクリプト設計

### `scripts/nuttx.mk`

`scripts/pybricks.mk` のパターンを踏襲:

```makefile
DOCKER_IMAGE := nuttx-builder
DOCKER_FILE  := docker/Dockerfile.nuttx
NUTTX_DIR    := nuttx
APPS_DIR     := nuttx-apps
BOARD_DIR    := boards/spike-prime-hub
BOARD_CONFIG := nsh

# Docker run コマンド (UID/GID マッピング付き)
DOCKER_RUN := docker run --rm \
    -v $(CURDIR):$(CURDIR) \
    -w $(CURDIR) \
    --user $(shell id -u):$(shell id -g) \
    -v /etc/passwd:/etc/passwd:ro \
    -v /etc/group:/etc/group:ro \
    $(DOCKER_IMAGE)
```

### ターゲット

| ターゲット | 動作 |
|---|---|
| `build` (デフォルト) | submodule init → Docker image build → configure → make |
| `configure` | `tools/configure.sh` 実行 |
| `clean` | `make clean` (ビルド成果物のみ削除) |
| `distclean` | `make distclean` + Docker image 削除 + submodule deinit |
| `flash` | `dfu-util` でファームウェア書込み (ホスト側で実行) |
| `menuconfig` | `make menuconfig` (対話的設定) |

### ビルドフロー

```
make -f scripts/nuttx.mk
  1. git submodule update --init nuttx nuttx-apps (未初期化の場合)
  2. docker build (イメージ未作成の場合)
  3. docker run: cd nuttx && ./tools/configure.sh -l ../boards/spike-prime-hub/configs/nsh
  4. docker run: cd nuttx && make -j$(nproc)
  5. 成果物: nuttx/nuttx.bin
```

---

## 4. DFU バイナリ生成

### 方法 1: raw バイナリ直接フラッシュ (推奨)

DFU ファイル変換不要。`nuttx.bin` を直接アドレス指定でフラッシュ:

```bash
dfu-util -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

| オプション | 意味 |
|---|---|
| `-a 0` | alternate interface 0 (内蔵フラッシュ) |
| `-s 0x08008000:leave` | 開始アドレス + DFU モード終了 |
| `-D nuttx.bin` | ダウンロードするバイナリ |

開発中はこの方法が最もシンプル。

### 方法 2: DfuSe 形式 .dfu ファイル作成

`dfuse-pack.py` (dfu-util ソースリポジトリに付属) を使用:

```bash
python3 dfuse-pack.py -b 0x08008000:nuttx.bin firmware.dfu
dfu-util -a 0 -D firmware.dfu
```

`.dfu` ファイルにはターゲットアドレスが埋め込まれるため、フラッシュ時に `-s` 不要。

### NuttX バイナリ生成

NuttX ビルドは以下を生成:
- `nuttx` — ELF ファイル (デバッグ情報付き)
- `nuttx.bin` — raw バイナリ (`arm-none-eabi-objcopy -O binary` で生成)

`nuttx.bin` がフラッシュ対象。

### VID/PID 指定

SPIKE Prime Hub の DFU は LEGO 固有の VID/PID を使用:
```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx.bin
```

### フラッシュスクリプト (make flash)

```bash
# DFU モード進入手順:
# 1. USB ケーブルを抜く
# 2. Bluetooth ボタンを 5 秒間長押し
# 3. ボタンを押したまま USB ケーブルを接続
# 4. ボタンを離す
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```
