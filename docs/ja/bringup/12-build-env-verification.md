# ビルド環境事前確認

## 1. kconfig-frontends

### Ubuntu 24.04 (Noble) での利用可能性

| パッケージ | バージョン | リポジトリ | 状態 |
|---|---|---|---|
| `kconfig-frontends-nox` | 4.11.0.1+dfsg-6build2 | universe | 利用可能 |
| `python3-kconfiglib` | 14.1.0-3 | universe | 利用可能 |

### 推奨: `python3-kconfiglib`

NuttX の公式ドキュメントと CI は kconfiglib (純 Python 実装) を推奨。C ベースの `kconfig-frontends` は新しい NuttX Kconfig ファイルで構文エラーが発生する場合がある ([issue #2405](https://github.com/apache/nuttx/issues/2405))。

```bash
apt-get install -y python3-kconfiglib
# または: pip3 install kconfiglib
```

---

## 2. NuttX 12.12.0 タグ

### 確認結果

| リポジトリ | タグ | 存在 |
|---|---|---|
| `apache/nuttx` | `nuttx-12.12.0` | 確認済み |
| `apache/nuttx-apps` | `nuttx-12.12.0` | 確認済み |

**注意**: GitHub Release オブジェクトではなくタグとして存在。`git clone --branch nuttx-12.12.0` で取得可能。

---

## 3. Out-of-tree ボード + 未登録チップ

### F413 チップ未登録時の制約

`configure.sh` は defconfig 内の `CONFIG_ARCH_CHIP` が Kconfig に存在する有効な値であることを要求する。`CONFIG_ARCH_CHIP_STM32F413VG` は存在しないため、そのままでは使用不可。

### F412ZG での代用 (初期ブリングアップ)

```
CONFIG_ARCH_CHIP_STM32F412ZG=y
```

| 項目 | F412ZG (NuttX 認識) | F413VG (実際) | 影響 |
|---|---|---|---|
| Flash | 1 MB | 1.5 MB | 追加 512KB 未使用 (問題なし) |
| SRAM | 256 KB | 320 KB | 追加 64KB 未使用 (問題なし) |
| UART | 1-8 | 1-10 | UART9/10 使用不可 |
| DFSDM | なし | あり | 未使用 (問題なし) |

**基本動作** (NSH + UART7 コンソール + USB CDC/ACM) は F412 設定で動作する見込み。UART9/10 (ポート E, F) は F413 フォーク完成後に有効化。

---

## 4. macOS での dfu-util

### Homebrew パッケージ

```bash
brew install dfu-util
```

- バージョン: 0.11 (stable)
- 依存: libusb
- インストール先: `/opt/homebrew/Cellar/dfu-util/0.11`

macOS で問題なく利用可能。

---

## 5. Dockerfile 更新案

既存の `docker/Dockerfile` (pybricks 用) とは別に `docker/Dockerfile.nuttx` を作成:

```dockerfile
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    git \
    python3 \
    python3-pip \
    python3-kconfiglib \
    bison flex gettext texinfo \
    libncurses5-dev libncursesw5-dev xxd \
    gperf automake libtool pkg-config \
    genromfs \
    u-boot-tools util-linux \
    dfu-util \
    && rm -rf /var/lib/apt/lists/*
```
