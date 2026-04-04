# NuttX 基礎調査

## 1. バージョン選定

### 最新リリース

| バージョン | リリース日 |
|---|---|
| **12.12.0** | 2025-12-31 |
| 12.11.0 | 2025-10-05 |
| 12.10.0 | 2025-07-07 |
| 12.9.0 | 2025-04-14 |
| 12.8.0 | 2025-01-06 |

### STM32F4 サポート状況

STM32F4 は NuttX で成熟したプラットフォーム。以下のボードが公式サポート:

- Nucleo F401RE, F410RB, F411RE, F412ZG, F429ZI, F446RE
- STM32F4-Discovery, STM32F411E-Discovery, STM32F429I-DISCO
- Olimex STM32-E407, H405, H407, P407
- OMNIBUSF4, ODrive V3.6, Axoloti 他

### STM32F413 サポート状況

**STM32F413 はチップレベルで未サポート。** `arch/arm/src/stm32/Kconfig` に `CONFIG_ARCH_CHIP_STM32F413` エントリがない。F41x 系は F410, F411, F412 までで F413 は欠落。

最も近い既存チップは **STM32F412**。F413 は F412 に対して以下が追加:
- UART チャンネル追加 (UART7-10)
- 追加タイマー (LPTIM 含む)
- 追加 SPI/I2C
- DFSDM
- Flash 容量増 (1MB → 1.5MB)

ChibiOS では F412 サポートを clone して F413 を追加した前例がある。

### 決定

**NuttX 12.12.0** を使用する。

- STM32F4 プラットフォームが成熟しており、最新リリースでバグ修正が最も充実
- F413 チップサポートの追加が必要だが、F412 をベースにすれば実現可能
- 初期ブリングアップでは F412 設定をそのまま使い、段階的に F413 固有対応を追加する方針も有効

---

## 2. ビルドシステム

### 2リポジトリ構成

```
workspace/
  nuttx/          # owhinata/nuttx (fork) — カーネル/OS
  nuttx-apps/     # owhinata/nuttx-apps (fork) — アプリケーション (別名: apps)
```

apps リポジトリはデフォルトで `../apps` (nuttx ディレクトリからの相対パス) に配置。`-a <path>` オプションで変更可能。

**nuttx リポジトリ:**
- `arch/` — プロセッサアーキテクチャ・チップ固有コード
- `boards/` — ボード定義 (`boards/<arch>/<chip>/<board>/`)
- `drivers/` — OS デバイスドライバ
- `sched/`, `fs/`, `net/` — スケジューラ, ファイルシステム, ネットワーク
- `tools/` — `configure.sh` 等のビルドツール

**nuttx-apps リポジトリ:**
- `examples/` — サンプルアプリ (hello 等)
- `nshlib/` — NuttShell ライブラリ
- `system/` — システムユーティリティ
- `builtin/` — ビルトインアプリ登録

### Kconfig / defconfig

NuttX は Linux Kconfig システムを使用:

- **Kconfig**: ソースツリー全体に分散。各ディレクトリの設定オプションを定義
- **defconfig**: `boards/<arch>/<chip>/<board>/configs/<config>/defconfig` に配置。デフォルトからの差分のみを含む最小設定ファイル
- **.config**: ビルド時に nuttx ルートに生成される完全な設定ファイル

```
# defconfig の例
CONFIG_ARCH="arm"
CONFIG_ARCH_CHIP="stm32"
CONFIG_ARCH_CHIP_STM32F412ZG=y
CONFIG_ARCH_BOARD="nucleo-f412zg"
```

設定ワークフロー:
1. `make menuconfig` — 対話的メニュー設定
2. `make olddefconfig` — 未設定オプションにデフォルト値を適用
3. `make savedefconfig` — 現在の .config から最小 defconfig を生成

### configure.sh ワークフロー

```bash
cd nuttx
./tools/configure.sh -l <board>:<config>   # -l: Linux, -m: macOS
make -j$(nproc)
```

処理内容:
1. `<board>:<config>` をパース → `boards/*/*/<board>/configs/<config>/` に解決
2. `Make.defs` を検索しコピー
3. `defconfig` を `nuttx/.config` にコピー
4. ホスト OS を .config に設定
5. `make olddefconfig` で完全な .config を展開

### ボード定義に必要なファイル

```
boards/arm/stm32/<board>/
  Kconfig                          # ボード固有 Kconfig
  CMakeLists.txt                   # add_subdirectory(src)
  configs/nsh/defconfig            # NSH ビルド用最小設定
  include/board.h                  # クロック設定, ピン定義, LED/ボタン定義
  scripts/Make.defs                # ツールチェーン, LDSCRIPT, コンパイラフラグ
  scripts/ld.script                # リンカスクリプト (MEMORY, SECTIONS)
  src/Make.defs                    # CSRCS = stm32_boot.c ...
  src/CMakeLists.txt               # CMake ソースリスト
  src/stm32_boot.c                 # stm32_boardinitialize() 実装
  src/stm32_bringup.c              # ペリフェラル初期化
  src/<board>.h                    # ボード内部ヘッダ
```

`boards/Kconfig` への追加エントリも必要:
```kconfig
config ARCH_BOARD_SPIKE_PRIME_HUB
    bool "LEGO SPIKE Prime Hub"
    depends on ARCH_CHIP_STM32F413VG
```

### Out-of-tree カスタムボード

NuttX ソースツリーを変更せずにボードを定義可能:
```
CONFIG_ARCH_BOARD_CUSTOM=y
CONFIG_ARCH_BOARD_CUSTOM_DIR="../custom-boards/spike-prime-hub"
```

```bash
./tools/configure.sh -l ../custom-boards/spike-prime-hub/configs/nsh
```

### ビルドに必要なツール

| ツール | 用途 |
|---|---|
| `arm-none-eabi-gcc` | ARM クロスコンパイラ |
| `python3-kconfiglib` | Kconfig ツール (純 Python 実装、NuttX 公式推奨) |
| `make` (GNU Make) | ビルドシステム |
| `genromfs` | ROMFS イメージ生成 (オプション) |
| `git`, `bison`, `flex`, `gperf`, `xxd` | ビルド依存 |

---

## 3. ライセンス互換性

### 組み合わせ一覧

| 組み合わせ | 互換性 | 備考 |
|---|---|---|
| Apache 2.0 (NuttX) + MIT (本プロジェクト) | 互換 | Apache 2.0 が結合著作物のライセンスとなる |
| Apache 2.0 + BSD 3-Clause (STM32 HAL) | 互換 | 各ファイルは元のライセンスヘッダを保持 |
| MIT + BSD 3-Clause | 互換 | 全て permissive ライセンス |
| pybricks を参照 (コード複製なし) | 問題なし | 学習・参照は義務を生じない |

### 帰属表示要件

| ライセンス | 要件 |
|---|---|
| Apache 2.0 | ライセンスのコピーを含む。NOTICE ファイルを保持。変更点を明示 |
| MIT | 著作権表示とライセンス本文をコピーに含む |
| BSD 3-Clause | 著作権表示・ライセンス本文を配布物に含む。寄稿者名での推奨禁止 |

### 方針

- NuttX 由来コードは Apache 2.0 ヘッダを保持
- 本プロジェクトのオリジナルコードは MIT
- STM32 HAL ファイルは BSD 3-Clause ヘッダを保持
- pybricks は参照のみ (コード複製なし) のため義務なし。謝辞としてドキュメントに記載する
