# デバッグ戦略

## 1. 概要

SPIKE Prime Hub では SWD ピン (PA13/PA14) が電源制御に転用されており、ピンを外部に引き出すのが困難なため SWD デバッグは使用しない。USB CDC/ACM 経由の NSH コンソールを主要デバッグ手段とする。

### デバッグ手段の優先順位

| 優先度 | 手段 | 用途 |
|---|---|---|
| **Primary** | USB CDC/ACM NSH + syslog + `dmesg` | 日常開発 |
| **Secondary** | NuttX coredump → バックアップ SRAM 永続化 | クラッシュ後分析 |
| **Diagnostic** | NSH コマンド (`ps`, `free`, `top`, `/proc`) | ランタイム監視 |
| **Last resort** | I/O ポート UART を一時的にデバッグ出力に使用 | USB 未動作時の初期ブリングアップ |

### 使用しない手段

- **SWD**: PA13/PA14 のピン引出しが困難なため不使用
- **GDB**: 基本的に不使用。やむを得ない場合は I/O ポート UART 経由で接続する可能性あり

---

## 2. NuttX ログシステム

### 階層的デバッグアーキテクチャ

NuttX のデバッグ出力は syslog ベース。2 次元の階層構造:

**優先度レベル** (個別に設定可能):

| 設定 | レベル | 説明 |
|---|---|---|
| `CONFIG_DEBUG_ERROR` | Error | 最重要 |
| `CONFIG_DEBUG_WARN` | Warning | 警告 |
| `CONFIG_DEBUG_INFO` | Info | 詳細 (最も冗長) |

**サブシステム** (各レベルの `_ERROR`, `_WARN`, `_INFO` バリアント):

| 設定 | 対象 |
|---|---|
| `CONFIG_DEBUG_USB` | USB スタック |
| `CONFIG_DEBUG_SCHED` | スケジューラ |
| `CONFIG_DEBUG_MM` | メモリ管理 |
| `CONFIG_DEBUG_GPIO` | GPIO |
| `CONFIG_DEBUG_I2C` | I2C バス |
| `CONFIG_DEBUG_SPI` | SPI バス |
| `CONFIG_DEBUG_NET` | ネットワーク |
| `CONFIG_DEBUG_FS` | ファイルシステム |

**マスタースイッチ**: `CONFIG_DEBUG_FEATURES=y` で全サブシステムオプションが有効化。

### syslog 出力先

| 設定 | 出力先 | 用途 |
|---|---|---|
| `CONFIG_SYSLOG_CHARDEV` | キャラクタデバイス | USB CDC/ACM (メイン出力先) |
| `CONFIG_RAMLOG` | RAM リングバッファ | `dmesg` で読み出し。USB 切断時も保持 |
| `CONFIG_SYSLOG_FILE` | ファイル | マウント済みファイルシステムへ |

**推奨**: `CONFIG_RAMLOG` を必ず有効化。USB CDC/ACM がダウンした場合でもログを保持し、再接続後に `dmesg` で確認可能。特にブート初期段階のメッセージは USB ドライバ動作前に出力されるため、RAMLOG でのみ確認できる。

---

## 3. クラッシュダンプ / ハードフォルト

### ハードフォルトハンドラ

NuttX は Cortex-M のハードフォルト時に例外フレームをスタックから取得し、レジスタダンプを出力。

**Kconfig**:
```
CONFIG_DEBUG_HARDFAULTS=y        # 詳細な障害レジスタ出力 (CFSR, HFSR, MMFAR, BFAR)
CONFIG_STACK_COLORATION=y        # 全タスクのスタック使用量をダンプ
CONFIG_DEBUG_ASSERTIONS=y        # ASSERT/PANIC パスでレジスタダンプ
```

**注意**: NSH コンソールが USB CDC/ACM の場合、ハードフォルト時に USB ドライバ自体が動作不能の可能性がある。RAMLOG にクラッシュ情報が書き込まれていれば、DFU でファームウェアを再書込み後に `dmesg` で確認できる (バックアップ SRAM 永続化が実装されている場合)。

### coredump (ポストモーテム分析)

NuttX は coredump サブシステムを提供:

```
CONFIG_COREDUMP=y                      # マスター有効化
CONFIG_BOARD_COREDUMP_SYSLOG=y         # syslog にダンプ
CONFIG_BOARD_COREDUMP_COMPRESSION=y    # LZF 圧縮 (デフォルト有効)
CONFIG_BOARD_COREDUMP_FULL=y           # 全タスク情報を保存
```

出力は hex エンコード + LZF 圧縮。`tools/coredump.py` で ELF core ファイルに変換 → GDB で分析。

### クラッシュデータの永続化

**課題**: クラッシュダンプはシリアル/syslog に出力され、リセット時に失われる。

**SPIKE Hub 向け推奨**:
- STM32F413 の 4KB バックアップ SRAM (VBAT 接続時はバッテリーバックアップ) にクラッシュ情報を保存
- または予約フラッシュセクタに書込み
- 再起動後に NSH コマンドで読み出し

---

## 4. NSH デバッグコマンド

### プロセス/タスク監視

| コマンド | 説明 | 必要な設定 |
|---|---|---|
| `ps` | タスク一覧 (PID, 優先度, 状態, スタックサイズ) | デフォルト |
| `free` | メモリ統計 (合計, 使用中, 空き, 最大空きブロック) | デフォルト |
| `top` | 動的 CPU 使用率表示 | `CONFIG_SYSTEM_TOP` |
| `dmesg` | syslog バッファ読み出し | `CONFIG_RAMLOG` |

### /proc ファイルシステム

| パス | 内容 |
|---|---|
| `/proc/<pid>/status` | タスク状態 |
| `/proc/<pid>/stack` | スタック使用量 (StackAlloc, StackBase, MaxStackUsed) |
| `/proc/<pid>/group/fd` | オープンファイルディスクリプタ |
| `/proc/meminfo` | システムメモリ情報 |
| `/proc/uptime` | システム稼働時間 |

### スタックオーバーフロー検出

3 つの相補的メカニズム:

| 方式 | 設定 | 検出タイミング | オーバーヘッド |
|---|---|---|---|
| スタックカラーリング | `CONFIG_STACK_COLORATION` | タスク終了/障害時 | 低 |
| コンテキストスイッチ検査 | デフォルト | コンテキストスイッチ時 | 低 |
| 関数レベル検査 | `CONFIG_ARMV7M_STACKCHECK` | 全関数呼出時 | 高 |

---

## 5. 推奨デバッグ設定 (defconfig)

### 開発ビルド

```
CONFIG_DEBUG_FEATURES=y
CONFIG_DEBUG_ERROR=y
CONFIG_DEBUG_WARN=y
CONFIG_DEBUG_INFO=y
CONFIG_DEBUG_ASSERTIONS=y
CONFIG_DEBUG_HARDFAULTS=y
CONFIG_DEBUG_SYMBOLS=y
CONFIG_STACK_COLORATION=y
CONFIG_RAMLOG=y
CONFIG_RAMLOG_SYSLOG=y
CONFIG_COREDUMP=y
CONFIG_BOARD_COREDUMP_SYSLOG=y
CONFIG_SYSTEM_TOP=y
```

### リリースビルド

```
CONFIG_DEBUG_ERROR=y
CONFIG_DEBUG_HARDFAULTS=y
CONFIG_STACK_COLORATION=y
CONFIG_RAMLOG=y
CONFIG_COREDUMP=y
```
