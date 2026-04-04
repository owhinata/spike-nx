# デバッグ

SPIKE Prime Hub のデバッグ手法。SWD ピン (PA13/PA14) が電源制御に転用されているため、SWD デバッグは使用しない。

## デバッグ手段

| 優先度 | 手段 | 用途 |
|---|---|---|
| Primary | USB CDC/ACM NSH + syslog + `dmesg` | 日常開発 |
| Secondary | NuttX coredump | クラッシュ後分析 |
| Diagnostic | NSH コマンド (`ps`, `free`, `top`, `/proc`) | ランタイム監視 |
| Last resort | I/O ポート UART を一時的にデバッグ出力に使用 | USB 未動作時 |

## RAMLOG と dmesg

`CONFIG_RAMLOG` を有効化すると syslog 出力が RAM リングバッファに保存される。USB CDC/ACM がダウンしてもログを保持し、再接続後に `dmesg` で確認可能。

ブート初期段階のメッセージは USB ドライバ動作前に出力されるため、RAMLOG でのみ確認できる。

```
CONFIG_RAMLOG=y
CONFIG_RAMLOG_SYSLOG=y
```

## DEBUG サブシステム

`CONFIG_DEBUG_FEATURES=y` がマスタースイッチ。有効化後、サブシステムごとに ERROR / WARN / INFO レベルを個別に設定できる。

### 優先度レベル

| 設定 | レベル | 説明 |
|---|---|---|
| `CONFIG_DEBUG_ERROR` | Error | 最重要 |
| `CONFIG_DEBUG_WARN` | Warning | 警告 |
| `CONFIG_DEBUG_INFO` | Info | 詳細 (最も冗長) |

### サブシステム

| 設定 | 対象 |
|---|---|
| `CONFIG_DEBUG_USB` | USB スタック |
| `CONFIG_DEBUG_SCHED` | スケジューラ |
| `CONFIG_DEBUG_MM` | メモリ管理 |
| `CONFIG_DEBUG_GPIO` | GPIO |
| `CONFIG_DEBUG_I2C` | I2C バス |
| `CONFIG_DEBUG_SPI` | SPI バス |

## 推奨 Kconfig 設定

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

## NSH デバッグコマンド

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

## HardFault 分析

NuttX は Cortex-M の HardFault 時に例外フレームからレジスタダンプを出力する。

```
CONFIG_DEBUG_HARDFAULTS=y        # 障害レジスタ出力 (CFSR, HFSR, MMFAR, BFAR)
CONFIG_STACK_COLORATION=y        # 全タスクのスタック使用量をダンプ
CONFIG_DEBUG_ASSERTIONS=y        # ASSERT/PANIC パスでレジスタダンプ
```

### 障害レジスタ

| レジスタ | 説明 |
|---|---|
| CFSR (Configurable Fault Status Register) | UsageFault / BusFault / MemManage の詳細 |
| HFSR (HardFault Status Register) | HardFault の原因 |
| MMFAR (MemManage Fault Address Register) | メモリ管理違反のアドレス |
| BFAR (BusFault Address Register) | バスフォルトのアドレス |

NSH コンソールが USB CDC/ACM の場合、HardFault 時に USB ドライバ自体が動作不能になる可能性がある。RAMLOG にクラッシュ情報が書き込まれていれば、ファームウェア再書込み後に `dmesg` で確認できる。

### coredump

NuttX の coredump サブシステムで事後分析が可能:

```
CONFIG_COREDUMP=y
CONFIG_BOARD_COREDUMP_SYSLOG=y
CONFIG_BOARD_COREDUMP_COMPRESSION=y    # LZF 圧縮
CONFIG_BOARD_COREDUMP_FULL=y           # 全タスク情報
```

出力は hex エンコード + LZF 圧縮。`tools/coredump.py` で ELF core ファイルに変換し、GDB で分析する。

## スタックオーバーフロー検出

| 方式 | 設定 | 検出タイミング | オーバーヘッド |
|---|---|---|---|
| スタックカラーリング | `CONFIG_STACK_COLORATION` | タスク終了/障害時 | 低 |
| コンテキストスイッチ検査 | デフォルト | コンテキストスイッチ時 | 低 |
| 関数レベル検査 | `CONFIG_ARMV7M_STACKCHECK` | 全関数呼出時 | 高 |
