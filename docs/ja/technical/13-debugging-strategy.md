# デバッグ戦略

## 1. 概要

SPIKE Prime Hub では SWD ピン (PA13/PA14) が電源制御に転用されているため、通常の SWD デバッグが困難。多層的なデバッグ戦略が必要。

### デバッグ手段の優先順位

| 優先度 | 手段 | 用途 |
|---|---|---|
| **Primary** | USB CDC/ACM NSH + syslog + `dmesg` | 日常開発 |
| **Secondary** | NuttX coredump → バックアップ SRAM 永続化 | クラッシュ後分析 |
| **Tertiary** | デバッグビルド + SWD (USB 電源時のみ) + OpenOCD | 困難な問題の調査 |
| **Diagnostic** | NSH コマンド (`ps`, `free`, `top`, `/proc`) | ランタイム監視 |
| **Emergency** | Connect-Under-Reset (NRST アクセス必要) | ブリックからの復旧 |

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
| `CONFIG_SYSLOG_SERIAL` | UART | I/O ポート UART 経由 |
| `CONFIG_SYSLOG_CHARDEV` | キャラクタデバイス | USB CDC/ACM 等 |
| `CONFIG_RAMLOG` | RAM リングバッファ | `dmesg` で読み出し。USB 切断時も保持 |
| `CONFIG_SYSLOG_FILE` | ファイル | マウント済みファイルシステムへ |

**推奨**: `CONFIG_RAMLOG` を副次的な syslog 出力先として設定。USB CDC/ACM がダウンした場合でもログを保持し、再起動後に `dmesg` で確認可能。

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

**注意**: NSH コンソールが USB CDC/ACM の場合、ハードフォルトパスの出力が USB に送信されない可能性がある。UART コンソールを副次出力として設定することを推奨。

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

**課題**: 現在の NuttX ではクラッシュダンプはシリアル/syslog に出力され、リセット時に失われる。

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

## 5. SWD デバッグ (USB 電源時)

### 原理

STM32 はリセット後、PA13/PA14 を自動的に SWDIO/SWCLK に設定 (内部プルアップ/プルダウン付き)。ファームウェアの GPIO 初期化コードが実行されるまでの短い時間窓で SWD は動作する。

### デバッグビルド戦略

USB 電源時はバッテリー電源維持回路が不要なため、PA13 の GPIO 再設定をスキップ可能:

```c
void stm32_boardinitialize(void)
{
#ifndef CONFIG_BOARD_SWD_DEBUG
    // 本番ビルド: PA13 = BAT_PWR_EN (HIGH)
    stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_OUTPUT_SET |
                     GPIO_PORTA | GPIO_PIN13);
#endif
    // デバッグビルド: PA13 は SWDIO のまま (USB 電源でのみ動作)
}
```

**注意**: SWD は PA13 (SWDIO) と PA14 (SWCLK) の**両方**が必要。片方だけでは動作しない。

### OpenOCD 接続

```bash
# ST-Link 使用
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c '$_TARGETNAME configure -rtos nuttx' \
  -c 'init; reset halt'

# GDB 接続
arm-none-eabi-gdb --tui nuttx -ex 'target extended-remote localhost:3333'
```

NuttX スレッド認識には [sony/openocd-nuttx](https://github.com/sony/openocd-nuttx) フォークを使用。`info threads` で全 NuttX タスクを表示可能。

### Connect-Under-Reset

本番ビルドでも NRST ピンにアクセスできれば SWD 接続可能:

1. NRST を LOW に保持 (リセット状態)
2. ST-Link/J-Link を "connect under reset" モードで接続
3. リセット状態では PA13/PA14 が SWD として動作
4. CPU を GPIO 初期化前に停止
5. SPIKE Hub PCB 上の NRST パッドへの物理アクセスが必要

---

## 6. GDB リモートデバッグ

### NuttX GDB Server

NuttX はアプリケーションレベルの GDB サーバー機能を提供。シリアル接続経由で GDB と通信可能。

### USB CDC/ACM 経由の GDB

原理的には可能だが、NSH コンソールと共用になる課題:
1. 2つ目の CDC/ACM インスタンスを GDB 専用にする
2. I/O ポートの UART を GDB 用に使用
3. モード切替方式 (NSH ↔ GDB)

### 推奨: I/O ポート UART をデバッグ用に確保

開発中はポート A (UART7) をデバッグ UART として確保し、USB CDC/ACM は NSH コンソールに使用。GDB が必要な場合は UART7 経由で接続。

---

## 7. 推奨デバッグ設定 (defconfig)

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
