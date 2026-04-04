# ウォッチドッグタイマー

SPIKE Prime Hub における NuttX のウォッチドッグタイマー構成について説明する。

## 概要

| 項目 | 設定 |
|------|------|
| ハードウェア watchdog | STM32 IWDG (Independent Watchdog) |
| タイムアウト | 3000 ms (`CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT=3000`) |
| ping 間隔 | 1000 ms (`CONFIG_WATCHDOG_AUTOMONITOR_PING_INTERVAL=1000`) |
| フィード方式 | カーネル自動 (NuttX wdog タイマー) |
| デバイスパス | `/dev/watchdog0` |

## IWDG の特性

IWDG は LSI (32 kHz) で独立動作するハードウェアウォッチドッグである。

- 他のタイマーリソース (TIM2, TIM5, TIM9 等) と競合しない
- **一度開始するとソフトウェアから停止できない** (ハードウェア仕様)
- システムがハングした場合、タイムアウト後に MCU をリセットする
- リセット後もFlash 上のファームウェアはそのまま残り、NuttX が再起動する

## 構成

### defconfig

```
CONFIG_STM32_IWDG=y
CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG=y
CONFIG_WATCHDOG_AUTOMONITOR_TIMEOUT=3000
CONFIG_WATCHDOG_AUTOMONITOR_PING_INTERVAL=1000
```

### ボード初期化

`stm32_bringup()` 内で IWDG デバイスを登録する:

```c
#ifdef CONFIG_STM32_IWDG
  stm32_iwdginitialize("/dev/watchdog0", STM32_LSI_FREQUENCY);
#endif
```

`STM32_LSI_FREQUENCY` (32000 Hz) は `board.h` で定義済み。

## automonitor の動作

`CONFIG_WATCHDOG_AUTOMONITOR_BY_WDOG` を有効にすると、NuttX の sw wdog タイマー機構を使って IWDG を自動的にフィードする。

1. ボード初期化時に `/dev/watchdog0` が登録される
2. ドライバが IWDG を開始し、automonitor が sw wdog タイマーを設定
3. 1000 ms 間隔で `keepalive` が呼ばれ、IWDG カウンタがリロードされる
4. カーネルがハングして sw wdog タイマーが発火できなくなると、3 秒後に IWDG がリセットを発生させる

アプリケーションが明示的に `/dev/watchdog0` を `open` して `WDIOC_START` すると、automonitor は停止し、アプリケーション側が ping 責任を持つ。

## pybricks との比較

| 項目 | pybricks | NuttX |
|------|----------|-------|
| 使用 watchdog | IWDG | IWDG |
| タイムアウト | 3 秒 | 3 秒 |
| prescaler | /64, reload=1500 | ドライバが自動計算 |
| フィード方式 | supervisor poll ループ内で毎サイクル | sw wdog タイマーで 1 秒間隔 |
| リセット検出 | RCC_CSR_IWDGRSTF を確認 | 未実装 |

## デバッグ時の注意

IWDG は停止できないため、デバッガでプログラムを一時停止すると 3 秒後にリセットが発生する。デバッグ時は `CONFIG_STM32_IWDG` を無効にするか、STM32 の DBGMCU レジスタで IWDG をフリーズする設定が必要になる場合がある。
