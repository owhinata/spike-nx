# SPIKE Prime Hub NuttX プロジェクト

SPIKE Prime Hub 上で NuttX RTOS を動作させる環境を構築するプロジェクト。

## Quick Start

### ビルド

```bash
make
```

### フラッシュ (DFU)

```bash
brew install dfu-util  # 初回のみ
```

1. Hub の USB を抜く
2. Bluetooth ボタンを押したまま USB 接続、5秒待って離す（DFU モード）
3. 書き込み:

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

### シリアル接続

```bash
picocom /dev/tty.usbmodem01
```

## ハードウェア仕様

| 項目 | スペック |
|------|---------|
| MCU | STM32F413VG (ARM Cortex-M4, 96MHz) |
| Flash | 1 MB (992KB available, 32KB bootloader) |
| RAM | 320 KB (SRAM1 256KB + SRAM2 64KB) |

## Benchmark

### CoreMark

| Board | MCU | CoreMark Score | CoreMark/MHz |
|-------|-----|---------------|-------------|
| SPIKE Prime Hub | STM32F413VG (96MHz) | 171.19 | 1.78 |
| B-L4S5I-IOT01A | STM32L4R5VI (80MHz) | 143.16 | 1.79 |
