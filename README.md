# spike-nx

## Benchmark Results

### CoreMark

| Board | MCU | CoreMark Score | CoreMark/MHz | Compiler | Flags |
|-------|-----|---------------|-------------|----------|-------|
| SPIKE Prime Hub | STM32F413VG (96MHz) | 171.19 | 1.78 | GCC 13.2.1 | `-Os` |
| B-L4S5I-IOT01A | STM32L4R5VI (80MHz) | 143.16 | 1.79 | GCC 13.2.1 | `-Os` |

<details>
<summary>SPIKE Prime Hub raw output</summary>

```
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 1168261
Total time (secs): 11.682610
Iterations/Sec   : 171.194622
Iterations       : 2000
Compiler version : GCC13.2.1 20231009
Compiler flags   : -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g
Memory location  : HEAP
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x4983
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 171.194622 / GCC13.2.1 20231009 -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g / HEAP
```

</details>

<details>
<summary>B-L4S5I-IOT01A raw output</summary>

```
2K performance run parameters for coremark.
CoreMark Size    : 666
Total ticks      : 1396995
Total time (secs): 13.969950
Iterations/Sec   : 143.164435
Iterations       : 2000
Compiler version : GCC13.2.1 20231009
Compiler flags   : -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g
Memory location  : HEAP
seedcrc          : 0xe9f5
[0]crclist       : 0xe714
[0]crcmatrix     : 0x1fd7
[0]crcstate      : 0x8e3a
[0]crcfinal      : 0x4983
Correct operation validated. See README.md for run and reporting rules.
CoreMark 1.0 : 143.164435 / GCC13.2.1 20231009 -Os -fno-strict-aliasing -fno-omit-frame-pointer -fno-optimize-sibling-calls -funwind-tables -fasynchronous-unwind-tables --param=min-pagesize=0 -fno-common -Wall -Wshadow -Wundef -ffunction-sections -fdata-sections -g / HEAP
```

</details>

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
