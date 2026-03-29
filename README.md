# spike-nx

## Benchmark Results

### CoreMark

| Board | MCU | CoreMark Score | CoreMark/MHz | Compiler | Flags |
|-------|-----|---------------|-------------|----------|-------|
| B-L4S5I-IOT01A | STM32L4R5VI (80MHz) | 143.16 | 1.79 | GCC 13.2.1 | `-Os` |

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
