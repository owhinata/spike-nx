# BUILD_PROTECTED 移行

Issue #37 の追跡。Cortex-M4 MPU を用いた kernel / user 空間分離への移行作業。**現状: WIP (ビルドは通るが実機起動未確認)**。

## 目的

`CONFIG_BUILD_FLAT=y` (デフォルト) だと `apps/` 配下のユーザーコードも privileged thread mode で動作するため、`MSR BASEPRI` 等の特権命令で割込マスクを直接操作できてしまう。`CONFIG_BUILD_PROTECTED=y` に移行すると MPU によって:

- ユーザーコードは user mode で動作、特権命令は実行不可
- 割込マスク・kernel メモリ・周辺レジスタへの直接アクセスを HardFault/MemFault で阻止
- システムロバスト性が大幅に向上

## メモリマップ

SPIKE Prime Hub (STM32F413VG, Flash 1.5MB / RAM 320KB) 用に設計:

### Flash (0x08008000 開始、0x08000000-0x08008000 は LEGO bootloader)

| 区画 | 範囲 | サイズ | 用途 |
|---|---|---|---|
| `kflash` | `0x08008000..0x08080000` | 480K | 特権カーネルコード (alignment gap 込み) |
| `uflash` | `0x08080000..0x08100000` | 512K | user-space code (2^n aligned) |
| `xflash` | `0x08100000..0x08180000` | 512K | user-space 予備領域 |

`CONFIG_NUTTX_USERSPACE = 0x08080000`

### SRAM (0x20000000, 320KB)

| 区画 | 範囲 | サイズ | 用途 |
|---|---|---|---|
| `ksram` | `0x20000000..0x20010000` | 64K | kernel .data/.bss (linker placement only) |
| `usram` | `0x20010000..0x20020000` | 64K | user .data/.bss (64K 境界整列) |
| `xsram` | `0x20020000..0x20050000` | 192K | runtime heap (kernel heap + user heap) |

`CONFIG_MM_KERNEL_HEAPSIZE = 32768` (下限値; 実行時は `stm32_allocateheap.c` が残りを自動分割)

## 実装ファイル

| パス | 内容 |
|---|---|
| `boards/spike-prime-hub/configs/usbnsh/defconfig` | `CONFIG_BUILD_PROTECTED=y`, MPU, USERSPACE など追加 |
| `boards/spike-prime-hub/kernel/Makefile` | PASS1 (user blob) ビルドルール |
| `boards/spike-prime-hub/kernel/stm32_userspace.c` | user blob 先頭の `struct userspace_s` |
| `boards/spike-prime-hub/kernel/CMakeLists.txt` | 同上 (CMake 用) |
| `boards/spike-prime-hub/scripts/memory.ld` | 物理メモリレイアウト |
| `boards/spike-prime-hub/scripts/kernel-space.ld` | kernel blob 用セクション配置 |
| `boards/spike-prime-hub/scripts/user-space.ld` | user blob 用セクション配置 |
| `boards/spike-prime-hub/scripts/Make.defs` | `ARCHSCRIPT` を `memory.ld + kernel-space.ld` に変更 |
| `apps/imu/*.c` | `clock_systime_ticks()` → `clock_gettime(CLOCK_MONOTONIC)` (user-space で呼べる POSIX 関数に置換) |

## 移行に伴う他の変更

| 項目 | 対応 | 理由 |
|---|---|---|
| `CONFIG_APP_LED` 無効化 | 一時 | `led_main.c` が `tlc5955_set_duty()` を直呼び。`/dev/rgbled` 風 char driver 経由へ refactor 必要 |
| `CONFIG_ARCH_PERF_EVENTS` 無効化 | — | `nuttx-apps/testing/ostest/perf_gettime.c` が `perf_gettime()` を呼ぶが syscall 化されていない |
| `CONFIG_STM32_IWDG`, `CONFIG_WATCHDOG_AUTOMONITOR` 無効化 | デバッグ用 | 早期クラッシュの切り分けで無効化。将来の BUILD_PROTECTED 安定時に再有効化検証 |

## ビルド

```bash
make nuttx-distclean && make
```

成果物:

- `nuttx/nuttx.bin` (~138KB) — kernel blob @ `0x08008000`
- `nuttx/nuttx_user.bin` (~146KB) — user blob @ `0x08080000`

## フラッシュ (2段階)

```bash
# DFU モードへ: USB 抜く → BT ボタン押したまま USB 接続 → 5秒
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
```

## 未解決の問題 (2026-04-18)

ビルドは正常に通り、両 blob とも DFU で書き込み成功。しかし**実機起動で USB CDC の enumeration に至らない** (黄色 LED 1Hz 点滅 = `CONFIG_BOARD_RESET_ON_ASSERT=2` によるクラッシュ→reset ループと推測)。

### 試した対策 (いずれも改善せず)

- `CONFIG_ARM_MPU_EARLY_RESET=y` / `CONFIG_ARM_MPU_RESET=y` 追加
- `CONFIG_WATCHDOG_AUTOMONITOR` 無効化
- `CONFIG_STM32_IWDG` 無効化
- 最小構成(apps 全無効, 周辺ドライバ削除, `CONFIG_BOARD_LATE_INITIALIZE` 無効) でも同症状

### 診断の制約

SPIKE Prime Hub は SWD が基板内部にあり、ケースを開けずにはアクセス不可。また UART コンソールは CDCACM のみで、CDC enumeration 前のクラッシュだと `syslog` / `RAMLOG` にも届かない。シリアル経由のライブデバッグができないため、これ以上の blind bisection は非効率と判断。

### 今後の方針

1. **STM32F413 Discovery ボード** (ST 公式 eval board, SWD 容易) で同じ BUILD_PROTECTED 構成を試す
2. そこでクラッシュ箇所を特定 (GDB + SWD or printf debug via USART)
3. 原因判明後、SPIKE Hub 固有部分(メモリマップ・LEGO bootloader 対応)に適用

### 成果物の保存

この実装は `feat/build-protected` ブランチに保存。main には merge せず、SWD デバッグ後に再開する。

## 参考

- NuttX 公式の BUILD_PROTECTED リファレンス: `nuttx/boards/arm/stm32/stm32f4discovery/configs/kostest/`
- STM32F4 MPU 初期化: `nuttx/arch/arm/src/stm32/stm32_mpuinit.c`
- ヒープ動的分割: `nuttx/arch/arm/src/stm32/stm32_allocateheap.c`
- Kconfig: `nuttx/Kconfig` (BUILD_PROTECTED / PASS1_* / NUTTX_USERSPACE)
