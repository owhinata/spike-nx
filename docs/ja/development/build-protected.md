# BUILD_PROTECTED 移行

Issue #37 の追跡。Cortex-M4 MPU を用いた kernel / user 空間分離への移行作業。**現状: SPIKE Prime Hub の usbnsh 構成で動作確認済 (2026-04-18)**。

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

`SRAM1_END = 0x20050000` は 2^n 境界ではないため、MPU で保護できる最大 user heap 領域 (128K) を 2^n 境界に切り下げると 60K 程度の **末端 tail 領域** が宙に浮く。これを救済するために以下の 2 設定を有効化している (詳細は [NuttX 側の必須修正](#nuttx) 参照):

- `CONFIG_MM_REGIONS=2`
- `CONFIG_STM32_CCMEXCLUDE=y` (F413 に CCM SRAM は無いが、第 2 region を tail 救済に転用するため宣言)

## 実装ファイル

| パス | 内容 |
|---|---|
| `boards/spike-prime-hub/configs/usbnsh/defconfig` | BUILD_PROTECTED 関連 CONFIG 一式 |
| `boards/spike-prime-hub/kernel/Makefile` | PASS1 (user blob) ビルドルール |
| `boards/spike-prime-hub/kernel/stm32_userspace.c` | user blob 先頭の `struct userspace_s` |
| `boards/spike-prime-hub/kernel/CMakeLists.txt` | 同上 (CMake 用) |
| `boards/spike-prime-hub/scripts/memory.ld` | 物理メモリレイアウト |
| `boards/spike-prime-hub/scripts/kernel-space.ld` | kernel blob 用セクション配置 |
| `boards/spike-prime-hub/scripts/user-space.ld` | user blob 用セクション配置 |
| `boards/spike-prime-hub/scripts/Make.defs` | `ARCHSCRIPT` を `memory.ld + kernel-space.ld` に変更 |
| `apps/imu/*.c` | `clock_systime_ticks()` → `clock_gettime(CLOCK_MONOTONIC)` (user-space で呼べる POSIX 関数に置換) |

## 主要 CONFIG

```
# BUILD_PROTECTED 本体
CONFIG_BUILD_PROTECTED=y
CONFIG_NUTTX_USERSPACE=0x08080000
CONFIG_PASS1_BUILDIR="../boards/spike-prime-hub/kernel"

# MPU
CONFIG_ARM_MPU=y
CONFIG_ARM_MPU_EARLY_RESET=y
CONFIG_ARM_MPU_RESET=y

# Heap / stack
CONFIG_MM_KERNEL_HEAPSIZE=32768
CONFIG_MM_REGIONS=2
CONFIG_STM32_CCMEXCLUDE=y
CONFIG_INIT_STACKSIZE=4096
CONFIG_SYSTEM_NSH_STACKSIZE=4096
```

## NuttX 側の必須修正

owhinata fork の `f413-support-12.13.0` ブランチに以下 3 commit を含む:

| Commit | 内容 |
|---|---|
| `97716f5a2a` | `arch/armv7-m`: `arm_dispatch_syscall` で caller の r11 を保存 (BUILD_PROTECTED の syscall 経路で AAPCS が壊れていた) |
| `b35c473a58` | `arch/stm32`: `SRAM1_END` が 2^n 境界でない場合に MPU-aligned user heap が末端を取りこぼす問題を `arm_addregion()` の追加 region で救済 (要 `CONFIG_MM_REGIONS>=2`) |
| `7c116a6de2` | `arch/arm`: BUILD_PROTECTED 構成で fork サポートを無効化 (kernel/user 境界で stack 共有できないため) |

## 移行に伴う他の変更

| 項目 | 対応 | 理由 |
|---|---|---|
| `CONFIG_APP_LED` 有効化 | Issue #39 で対応済 (2026-04-19) | `/dev/rgbled0` char ドライバ (ioctl) 経由に refactor して user blob から利用可能にした |
| `CONFIG_ARCH_PERF_EVENTS` 無効化 | 別 issue で対応 | `nuttx-apps/testing/ostest/perf_gettime.c` が `perf_gettime()` を呼ぶが syscall 化されていない |

## ビルド

```bash
make nuttx-distclean && make
```

成果物:

- `nuttx/nuttx.bin` (~139KB) — kernel blob @ `0x08008000`
- `nuttx/nuttx_user.bin` (~146KB) — user blob @ `0x08080000`

## フラッシュ (2段階)

```bash
# DFU モードへ: USB 抜く → BT ボタン押したまま USB 接続 → 5秒
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
```

## 実機動作確認 (2026-04-18, commit `55347a8`)

| 項目 | 結果 |
|---|---|
| NSH 起動 (USB CDC) | ✅ `NuttShell (NSH) NuttX-12.13.0` プロンプト到達 |
| `free` の Umem | ✅ 196608 B (192K) — 末端 tail 救済で plan 通り |
| `free` の Kmem | ✅ 62536 B |
| CoreMark 2000 iter | ✅ **170.46 iter/sec** |
| `sleep 10` (WATCHDOG 干渉確認) | ✅ 10.0 秒で完走、WATCHDOG fire なし |
| `pytest -m "not slow and not interactive"` | ✅ **30 passed** (Issue #39 で `APP_LED` 再有効化後、LED 関連 2 件も PASS) |
| Crash 系 (assert / null deref / divzero / stack overflow) | ✅ 4/4 |
| Driver 系 (battery / IMU / I2C) | ✅ 6/6 |
| Sound 系 | ✅ 9/9 |
| System 系 (watchdog / cpuload / stackmonitor) | ✅ 3/3 |

## 既知の follow-up

- **ostest 中断** ([#38](https://github.com/owhinata/spike-nx/issues/38)): `ostest` を起動すると serial がドロップして board reset する。`sleep 10` は問題ないので WATCHDOG の単純 fire ではなく、特定 subtest の assertion または HardFault による `BOARD_RESET_ON_ASSERT=2` reset の可能性が高い。f413-discovery knsh では完走しているので、SPIKE Hub 固有のペリフェラル (battery / IMU / sound 等) 起動時 init との相互作用が疑わしい。
- **ARCH_PERF_EVENTS の syscall 化** ([#40](https://github.com/owhinata/spike-nx/issues/40)): `perf_gettime()` を syscall として登録するか、ostest 側で `clock_gettime` ベースに置き換える。

## 参考

- NuttX 公式の BUILD_PROTECTED リファレンス: `nuttx/boards/arm/stm32/stm32f4discovery/configs/kostest/`
- STM32F4 MPU 初期化: `nuttx/arch/arm/src/stm32/stm32_mpuinit.c`
- ヒープ動的分割: `nuttx/arch/arm/src/stm32/stm32_allocateheap.c`
- Kconfig: `nuttx/Kconfig` (BUILD_PROTECTED / PASS1_* / NUTTX_USERSPACE)
- 検証用足場: `boards/stm32f413-discovery/configs/knsh/`
