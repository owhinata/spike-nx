# 開発ワークフロー

> **注**: 本ドキュメントは 06-dev-workflow.md を置換する。

## 1. 開発環境の全体像

SPIKE Prime Hub の実機が未入手のため、同一 MCU ファミリ (STM32F413) を搭載する **STM32F413H-Discovery Kit** でブリングアップを行う。Powered Up デバイスドライバの実装は Hub 入手後。

### STM32F413H-Discovery vs SPIKE Prime Hub

| 項目 | Discovery Kit | SPIKE Prime Hub |
|---|---|---|
| MCU | STM32F413**ZH**T6 (144pin) | STM32F413**VG**T6 (100pin) |
| Flash | 1.5 MB | 1 MB (VG 公称。実質 1.5MB の可能性あり — 要実機確認) |
| RAM | 320 KB | 320 KB |
| HSE | 8 MHz | 16 MHz |
| Flash 開始 | 0x08000000 | 0x08008000 (LEGO ブートローダー後) |
| SWD | ST-Link/V2-1 **使用可能** | PA13/PA14 転用、**使用不可** |
| NSH コンソール | USB CDC/ACM | USB CDC/ACM |
| USART6 | ST-Link VCP に接続 (デバッグ予備) | 未アサイン (pybricks デバッグ用のみ) |
| LED | GPIO 直接 (LD1=PE3, LD2=PC5) | TLC5955 via SPI1 (GPIO 直接制御不可) |
| PA13/PA14 | SWDIO/SWCLK (デバッグ) | BAT_PWR_EN / PORT_3V3_EN (電源制御) |
| USB OTG FS | あり | あり |
| User Button | あり | あり (Bluetooth ボタン) |

> **Flash サイズ注記**: SPIKE Hub の MCU は STM32F413VG (公称 1MB Flash)。ただし pybricks のリンカスクリプトはブートローダー後 992K を使用しており、STM32F413 は全バリアントで物理的に 1.5MB Flash を持つ可能性がある。実機での確認が必要。

### NuttX フォーク

- https://github.com/owhinata/nuttx (`f413-support` ブランチ)
- https://github.com/owhinata/nuttx-apps

---

## 2. ブリングアップ計画

### Phase A: ビルド環境構築

| 作成ファイル | 内容 |
|---|---|
| `docker/Dockerfile.nuttx` | Ubuntu 24.04 + gcc-arm-none-eabi + python3-kconfiglib 等 |
| `scripts/nuttx.mk` | ビルドスクリプト (pybricks.mk パターン踏襲) |
| `.gitmodules` (更新) | owhinata/nuttx + owhinata/nuttx-apps を submodule 追加 |

**nuttx.mk ターゲット:**

| ターゲット | 動作 |
|---|---|
| `build` (デフォルト) | submodule init → Docker build → configure → make |
| `configure` | `tools/configure.sh` 実行 |
| `clean` | `make clean` |
| `distclean` | 全クリーン (Docker image 削除 + submodule deinit) |
| `menuconfig` | 対話的設定 |
| `savedefconfig` | defconfig 更新 |
| `flash` | Discovery: OpenOCD SWD / Hub: dfu-util DFU |

**BOARD 切替:**

```bash
make -f scripts/nuttx.mk                          # デフォルト: stm32f413-discovery
make -f scripts/nuttx.mk BOARD=spike-prime-hub     # SPIKE Hub
```

### Phase B: F412 代用ブリングアップ (Discovery)

F412ZG 設定を代用し、Discovery Kit で NSH を動作させる。コンソールは **USB CDC/ACM** (SPIKE Hub と同一構成)。USART6 は空けておく。

**ボード定義:**

```
boards/stm32f413-discovery/
  Kconfig
  configs/nsh/defconfig           # F412ZG 代用、USB CDC/ACM コンソール
  include/board.h                 # 8MHz HSE → 96MHz SYSCLK
  scripts/Make.defs
  scripts/ld.script               # 0x08000000, 1024K Flash, 256K SRAM (F412 制限)
  src/Make.defs
  src/stm32_boot.c
  src/stm32_bringup.c
  src/stm32_usbdev.c              # USB デバイス初期化
  src/stm32f413_discovery.h
```

**成功基準:** NSH が USB CDC/ACM で動作、LED 点灯

### Phase C: STM32F413 チップサポート (NuttX フォーク)

owhinata/nuttx の `f413-support` ブランチに 10 ファイルパッチを適用:

1. Kconfig: F413 ファミリ + UART9/10
2. chip.h: ペリフェラル数、1.5MB Flash / 320KB SRAM
3. stm32f40xxx_irq.h: UART9 IRQ=88, UART10 IRQ=89
4. stm32f40xxx_memorymap.h: UART9/10 ベースアドレス
5. stm32f40xxx_rcc.h: APB2ENR UART9/10 ビット
6. stm32f40xxx_rcc.c: rcc_enableapb2() に UART9/10 追加
7. stm32_serial.c: UART9/10 デバイスインスタンス (PCLK2 使用)
8. stm32f413xx_pinmap.h: 新規 F413 ピンマップ
9. stm32_pinmap.h: F413 include 分岐
10. boards/Kconfig: ボードエントリ

**成功基準:** `free` で ~320KB RAM 表示、ハードフォルトなし

### Phase D: UART9/10 検証 (Discovery)

Discovery Kit で F413 固有 UART9/10 を動作確認:

| UART | TX | RX | 備考 |
|---|---|---|---|
| UART9 | PD15 (AF11) | PD14 (AF11) | |
| UART10 | PG12 (AF11) | PG11 (AF11) | PE3 は LD1 と競合するため回避 |

**成功基準:** USB-UART アダプタ経由で 115200 baud 送受信成功

### Phase E: SPIKE Hub ボード定義 (ビルドのみ)

Hub 用ボード定義を作成。実機テストは入手後。

```
boards/spike-prime-hub/
  configs/nsh/defconfig           # USB CDC/ACM コンソール
  include/board.h                 # 16MHz HSE → 96MHz SYSCLK
  scripts/ld.script               # 0x08008000, 992K Flash, 320K SRAM
  src/stm32_boot.c                # PA13 BAT_PWR_EN, PA14 PORT_3V3_EN
```

**成功基準:** ビルド成功、バイナリが 0x08008000 にリンク

### 実行順序

```
Phase A ──→ Phase B ──→ Phase C ──→ Phase D
                           │
                           └──→ Phase E
```

---

## 3. 開発サイクル

### Discovery Kit

```
コード編集
  ↓
make -f scripts/nuttx.mk                         # Docker 内ビルド
  ↓
make -f scripts/nuttx.mk flash                   # OpenOCD SWD フラッシュ
  ↓
screen /dev/ttyACM0 115200                        # USB CDC/ACM コンソール
```

フラッシュ (OpenOCD SWD):
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "program nuttx/nuttx.bin 0x08000000 verify reset exit"
```

GDB デバッグ (Discovery のみ):
```bash
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c '$_TARGETNAME configure -rtos nuttx' -c 'init; reset halt'
# 別ターミナル:
arm-none-eabi-gdb nuttx/nuttx -ex 'target extended-remote localhost:3333'
```

### SPIKE Prime Hub (DFU) — Hub 入手後

```
コード編集
  ↓
make -f scripts/nuttx.mk BOARD=spike-prime-hub    # Docker 内ビルド
  ↓
DFU モード進入                                     # Bluetooth ボタン長押し + USB 接続
  ↓
make -f scripts/nuttx.mk BOARD=spike-prime-hub flash  # dfu-util DFU フラッシュ
  ↓
screen /dev/ttyACM0 115200                         # USB CDC/ACM コンソール
```

DFU モード進入手順:
1. USB ケーブルを抜く
2. Bluetooth ボタンを 5 秒間長押し
3. ボタンを押したまま USB ケーブルを接続
4. ステータス LED が点滅 → DFU モード
5. ボタンを離す

DFU フラッシュコマンド:
```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx/nuttx.bin
```

### 共通操作

```bash
make -f scripts/nuttx.mk build                    # ビルドのみ
make -f scripts/nuttx.mk menuconfig                # 対話的設定
make -f scripts/nuttx.mk savedefconfig             # defconfig 更新
make -f scripts/nuttx.mk clean                     # ビルド成果物削除
make -f scripts/nuttx.mk distclean                 # 完全クリーン
```

---

## 4. デバッグ方針

### Discovery Kit

| 手段 | 用途 |
|---|---|
| **SWD + OpenOCD + GDB** | ステップ実行、ブレークポイント、メモリ検査 |
| **USB CDC/ACM NSH** | NSH コンソール + syslog + `dmesg` |
| **USART6 VCP** | デバッグ予備 (NSH には使用しない。syslog 副次出力等に利用可能) |
| **NSH コマンド** | `ps`, `free`, `top`, `dmesg`, `/proc` |
| **LED (PE3, PC5)** | ブート進捗表示 |

### SPIKE Prime Hub (入手後)

SWD 使用不可。デバッグ手段は限定的:

| 手段 | 用途 |
|---|---|
| **USB CDC/ACM NSH** | 日常開発の主要手段 |
| **RAMLOG + dmesg** | USB 切断時/起動初期のログ保持 |
| **coredump** | クラッシュ後分析 (バックアップ SRAM 永続化) |
| **NSH コマンド** | `ps`, `free`, `top`, `/proc` |

詳細は [13-debugging-strategy.md](13-debugging-strategy.md) を参照。
