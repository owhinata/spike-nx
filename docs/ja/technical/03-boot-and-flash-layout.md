# ブートプロセスとフラッシュレイアウト

## 1. STM32F413VG フラッシュセクタレイアウト

SPIKE Prime Hub の MCU は STM32F413**VG** (公称 1 MB Flash)。ただし STM32F413 ファミリは物理的に 1.5 MB Flash を持つ可能性がある (pybricks はブートローダー後 992K を使用)。**実機確認まで 1 MB 前提**で設計する。

> **注**: STM32F413H-Discovery Kit は STM32F413**ZH** (1.5 MB Flash)。Bank 2 (セクタ 12-15) は Discovery では利用可能だが、SPIKE Hub では要確認。

### Bank 1 (1 MB, セクタ 0-11)

| セクタ | アドレス | サイズ | 用途 |
|--------|---------|-------|------|
| 0 | `0x08000000` | 16 KB | LEGO ブートローダー |
| 1 | `0x08004000` | 16 KB | LEGO ブートローダー |
| 2 | `0x08008000` | 16 KB | **ファームウェア開始** |
| 3 | `0x0800C000` | 16 KB | ファームウェア |
| 4 | `0x08010000` | 64 KB | ファームウェア |
| 5 | `0x08020000` | 128 KB | ファームウェア |
| 6 | `0x08040000` | 128 KB | ファームウェア |
| 7 | `0x08060000` | 128 KB | ファームウェア |
| 8 | `0x08080000` | 128 KB | ファームウェア |
| 9 | `0x080A0000` | 128 KB | ファームウェア |
| 10 | `0x080C0000` | 128 KB | ファームウェア |
| 11 | `0x080E0000` | 128 KB | ファームウェア |

### Bank 2 (512 KB, セクタ 12-15) — 要実機確認

| セクタ | アドレス | サイズ | 用途 |
|--------|---------|-------|------|
| 12 | `0x08100000` | 128 KB | 追加領域 (VG では利用不可の可能性) |
| 13 | `0x08120000` | 128 KB | 同上 |
| 14 | `0x08140000` | 128 KB | 同上 |
| 15 | `0x08160000` | 128 KB | 同上 |

**Bank 1 合計**: 4×16 KB + 1×64 KB + 7×128 KB = 1024 KB = 1 MB (VG 公称)
**Bank 1+2 合計**: 1024 KB + 512 KB = 1536 KB = 1.5 MB (ZH / 物理的に存在する場合)

出典: RM0430 (STM32F413/423 リファレンスマニュアル) Table 5

---

## 2. SPIKE Prime Hub ブートチェーン

### メモリレイアウト

pybricks リンカスクリプト (`pybricks/lib/pbio/platform/prime_hub/platform.ld`) より:

```
FLASH_BOOTLOADER : ORIGIN = 0x08000000, LENGTH = 32K   (セクタ 0-1)
FLASH_FIRMWARE   : ORIGIN = 0x08008000, LENGTH = 992K   (セクタ 2 以降)
RAM              : ORIGIN = 0x20000000, LENGTH = 320K   (SRAM1 256K + SRAM2 64K)
```

```
0x08000000 ┌─────────────────────┐
           │ LEGO ブートローダー  │ 32 KB (書換不可)
0x08008000 ├─────────────────────┤
           │                     │
           │   ファームウェア     │ 992 KB (書換可能)
           │                     │
0x08100000 ├─────────────────────┤
           │   追加領域           │ 512 KB
0x08180000 └─────────────────────┘

0x20000000 ┌─────────────────────┐
           │   SRAM1             │ 256 KB
0x20040000 ├─────────────────────┤
           │   SRAM2             │ 64 KB
0x20050000 └─────────────────────┘
```

### ブートシーケンス

1. **LEGO ブートローダー** (0x08000000-0x08007FFF)
   - 書換不可の組み込みブートローダー
   - DFU モード: Bluetooth ボタン 5 秒長押し + USB 接続
   - DFU VID/PID: `0x0694` / `0x0008`
   - ファームウェア (0x08008000) へジャンプ

2. **Reset_Handler** (startup.s)
   - スタックポインタ設定 (`_estack`)
   - `.data` セクションをフラッシュから SRAM へコピー
   - `.bss` セクションをゼロクリア
   - `SystemInit()` 呼び出し

3. **SystemInit()** (platform.c)
   - 8バイトスタックアラインメント有効化
   - **VTOR 設定**: `SCB->VTOR = &_fw_isr_vector_src`
   - PLL 設定: 16 MHz HSE → 96 MHz SYSCLK
   - クロック分周: AHB /1, APB1 /2, APB2 /1
   - 全ペリフェラルクロック有効化
   - **BAT_PWR_EN (PA13) を HIGH** → 電源維持

4. **main()** → アプリケーション実行

---

## 3. NuttX VTOR 設定

### NuttX のベクタテーブル配置方式

NuttX は `stm32_irq.c` で VTOR を設定:

```c
putreg32((uint32_t)_vectors, NVIC_VECTAB);
```

`_vectors` シンボルはリンカスクリプトで配置される。`.vectors` セクションを FLASH 領域の先頭に配置すれば、VTOR は自動的に正しいアドレスになる。

**専用の `CONFIG_STM32_VECTAB_OFFSET` 設定は存在しない。** リンカスクリプトの FLASH ORIGIN が実質的に VTOR を決定する。

### SPIKE Prime Hub での設定方法

1. リンカスクリプトで `FLASH ORIGIN = 0x08008000, LENGTH = 992K` を設定
2. `.vectors` セクションを FLASH 領域の先頭に配置
3. `_vectors` シンボルが `0x08008000` に解決される
4. `stm32_irq.c` が自動的に `NVIC_VECTAB = 0x08008000` を書き込む
5. VTOR オフセット `0x8000` (32 KB) は ARM の 512 バイトアラインメント要件を満たす

既存の NuttX コードの変更は不要。リンカスクリプトの FLASH ORIGIN を設定するだけで対応可能。

---

## 4. BAT_PWR_EN (PA13) 電源管理

### 問題

PA13 は以下の 2 つの機能を持つ:
- **JTMS-SWDIO**: SWD デバッグインターフェース (デフォルト)
- **BAT_PWR_EN**: バッテリー電源維持 (GPIO 出力として使用)

バッテリー駆動時、PA13 を HIGH に設定しないとハブの電源が落ちる。

### pybricks の実装

`SystemInit()` 内 (platform.c 1043-1049行目) で最も早いタイミングで設定:

```c
// PA13 を push-pull 出力に設定し HIGH 駆動
GPIO_InitTypeDef gpio_init = {
    .Pin = GPIO_PIN_13,
    .Mode = GPIO_MODE_OUTPUT_PP,
};
HAL_GPIO_Init(GPIOA, &gpio_init);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
```

### NuttX での実装方針

NuttX ブートシーケンス:

```
__start()
  ├─ stm32_clockconfig()     ← クロック設定
  ├─ arm_fpuconfig()         ← FPU 有効化
  ├─ stm32_lowsetup()        ← 早期 UART 設定
  ├─ stm32_gpioinit()        ← GPIO クロック有効化
  ├─ BSS クリア、.data コピー
  ├─ stm32_boardinitialize() ← ★ ボード固有早期初期化
  └─ nx_start()              ← OS カーネル起動
```

**`stm32_boardinitialize()`** が PA13 設定の適切な場所:
- OS サービスは利用不可だが、直接レジスタアクセスは可能
- `stm32_gpioinit()` で GPIO クロックが有効化済み
- `stm32_configgpio()` で PA13 を出力に設定し HIGH 駆動

```c
void stm32_boardinitialize(void)
{
    // バッテリー電源維持 (PA13 = BAT_PWR_EN)
    stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ |
                     GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN13);
}
```

### pybricks との差分分析

pybricks の `SystemInit()` (platform.c 999-1050行) での実行順序:

```
1. SCB->CCR: スタックアラインメント
2. SCB->VTOR: ベクタテーブル再配置
3. HAL_RCC_OscConfig(): HSE + PLL 設定 (16MHz → 96MHz)  ← HSE起動+PLLロック待ち (~数ms)
4. HAL_RCC_ClockConfig(): SYSCLK=PLL, AHB/APB 分周設定
5. RCC->AHB1ENR: GPIOA-E, DMA1-2 クロック有効化          ← GPIO クロック有効化
6. RCC->APB1ENR: UART, TIM, I2C, DAC, SPI2 クロック有効化
7. RCC->APB2ENR: TIM1/8, UART9/10, ADC1, SPI1 クロック有効化
8. RCC->AHB2ENR: OTG_FS クロック有効化
9. PA13 = OUTPUT_PP, HIGH                                 ← BAT_PWR_EN 設定
```

NuttX の `__start()` での対応する実行順序:

```
1. stm32_clockconfig()     ← HSE + PLL + ペリフェラルクロック (pybricks の 3-8 に相当)
2. arm_fpuconfig()
3. stm32_lowsetup()
4. stm32_gpioinit()        ← GPIO クロックは stm32_clockconfig() で既に有効化済み
5. BSS クリア、.data コピー
6. stm32_boardinitialize() ← ★ PA13 設定 (pybricks の 9 に相当)
```

**差分**: NuttX では PA13 設定の前に FPU 設定、早期 UART 設定、BSS クリア等が追加で挟まる。ただし GPIO クロック (RCC_AHB1ENR_GPIOAEN) は `stm32_clockconfig()` 内で有効化されるため、`stm32_boardinitialize()` 時点で PA13 は設定可能。追加のステップは数十μs 程度であり、HSE 起動 + PLL ロック (~数ms) が支配的な pybricks と大きな差はない。

### タイミングの懸念

- pybricks と NuttX の PA13 設定タイミング差は数十μs〜数百μs 程度
- LEGO ブートローダーからジャンプ後、PA13 が駆動されるまでの合計時間は両者とも数ms (HSE+PLL が支配的)
- 通常はこの遅延で問題ないはず
- 問題が発生する場合は `stm32_clockconfig()` 内にボード固有のフックを追加して PA13 を早期に設定可能

### SWD デバッグとのトレードオフ

PA13 (SWDIO) と PA14 (SWCLK) を GPIO に転用すると SWD デバッグが不可能になる:
- PA14 は `PORT_3V3_EN` (I/O ポートへの 3.3V 電源) として使用
- USB 電源動作時はバッテリー電源維持が不要なため、デバッグビルドでは PA13/PA14 の再設定をスキップ可能
- 詳細は [13-debugging-strategy.md](13-debugging-strategy.md) を参照
