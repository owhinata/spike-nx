# SPIKE Prime Hub ハードウェア概要

## MCU 仕様

| 項目 | 値 |
|---|---|
| MCU | STM32F413VG (ARM Cortex-M4F) |
| SYSCLK | 96 MHz (16 MHz HSE + PLL) |
| Flash | 1 MB (公称)。物理的に 1.5 MB 存在する可能性あり (要実機確認) |
| RAM | 320 KB (SRAM1 256 KB + SRAM2 64 KB) |
| パッケージ | LQFP100 |

> **注**: STM32F413H-Discovery Kit は STM32F413**ZH** (1.5 MB Flash)。SPIKE Hub は STM32F413**VG** (公称 1 MB)。Bank 2 (512 KB) が利用可能かは要実機確認。

---

## フラッシュレイアウト

### メモリマップ

```
0x08000000 ┌─────────────────────┐
           │ LEGO ブートローダー  │ 32 KB (セクタ 0-1, 書換不可)
0x08008000 ├─────────────────────┤
           │                     │
           │   ファームウェア     │ 992 KB (セクタ 2-11)
           │                     │
0x08100000 ├─────────────────────┤
           │   Bank 2 追加領域    │ 512 KB (セクタ 12-15, VG では利用不可の可能性)
0x08180000 └─────────────────────┘

0x20000000 ┌─────────────────────┐
           │   SRAM1             │ 256 KB
0x20040000 ├─────────────────────┤
           │   SRAM2             │ 64 KB
0x20050000 └─────────────────────┘
```

### Bank 1 セクタ詳細 (1 MB)

| セクタ | アドレス | サイズ | 用途 |
|--------|---------|-------|------|
| 0 | `0x08000000` | 16 KB | LEGO ブートローダー |
| 1 | `0x08004000` | 16 KB | LEGO ブートローダー |
| 2 | `0x08008000` | 16 KB | ファームウェア開始 |
| 3 | `0x0800C000` | 16 KB | ファームウェア |
| 4 | `0x08010000` | 64 KB | ファームウェア |
| 5 | `0x08020000` | 128 KB | ファームウェア |
| 6-11 | `0x08040000` - `0x080FFFFF` | 各 128 KB | ファームウェア |

出典: RM0430 (STM32F413/423 リファレンスマニュアル) Table 5

---

## 電源管理

### BAT_PWR_EN (PA13)

バッテリー駆動時、PA13 を HIGH に設定しないとハブの電源が落ちる。

| ピン | 機能 | 備考 |
|------|------|------|
| PA13 | BAT_PWR_EN | バッテリー電源維持。起動直後に HIGH 駆動が必須 |
| PA14 | PORT_3V3_EN | I/O ポートへの 3.3V 電源制御 |

PA13 はデフォルトで JTMS-SWDIO (SWD デバッグ) ピンと共用。GPIO 出力に設定すると SWD デバッグが不可能になる。USB 電源動作時はバッテリー電源維持が不要なため、デバッグビルドでは PA13/PA14 の再設定をスキップ可能。

### NuttX での PA13 設定

`stm32_boardinitialize()` で設定する:

```c
void stm32_boardinitialize(void)
{
    // バッテリー電源維持 (PA13 = BAT_PWR_EN)
    stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ |
                     GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN13);
}
```

`stm32_clockconfig()` で GPIO クロックが有効化済みのため、この時点で PA13 は設定可能。pybricks との PA13 設定タイミング差は数十us〜数百us 程度で問題にならない (HSE 起動 + PLL ロックの数ms が支配的)。

---

## ブートシーケンス

### LEGO DFU ブートローダー → NuttX 起動

```
LEGO DFU ブートローダー (0x08000000)
  │
  ├─ DFU モード判定 (BT ボタン長押し → DFU モード)
  │
  └─ ファームウェアへジャンプ (0x08008000)
       │
       NuttX __start()
         ├─ stm32_clockconfig()     ← HSE + PLL 設定 (16MHz → 96MHz), ペリフェラルクロック有効化
         ├─ arm_fpuconfig()         ← FPU 有効化
         ├─ stm32_lowsetup()        ← 早期 UART 設定
         ├─ stm32_gpioinit()        ← GPIO クロック有効化 (clockconfig で済み)
         ├─ BSS クリア、.data コピー
         ├─ stm32_boardinitialize() ← ★ PA13 (BAT_PWR_EN) = HIGH
         └─ nx_start()              ← OS カーネル起動
```

### VTOR 設定

NuttX はリンカスクリプトで `FLASH ORIGIN = 0x08008000` を設定するだけで VTOR が自動的に正しく設定される。`.vectors` セクションが FLASH 先頭に配置され、`stm32_irq.c` が `NVIC_VECTAB = 0x08008000` を書き込む。

### クロック設定

| パラメータ | 値 |
|---|---|
| HSE | 16 MHz |
| PLL | 16 MHz → 96 MHz |
| AHB | /1 (96 MHz) |
| APB1 | /2 (48 MHz) |
| APB2 | /1 (96 MHz) |
| Flash wait state | 5 |

---

## DFU ブートローダー

### 基本情報

| 項目 | 値 |
|---|---|
| VID:PID | `0694:0008` |
| プロトコル | STMicroelectronics DfuSe 互換 |
| 保護領域 | `0x08000000` - `0x08007FFF` (書換不可) |

### DFU モード進入方法

1. USB ケーブルを抜く
2. Bluetooth ボタンを長押し
3. ボタンを押したまま USB を接続
4. 5 秒後にボタンを離す → バッテリー LED が色を切り替え

### ファームウェア書込み

```bash
# デバイス確認
dfu-util -l
# → [0694:0008] Internal Flash

# ファームウェア書込み
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx.bin
```

### 制限事項

- プログラムからの DFU モード進入不可 (物理ボタン操作が必要)
- バッテリー駆動時、DFU ブートローダーは PA13 (BAT_PWR_EN) を HIGH にしない → USB 電源が必要
- `0x08000000` - `0x08007FFF` の領域は書込み保護
