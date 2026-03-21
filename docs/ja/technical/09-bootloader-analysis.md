# ブートローダー分析

## 1. 3段階ブートアーキテクチャ

SPIKE Prime Hub は 3 段階のブート構成を持つ:

| 段階 | アドレス範囲 | サイズ | 説明 |
|---|---|---|---|
| LEGO 内蔵ブートローダー | `0x08000000` - `0x08007FFF` | 32 KB | 書換不可の DFU ブートローダー |
| mboot (二次ブートローダー) | `0x08008000` - `0x0800FFFF` | 32 KB | MicroPython の mboot |
| アプリケーション | `0x08010000` - `0x080FFFFF` | 960 KB | メインファームウェア |

### pybricks の場合

pybricks は mboot を使わず、`0x08008000` から直接ファームウェアを配置 (992 KB 利用可能):

```
FLASH_BOOTLOADER : ORIGIN = 0x08000000, LENGTH = 32K
FLASH_FIRMWARE   : ORIGIN = 0x08008000, LENGTH = 992K
```

### NuttX の選択肢

| 方式 | ファームウェア開始 | 利用可能サイズ | メリット |
|---|---|---|---|
| **A: mboot なし** | `0x08008000` | 992 KB + 512 KB | 最大領域。pybricks と同じ方式 |
| B: mboot あり | `0x08010000` | 960 KB + 512 KB | OTA 更新が容易 |

**推奨**: 方式 A (`0x08008000`)。初期段階では mboot 不要。

---

## 2. LEGO 内蔵ブートローダー (DFU)

### 進入方法

1. USB ケーブルを抜く
2. Bluetooth ボタンを長押し
3. ボタンを押したまま USB を接続
4. 5 秒後にボタンを離す → バッテリー LED が色を切り替え

### プロトコル

**STMicroelectronics DfuSe 互換**。標準の `dfu-util` で操作可能。

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

---

## 3. mboot ファームウェア更新メカニズム

MicroPython の mboot は SPI Flash 経由の OTA 更新をサポート:

1. 更新キー (`0x12345678`) を SPI Flash のアドレス `1020 * 1024` に書込み
2. 次回起動時に mboot がキーを検出
3. SPI Flash ファイルシステムからファームウェアを読み出し、内蔵フラッシュに書込み
4. 電源断に対して安全 (中断時は再起動で更新再開)
5. 成功後にキーを消去

NuttX では初期段階ではこの仕組みは不要。将来的に独自の OTA 更新を実装可能。

---

## 4. SystemInit() コードフロー分析

`pybricks/lib/pbio/platform/prime_hub/platform.c` の SystemInit() (999-1050 行):

```
Reset_Handler (startup.s)
  ├─ SP 設定 (_estack)
  ├─ .data コピー (Flash → SRAM)
  ├─ .bss ゼロクリア
  └─ SystemInit() 呼出し
       ├─ SCB->CCR: 8byte スタックアラインメント
       ├─ SCB->VTOR: ベクタテーブル再配置
       ├─ RCC: HSE 有効化 + PLL 設定 (16MHz → 96MHz)
       ├─ FLASH: wait state = 5
       ├─ RCC: SYSCLK = PLL, AHB/1, APB1/2, APB2/1
       ├─ RCC: 全ペリフェラルクロック有効化
       │   (GPIOA-E, DMA1-2, USART2, UART4/5/7-10,
       │    TIM1-8/12, I2C2, DAC, SPI1-2, ADC1, OTG_FS, SYSCFG)
       └─ PA13: BAT_PWR_EN = HIGH (電源維持)
```

### PA13 設定までのクロック数見積もり

SystemInit() の PA13 設定は以下の後に実行:
1. スタックアラインメント設定 (~数クロック)
2. VTOR 設定 (~数クロック)
3. HSE 起動待ち (~数百μs 〜数ms)
4. PLL ロック待ち (~数百μs)
5. FLASH wait state 設定
6. バスクロック設定
7. 全ペリフェラルクロック有効化 (~数十クロック)

**HSE 起動 + PLL ロックが支配的で、合計 ~数ms**。この間 CPU は HSI (16 MHz 内蔵 RC) で動作。LEGO ブートローダーからジャンプした時点ではクロックが既に設定されている可能性もあるが、pybricks は再初期化している。

---

## 5. NuttX ブートシーケンスとの対応

| pybricks | NuttX | 備考 |
|---|---|---|
| Reset_Handler | `__start()` | エントリポイント |
| .data コピー / .bss クリア | `__start()` 内 | NuttX も同様 |
| SystemInit() → クロック設定 | `stm32_clockconfig()` | |
| SystemInit() → VTOR 設定 | `stm32_irq.c` → NVIC_VECTAB | リンカスクリプトで自動 |
| SystemInit() → PA13 HIGH | `stm32_boardinitialize()` | ここが最も適切 |
| main() | `nx_start()` → `board_late_initialize()` | |
