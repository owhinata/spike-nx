# SPIKE Prime Hub ピンマッピング

SPIKE Prime Hub (STM32F413VG) の全ペリフェラルピンアサインをまとめる。

---

## I/O ポート (A-F)

各ポートには UART (デバイス通信)、GPIO (デバイス検出)、PWM (モーター制御) の 3 系統が接続されている。

### UART (デバイス通信)

| ポート | UART | TX ピン | RX ピン | AF | uart_buf | 備考 |
|---|---|---|---|---|---|---|
| A | UART7 | PE8 | PE7 | AF8 | PA10 | |
| B | UART4 | PD1 | PD0 | AF11 | PA8 | |
| C | UART8 | PE1 | PE0 | AF8 | PE5 | |
| D | UART5 | PC12 | PD2 | AF8 | PB2 | |
| E | UART10 | PE3 | PE2 | AF11 | PB5 | F413 固有、NuttX 未対応 (#43 で対応予定) |
| F | UART9 | PD15 | PD14 | AF11 | PC5 | F413 固有、NuttX 未対応 (#43 で対応予定) |

`uart_buf` はバッファ有効化ピン (Active Low)。TX/RX ピンと GPIO1/GPIO2 は同じ物理ピン (Pin 5/Pin 6) にバッファ経由で接続。

!!! warning "Issue #42 で抑える pin caveat"
    - **PB2 (Port D `uart_buf`)** は STM32F4 の **BOOT1 strap pin**。reset 中は HW ボード側が正しいレベルを保持する必要がある (ソフトでは修正不可)。reset 後は GPIO 出力として再構成可能。
    - **PC13/PC14/PC15 (Port D gpio1/gpio2 + Port E gpio1)** は STM32 backup-domain I/O。RM0430 §6.2.4 に従い `GPIO_SPEED_2MHz` 固定、ID 抵抗 sense 用途のみ (出力負荷駆動 NG)、`LSEON` は常に 0 を維持。
    - **PA10 (Port A `uart_buf`)** は USB OTG_FS_ID 兼用。`CONFIG_OTG_ID_GPIO_DISABLE=y` を defconfig で設定して NuttX upstream OTG init が PA10 を上書きしないようにしている (#42 必須)。

### GPIO (デバイス検出)

| ポート | gpio1 (Pin 5) | gpio2 (Pin 6) |
|---|---|---|
| A | PD7 | PD8 |
| B | PD9 | PD10 |
| C | PD11 | PE4 |
| D | PC15 | PC14 |
| E | PC13 | PE12 |
| F | PC11 | PE6 |

gpio1/gpio2 は抵抗ベースのパッシブ検出で使用。プルアップ/プルダウンは無効化して正確な抵抗値を検出する。

### モーター PWM (H-Bridge 制御)

| ポート | M1 ピン | M1 Timer/Ch | M2 ピン | M2 Timer/Ch | AF |
|---|---|---|---|---|---|
| A | PE9 | TIM1 CH1 | PE11 | TIM1 CH2 | AF1 |
| B | PE13 | TIM1 CH3 | PE14 | TIM1 CH4 | AF1 |
| C | PB6 | TIM4 CH1 | PB7 | TIM4 CH2 | AF2 |
| D | PB8 | TIM4 CH3 | PB9 | TIM4 CH4 | AF2 |
| E | PC6 | TIM3 CH1 | PC7 | TIM3 CH2 | AF2 |
| F | PC8 | TIM3 CH3 | PB1 | TIM3 CH4 | AF2 |

PWM 周波数: 12 kHz (prescaler=8, period=1000)。反転 PWM (CCxP=1) 設定。M1/M2 は GPIO/AF モード動的切替で Coast/Brake/Forward/Reverse を実現。Issue #80 (`/dev/legoport_pwm[0..5]`) で実装済。

!!! warning "Port E (PC6/PC7) と USART6 の排他"
    Port E は USART6 と物理ピンを共有する。`CONFIG_BOARD_LEGOPORT_PWM=y` の時は USART6 を disable しなければならず (Kconfig で強制)、emergency / early / panic debug の物理経路が失われる (SPIKE Prime Hub は SWD も電源制御転用で使用不可)。詳細は `drivers/legoport-pwm.md` 参照。

---

## SPI1: TLC5955 LED ドライバ

| 信号 | ピン | 備考 |
|---|---|---|
| MOSI (SIN) | SPI1 MOSI | DMA2 Stream3 (TX, ch3) |
| SCK (SCLK) | SPI1 SCK | 24 MHz (APB2 96 MHz / 4) |
| LAT | PA15 (GPIO) | ラッチ信号 (GPIO 出力) |
| GSCLK | TIM12 CH2 | 9.6 MHz (96 MHz / prescaler 1 / period 10) |

SPI モード: CPOL=0, CPHA=0 (Mode 0), MSB first, 8bit。DMA2 Stream2 (RX, ch3) も使用。

---

## SPI2: W25Q256 外部 Flash

| 信号 | ピン | 備考 |
|---|---|---|
| MOSI | SPI2 MOSI | DMA1 Stream4 (TX, ch0) |
| MISO | SPI2 MISO | DMA1 Stream3 (RX, ch0) |
| SCK | SPI2 SCK | APB1 クロック / 2 |
| CS | PB12 | ソフトウェア NSS (Active Low) |

SPI モード: CPOL=0, CPHA=0 (Mode 0), MSB first。

---

## I2C2: IMU (LSM6DS3TR-C)

| 信号 | ピン | AF | 備考 |
|---|---|---|---|
| SCL | PB10 | AF4 | 標準マッピング |
| SDA | PB3 | AF9 | F413 固有の非標準マッピング。ピンマップヘッダで正しく定義が必要 |

デバイスアドレス: **0x6A**

---

## USB OTG FS

| 信号 | ピン | 備考 |
|---|---|---|
| DM | PA11 | 標準マッピング、リマップ不要 |
| DP | PA12 | 標準マッピング、リマップ不要 |

---

## Bluetooth (CC2564C)

| 信号 | ピン | AF | 備考 |
|---|---|---|---|
| TX | PD5 | AF7 | USART2 TX |
| RX | PD6 | AF7 | USART2 RX |
| CTS | PD3 | AF7 | ハードウェアフロー制御必須 |
| RTS | PD4 | AF7 | ハードウェアフロー制御必須 |
| nSHUTD | PA2 | GPIO | CC2564C chip enable (Active HIGH、初期 LOW で reset 保持) |
| SLOWCLK | PC9 | AF3 | TIM8 CH4 → 32.768 kHz 50% duty (sleep clock、nSHUTD HIGH 前に安定化) |

DMA: TX = DMA1 Stream 6 Channel 4, RX = DMA1 Stream 7 Channel 6 (RM0430 Table 30、F413 固有の多重マッピング #2 を BT 専用に使用)。NVIC 優先度 0xA0 (Issue #50 予約枠)。

---

## 電源制御

| ピン | 機能 | 備考 |
|---|---|---|
| PA13 | BAT_PWR_EN | バッテリー電源維持。起動直後に HIGH 駆動必須。SWD (SWDIO) と共用 |
| PA14 | PORT_3V3_EN | I/O ポート 3.3V 電源制御。SWD (SWCLK) と共用 |

---

## アナログ

| ピン | 機能 | 備考 |
|---|---|---|
| PA4 | DAC_OUT1 | スピーカー出力。タイマートリガー DMA 対応 (音声波形生成) |
| ADC1 | バッテリー監視 | マルチチャンネルスキャン + DMA (6ch) |

---

## タイマー割当一覧

| タイマー | 用途 | チャンネル | クロック |
|---|---|---|---|
| TIM1 | モーター (ポート A, B) | CH1-CH4 | 96 MHz (APB2) |
| TIM3 | モーター (ポート E, F) | CH1-CH4 | 96 MHz (APB1x2) |
| TIM4 | モーター (ポート C, D) | CH1-CH4 | 96 MHz (APB1x2) |
| TIM8 | Bluetooth 32.768 kHz slow clock | CH4 → PC9 | 96 MHz (APB2) → 32.764 kHz 出力 |
| TIM12 | TLC5955 GSCLK | CH2 | 96 MHz → 9.6 MHz 出力 |
