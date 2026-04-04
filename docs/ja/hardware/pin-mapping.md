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
| E | UART10 | PE3 | PE2 | AF11 | PB5 | F413 固有、NuttX 未対応 |
| F | UART9 | PD15 | PD14 | AF11 | PC5 | F413 固有、NuttX 未対応 |

`uart_buf` はバッファ有効化ピン (Active Low)。TX/RX ピンと GPIO1/GPIO2 は同じ物理ピン (Pin 5/Pin 6) にバッファ経由で接続。

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

PWM 周波数: 12 kHz (prescaler=8, period=1000)。反転 PWM 設定。M1/M2 は GPIO/AF モード動的切替で Coast/Brake/Forward/Reverse を実現。

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

| 信号 | ピン | 備考 |
|---|---|---|
| TX | PD5 | USART2 |
| RX | PD6 | USART2 |
| CTS | PD3 | ハードウェアフロー制御必須 |
| RTS | PD4 | ハードウェアフロー制御必須 |

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
| TIM12 | TLC5955 GSCLK | CH2 | 96 MHz → 9.6 MHz 出力 |
