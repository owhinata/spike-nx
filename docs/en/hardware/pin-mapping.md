# SPIKE Prime Hub Pin Mapping

A complete peripheral pin assignment reference for the SPIKE Prime Hub (STM32F413VG).

---

## I/O Ports (A-F)

Each port has three signal groups: UART (device communication), GPIO (device detection), and PWM (motor control).

### UART (Device Communication)

| Port | UART | TX Pin | RX Pin | AF | uart_buf | Notes |
|---|---|---|---|---|---|---|
| A | UART7 | PE8 | PE7 | AF8 | PA10 | |
| B | UART4 | PD1 | PD0 | AF11 | PA8 | |
| C | UART8 | PE1 | PE0 | AF8 | PE5 | |
| D | UART5 | PC12 | PD2 | AF8 | PB2 | |
| E | UART10 | PE3 | PE2 | AF11 | PB5 | F413-specific, not supported in NuttX (planned for #43) |
| F | UART9 | PD15 | PD14 | AF11 | PC5 | F413-specific, not supported in NuttX (planned for #43) |

`uart_buf` is the buffer enable pin (Active Low). TX/RX pins and GPIO1/GPIO2 share the same physical pins (Pin 5/Pin 6) connected through a buffer.

!!! warning "Pin caveats handled by Issue #42"
    - **PB2 (Port D `uart_buf`)** is also the STM32F4 **BOOT1 strap pin**.  The HW board must hold the correct level during reset (software cannot fix that).  After reset the pin can be reconfigured as GPIO output.
    - **PC13/PC14/PC15 (Port D gpio1/gpio2 + Port E gpio1)** are STM32 backup-domain I/O.  Per RM0430 §6.2.4 they must use `GPIO_SPEED_2MHz` only, must only be used for ID-resistor sensing (no real load drive), and `LSEON` must remain 0.
    - **PA10 (Port A `uart_buf`)** doubles as USB OTG_FS_ID.  The defconfig sets `CONFIG_OTG_ID_GPIO_DISABLE=y` so NuttX's upstream OTG init does not overwrite the legoport configuration on PA10 (required for #42).

### GPIO (Device Detection)

| Port | gpio1 (Pin 5) | gpio2 (Pin 6) |
|---|---|---|
| A | PD7 | PD8 |
| B | PD9 | PD10 |
| C | PD11 | PE4 |
| D | PC15 | PC14 |
| E | PC13 | PE12 |
| F | PC11 | PE6 |

gpio1/gpio2 are used for resistor-based passive detection. Pull-up/pull-down must be disabled for accurate resistance detection.

### Motor PWM (H-Bridge Control)

| Port | M1 Pin | M1 Timer/Ch | M2 Pin | M2 Timer/Ch | AF |
|---|---|---|---|---|---|
| A | PE9 | TIM1 CH1 | PE11 | TIM1 CH2 | AF1 |
| B | PE13 | TIM1 CH3 | PE14 | TIM1 CH4 | AF1 |
| C | PB6 | TIM4 CH1 | PB7 | TIM4 CH2 | AF2 |
| D | PB8 | TIM4 CH3 | PB9 | TIM4 CH4 | AF2 |
| E | PC6 | TIM3 CH1 | PC7 | TIM3 CH2 | AF2 |
| F | PC8 | TIM3 CH3 | PB1 | TIM3 CH4 | AF2 |

PWM frequency: 12 kHz (prescaler=8, period=1000). Inverted PWM configuration. M1/M2 dynamically switch between GPIO/AF modes to achieve Coast/Brake/Forward/Reverse.

---

## SPI1: TLC5955 LED Driver

| Signal | Pin | Notes |
|---|---|---|
| MOSI (SIN) | SPI1 MOSI | DMA2 Stream3 (TX, ch3) |
| SCK (SCLK) | SPI1 SCK | 24 MHz (APB2 96 MHz / 4) |
| LAT | PA15 (GPIO) | Latch signal (GPIO output) |
| GSCLK | TIM12 CH2 | 9.6 MHz (96 MHz / prescaler 1 / period 10) |

SPI mode: CPOL=0, CPHA=0 (Mode 0), MSB first, 8-bit. DMA2 Stream2 (RX, ch3) also used.

---

## SPI2: W25Q256 External Flash

| Signal | Pin | Notes |
|---|---|---|
| MOSI | SPI2 MOSI | DMA1 Stream4 (TX, ch0) |
| MISO | SPI2 MISO | DMA1 Stream3 (RX, ch0) |
| SCK | SPI2 SCK | APB1 clock / 2 |
| CS | PB12 | Software NSS (Active Low) |

SPI mode: CPOL=0, CPHA=0 (Mode 0), MSB first.

---

## I2C2: IMU (LSM6DS3TR-C)

| Signal | Pin | AF | Notes |
|---|---|---|---|
| SCL | PB10 | AF4 | Standard mapping |
| SDA | PB3 | AF9 | F413-specific non-standard mapping. Requires correct definition in pinmap header |

Device address: **0x6A**

---

## USB OTG FS

| Signal | Pin | Notes |
|---|---|---|
| DM | PA11 | Standard mapping, no remapping needed |
| DP | PA12 | Standard mapping, no remapping needed |

---

## Bluetooth (CC2564C)

| Signal | Pin | AF | Notes |
|---|---|---|---|
| TX | PD5 | AF7 | USART2 TX |
| RX | PD6 | AF7 | USART2 RX |
| CTS | PD3 | AF7 | Hardware flow control required |
| RTS | PD4 | AF7 | Hardware flow control required |
| nSHUTD | PA2 | GPIO | CC2564C chip enable (active HIGH, driven LOW at boot to hold reset) |
| SLOWCLK | PC9 | AF3 | TIM8 CH4 -> 32.768 kHz 50% duty (sleep clock, stable before nSHUTD HIGH) |

DMA: TX = DMA1 Stream 6 Channel 4, RX = DMA1 Stream 7 Channel 6 (RM0430 Table 30; F413-specific multiplexed mapping #2 dedicated to Bluetooth). NVIC priority 0xA0 (Issue #50 reserved slot).

---

## Power Control

| Pin | Function | Notes |
|---|---|---|
| PA13 | BAT_PWR_EN | Battery power hold. Must be driven HIGH immediately after boot. Shared with SWD (SWDIO) |
| PA14 | PORT_3V3_EN | I/O port 3.3V power control. Shared with SWD (SWCLK) |

---

## Analog

| Pin | Function | Notes |
|---|---|---|
| PA4 | DAC_OUT1 | Speaker output. Supports timer-triggered DMA (audio waveform generation) |
| ADC1 | Battery monitoring | Multi-channel scan + DMA (6 channels) |

---

## Timer Assignment Summary

| Timer | Usage | Channels | Clock |
|---|---|---|---|
| TIM1 | Motors (Port A, B) | CH1-CH4 | 96 MHz (APB2) |
| TIM3 | Motors (Port E, F) | CH1-CH4 | 96 MHz (APB1x2) |
| TIM4 | Motors (Port C, D) | CH1-CH4 | 96 MHz (APB1x2) |
| TIM8 | Bluetooth 32.768 kHz slow clock | CH4 -> PC9 | 96 MHz (APB2) -> 32.764 kHz output |
| TIM12 | TLC5955 GSCLK | CH2 | 96 MHz -> 9.6 MHz output |
