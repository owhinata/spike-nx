# TLC5955 LED Driver Protocol Details

## 1. Overview

TLC5955 is a TI 48-channel 16-bit PWM LED driver communicating via SPI-like shift register protocol.

| Item | Value |
|---|---|
| Channels | 48 (16 RGB groups × 3) |
| PWM resolution | 16 bit |
| Shift register length | 769 bit (97 bytes) |
| Communication | SPI (SCLK + SIN) + LAT (latch) |
| PWM clock | GSCLK (external input) |

---

## 2. SPIKE Hub Connection

| Signal | Connected To | Notes |
|---|---|---|
| SPI | SPI1 | DMA2 Stream2 (RX, ch3) / Stream3 (TX, ch3) |
| LAT | PA15 (GPIO) | Latch signal |
| SPI mode | CPOL=0, CPHA=0 (Mode 0) | MSB first, 8-bit, prescaler /4 |

---

## 3. Shift Register Format (769 bit)

### Control Data Latch

| Bit Position | Field | Description |
|---|---|---|
| 768 | Latch select | `1` = control data |
| 767-760 | Magic byte | `0x96` (identification) |
| 370 | LSDVLT | LED short detection voltage |
| 369 | ESPWM | ES-PWM mode (recommended enabled) |
| 368 | RFRESH | Auto refresh |
| 367 | TMGRST | Timing reset |
| 366 | DSPRPT | Auto display repeat (recommended enabled) |
| 365-345 | BC (3×7bit) | Global brightness (Blue, Green, Red) |
| 344-336 | MC (3×3bit) | Maximum current (Blue, Green, Red) |
| 335-0 | DC (48×7bit) | Dot correction (per channel) |

### Grayscale Data Latch

| Bit Position | Field | Description |
|---|---|---|
| 768 | Latch select | `0` = grayscale data |
| 767-0 | GS data | 48 channels × 16 bit = 768 bit |

Channel order: CH47 (MSB side) → CH0 (LSB side). Reverse mapping.

---

## 4. Initialization Sequence

From pybricks (`pwm_tlc5955_stm32.c`):

1. Transmit control data latch (97 bytes) via SPI DMA
2. Toggle LAT pin (PA15): HIGH → LOW
3. **Re-transmit** control data latch (required for max current to take effect)
4. Toggle LAT again
5. Grayscale update loop: on value change, transmit grayscale data + toggle LAT

### SPIKE Hub Settings

```c
Max current: 3.2 mA (TLC5955_MC_3_2)
ES-PWM: enabled
Auto display repeat: enabled
```

---

## 5. Channel → LED Mapping

### 5×5 LED Matrix (TLC5955 channel numbers)

| | Col 0 | Col 1 | Col 2 | Col 3 | Col 4 |
|---|---|---|---|---|---|
| Row 0 | CH38 | CH36 | CH41 | CH46 | CH33 |
| Row 1 | CH37 | CH28 | CH39 | CH47 | CH21 |
| Row 2 | CH24 | CH29 | CH31 | CH45 | CH23 |
| Row 3 | CH26 | CH27 | CH32 | CH34 | CH22 |
| Row 4 | CH25 | CH40 | CH30 | CH35 | CH9 |

5×5 matrix is **single color** (1 channel per pixel). 25 channels used.

### Status LEDs (RGB)

| LED | R | G | B |
|---|---|---|---|
| Status Top (center button upper) | CH5 | CH4 | CH3 |
| Status Bottom (center button lower) | CH8 | CH7 | CH6 |
| Battery indicator | CH2 | CH1 | CH0 |
| Bluetooth indicator | CH20 | CH19 | CH18 |

4 RGB LEDs × 3 channels = 12 channels. Color correction: R=1000, G=170, B=200.

### Unused Channels

CH10-17, CH42-44 (11 channels unused)

### Total

25 (matrix) + 12 (status) + 11 (unused) = 48 channels

---

## 6. NuttX Driver Design

### Required Components

1. **SPI1 master driver** + DMA (NuttX standard driver)
2. **GPIO control** (PA15 = LAT pin)
3. **Custom TLC5955 driver** (character device)

### Driver Interface

```
/dev/ledmatrix    — 5×5 matrix control
/dev/statusled    — Status LED control
```

Or extend NuttX USERLED framework.

### Priority

TLC5955 is deferred during initial bring-up. First check if any LEDs are directly GPIO-controllable (likely none — all LEDs are TLC5955-driven).
