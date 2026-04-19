# SPIKE Prime Hub Peripheral Details

---

## TLC5955 LED Driver

### Overview

| Item | Value |
|---|---|
| Device | TLC5955 (Texas Instruments) |
| Channels | 48 (16 RGB groups x 3) |
| PWM resolution | 16 bit |
| Shift register length | 769 bit (97 bytes) |
| Connection | SPI1 + DMA2 |
| Max current setting | 3.2 mA (`TLC5955_MC_3_2`) |

### Communication Protocol

The TLC5955 communicates via a non-standard SPI protocol using a 769-bit shift register. After data transmission, the LAT pin (PA15) is toggled to latch data.

#### Control Data Latch (769 bit)

| Bit Position | Field | Description |
|---|---|---|
| 768 | Latch select | `1` = control data |
| 767-760 | Magic byte | `0x96` (identification) |
| 370 | LSDVLT | LED short detection voltage |
| 369 | ESPWM | ES-PWM mode (recommended enabled) |
| 368 | RFRESH | Auto refresh |
| 367 | TMGRST | Timing reset |
| 366 | DSPRPT | Auto display repeat (recommended enabled) |
| 365-345 | BC (3x7bit) | Global brightness control (Blue, Green, Red) |
| 344-336 | MC (3x3bit) | Maximum current setting (Blue, Green, Red) |
| 335-0 | DC (48x7bit) | Dot correction (per channel) |

#### Grayscale Data Latch (769 bit)

| Bit Position | Field | Description |
|---|---|---|
| 768 | Latch select | `0` = grayscale data |
| 767-0 | GS data | 48 channels x 16 bit = 768 bit |

Channel order: CH47 (MSB side) to CH0 (LSB side). Reverse mapping.

### Initialization Sequence

1. Transmit control data latch (97 bytes) via SPI DMA
2. Toggle LAT pin (PA15): HIGH to LOW
3. **Re-transmit** control data latch (required to confirm max current setting -- two transmissions needed)
4. Toggle LAT again
5. Grayscale update loop: on value change, transmit grayscale data + toggle LAT

SPIKE Hub settings: ES-PWM enabled, auto display repeat enabled, max current 3.2 mA.

### Channel Mapping

#### 5x5 LED Matrix (Single Color, 25 Channels)

| | Col 0 | Col 1 | Col 2 | Col 3 | Col 4 |
|---|---|---|---|---|---|
| Row 0 | CH38 | CH36 | CH41 | CH46 | CH33 |
| Row 1 | CH37 | CH28 | CH39 | CH47 | CH21 |
| Row 2 | CH24 | CH29 | CH31 | CH45 | CH23 |
| Row 3 | CH26 | CH27 | CH32 | CH34 | CH22 |
| Row 4 | CH25 | CH40 | CH30 | CH35 | CH9 |

#### Status LEDs (RGB, 12 Channels)

| LED | R | G | B |
|---|---|---|---|
| Status Top (above center button) | CH5 | CH4 | CH3 |
| Status Bottom (below center button) | CH8 | CH7 | CH6 |
| Battery indicator | CH2 | CH1 | CH0 |
| Bluetooth indicator | CH20 | CH19 | CH18 |

Color correction coefficients: R=1000, G=170, B=200.

#### Channel Usage Summary

- 25 channels: 5x5 matrix (single color)
- 12 channels: Status LEDs (4 LEDs x RGB)
- 11 channels: Unused (CH10-17, CH42-44)
- Total: 48 channels

---

## W25Q256 External SPI NOR Flash

### Overview

| Item | Value |
|---|---|
| Device | W25Q256 (Winbond) |
| Capacity | 32 MB (256 Mbit) |
| Interface | SPI (Standard/Dual/Quad) |
| Page size | 256 bytes |
| Sector size | 4 KB |
| Block size | 64 KB |
| Device ID | `0xEF` `0x40` `0x19` |
| Connection | SPI2 + DMA1 |
| CS | PB12 (Software NSS) |

### 4-Byte Addressing

The W25Q256 requires 4-byte addressing for its full 32 MB capacity. Pybricks uses dedicated 4-byte commands (robust, does not depend on the address mode register):

| Operation | 4-Byte Command | Notes |
|---|---|---|
| Fast Read | `0x0C` | `[0x0C] [addr3-0] [dummy] [data...]` |
| Page Program | `0x12` | 256 bytes per page |
| Sector Erase | `0x21` | 4 KB per sector |

### Flash Memory Layout

| Region | Address | Size | Usage |
|---|---|---|---|
| Bootloader data | `0x000000` - `0x07FFFF` | 512 KB | LEGO bootloader area |
| Pybricks block device | `0x080000` - `0x0BFFFF` | 256 KB | User program backup |
| Update key | `0x0FF000` - `0x0FFFFF` | 4 KB | mboot FS-load key |
| Filesystem | `0x100000` - `0x1FFFFFF` | 31 MB | FAT filesystem |

For NuttX, treat the first 1 MB as reserved and use 1 MB onward for the filesystem.

### NuttX Driver Support

NuttX upstream `w25.c` (MTD_W25) supports 3-byte addressing only (up to W25Q128) and does not support W25Q256. `w25qxxxjv.c` is QSPI-only and does not support standard SPI.

**Implemented**: `boards/spike-prime-hub/src/stm32_w25q256.c` provides a board-local dedicated MTD driver. It always uses the 4-byte-address opcodes (Fast Read `0x0C` / Page Program `0x12` / Sector Erase `0x21`), so it does not depend on the device's address-mode register state (matches pybricks). The first 1 MB is reserved; the 31 MB region from `0x100000` is exposed via `mtd_partition()` as `/dev/mtdblock0` and mounted as LittleFS at `/mnt/flash`. See the [W25Q256 driver](../drivers/w25q256.md) document for details.

---

## LSM6DS3TR-C IMU (6-Axis Inertial Measurement Unit)

### Overview

| Item | Value |
|---|---|
| Device | LSM6DS3TR-C (STMicroelectronics) |
| Function | 3-axis accelerometer + 3-axis gyroscope |
| Connection | I2C2 (SCL=PB10, SDA=PB3) |
| I2C address | `0x6A` |

### Register Configuration

| Register | Setting | Description |
|---|---|---|
| CTRL3_C | BDU + IF_INC | Block Data Update enabled + address auto-increment enabled |
| INT1_CTRL | DRDY | Data Ready interrupt output to INT1 |

### Data Reading

12-byte burst read to obtain accelerometer (3 axes x 2 bytes) + gyroscope (3 axes x 2 bytes) in a single transaction.

### NuttX Driver Support

The existing `lsm6dsl.c` (`CONFIG_SENSORS_LSM6DSL`) can be used as a substitute. The LSM6DS3TR-C is fundamentally register-compatible with the LSM6DSL. A minor modification is needed to handle the difference in WHO_AM_I register values.

---

## CC2564C Bluetooth

### Overview

| Item | Value |
|---|---|
| Device | CC2564C (Texas Instruments) |
| Connection | USART2 (TX=PD5, RX=PD6, CTS=PD3, RTS=PD4) |
| Flow control | RTS/CTS hardware flow control required |

### Status

Not targeted during initial bring-up. Requires firmware patch loading (~7 KB) at boot and HCI UART Bluetooth stack integration. NuttX has a foundation with `WIRELESS_BLUETOOTH` + `BLUETOOTH_UART`, and CC2564-specific initialization sequences exist.

---

## ADC (Battery Monitoring)

### Overview

| Item | Value |
|---|---|
| Peripheral | ADC1 |
| DMA | DMA2 |
| Channels | 6 channels (battery monitoring) |

Multi-channel scan mode + DMA for battery voltage monitoring. Channel list and sample times are configured in board-level code.

---

## DAC (Speaker Output)

### Overview

| Item | Value |
|---|---|
| Peripheral | DAC1 |
| Output pin | PA4 (DAC_OUT1, fixed) |

Supports timer-triggered DMA for continuous conversion. Can be used for audio waveform generation.
