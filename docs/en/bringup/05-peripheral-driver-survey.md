# Peripheral Driver Survey

## Summary

| # | Peripheral | NuttX Driver | Kconfig | Status | Effort |
|---|---|---|---|---|---|
| 1 | USB OTG FS (CDC/ACM) | `stm32_otgfsdev.c` | `STM32_OTGFS`, `CDCACM` | Mature, well-tested on F4 | Low |
| 2 | UART (USART1-3,6, UART4-8) | `stm32_serial.c` | `STM32_USARTn` / `STM32_UARTn` | UART9/10 NOT supported | Low (except 9/10) |
| 3 | SPI (SPI1, SPI2) | `stm32_spi.c` | `STM32_SPI1`, `STM32_SPI2` | Mature, DMA supported | Low |
| 4 | ADC + DMA | `stm32_adc.c` | `STM32_ADC1`, `ADC` | Multi-channel scan + DMA | Low-Medium |
| 5 | I2C (I2C2) | `stm32_i2c.c` | `STM32_I2C2` | PB3 AF9 non-standard pin is a challenge | Medium |
| 6 | Timer/PWM | `stm32_pwm.c` | `STM32_TIMn`, `PWM` | TIM1/3/4 all supported | Low |
| 7 | DAC | `stm32_dac.c` | `STM32_DAC1`, `DAC` | PA4 standard mapping | Low |
| 8 | TLC5955 LED | **None** | — | Custom driver required | High |
| 9 | W25Q256 Flash | **Partial** | `MTD_W25` (SPI) | No 32-bit addressing. Driver extension needed | Medium |
| 10 | LSM6DS3TR-C IMU | **Partial** | `SENSORS_LSM6DSL` | LSM6DSL driver usable with minor mods | Medium |
| 11 | Bluetooth CC2564C | Yes | `WIRELESS_BLUETOOTH`, `BLUETOOTH_UART` | FW patch loading needed. Defer to later | Medium-High |

---

## Details

### 1. USB OTG FS (CDC/ACM) — HIGH

**Driver**: `arch/arm/src/stm32/stm32_otgfsdev.c` — mature on STM32F4.

**Required Kconfig**:
```
CONFIG_STM32_OTGFS=y
CONFIG_USBDEV=y
CONFIG_CDCACM=y
CONFIG_CDCACM_CONSOLE=y
CONFIG_BOARDCTL_USBDEVCTRL=y
CONFIG_NSH_USBCONSOLE=y
CONFIG_NSH_USBCONDEV="/dev/ttyACM0"
```

**Pins**: PA11 (DM), PA12 (DP) — standard mapping, no remapping needed.

**Proven boards**: stm32f4discovery:usbnsh, stm32f411-minimum, nucleo-f446re

---

### 2. UART — HIGH

| Peripheral | Supported | Notes |
|---|---|---|
| USART1, 2, 3, 6 | Yes | Common to all F4 |
| UART4, 5 | Yes | Enabled for F427/F429/F469 |
| UART7, 8 | Yes | IRQ, RCC, driver all implemented |
| **UART9** | **No** | F413-specific. IRQ/RCC/driver all missing |
| **UART10** | **No** | Same as above |

**Note**: UART9/10 are clocked from APB2. ST HAL had a baud rate calculation bug using APB1 clock. Custom driver must use the correct clock source.

SPIKE Hub port mapping: A=UART7, B=UART4, C=UART8, D=UART5, E=UART10, F=UART9

---

### 3. SPI — MEDIUM

**Driver**: `arch/arm/src/stm32/stm32_spi.c` — supports SPI1-5.

**Kconfig**:
```
CONFIG_SPI=y
CONFIG_STM32_SPI1=y          # TLC5955 LED controller
CONFIG_STM32_SPI2=y          # W25Q256 Flash
CONFIG_STM32_SPI1_DMA=y      # Optional: DMA transfers
CONFIG_STM32_SPI2_DMA=y
```

**DMA**: Fully supported. Auto-falls back to PIO if buffer is not in DMA-capable memory.

**Limitation**: `CONFIG_STM32_SPI_INTERRUPT` and `CONFIG_STM32_SPIx_DMA` are mutually exclusive.

---

### 4. ADC + DMA — MEDIUM

**Driver**: `arch/arm/src/stm32/stm32_adc.c` — supports ADC1-3.

**Kconfig**:
```
CONFIG_ANALOG=y
CONFIG_ADC=y
CONFIG_STM32_ADC1=y
CONFIG_STM32_DMA2=y           # ADC1 uses DMA2
CONFIG_ADC_FIFOSIZE=16
```

**Multi-channel**: Scan mode + DMA supports the 6 battery monitoring channels. Channel list and sample times configured in board-level code.

---

### 5. I2C (I2C2) — MEDIUM

**Driver**: `arch/arm/src/stm32/stm32_i2c.c` — supports I2C1-3.

**Kconfig**:
```
CONFIG_I2C=y
CONFIG_STM32_I2C2=y
```

**Challenge**: PB3 as I2C2_SDA (AF9) is non-standard.

- PB10 (SCL, AF4) — standard mapping, no issue
- **PB3 (SDA, AF9)** — F413-specific alternate function. Must be correctly defined in pinmap header (`stm32f413xx_pinmap.h`)

Board init registers I2C2 bus and attaches LSM6DS3TR-C at address 0x6A.

---

### 6. Timer/PWM — MEDIUM

**Driver**: `arch/arm/src/stm32/stm32_pwm.c` — supports all timers.

**Kconfig (for Ports A-B)**:
```
CONFIG_PWM=y
CONFIG_STM32_TIM1=y
CONFIG_STM32_TIM1_PWM=y
CONFIG_STM32_TIM1_CHANNEL=1
```

**Motor control considerations**:
- TIM1 (advanced): supports complementary outputs + dead-time insertion
- TIM3, TIM4 (general purpose): no complementary output; H-bridge direction via separate GPIO
- Standard PWM API (`/dev/pwmN`) may need board-level custom logic for brake/coast patterns

---

### 7. DAC — LOW

**Driver**: `arch/arm/src/stm32/stm32_dac.c`

**Kconfig**:
```
CONFIG_DAC=y
CONFIG_STM32_DAC1=y
```

PA4 = DAC_OUT1 is a fixed pin (analog mode). Timer-triggered DMA for continuous waveform (audio) supported.

---

### 8. TLC5955 LED Driver — LOW

**No NuttX driver.** TLC5955 is a 48-channel 16-bit PWM LED driver from TI with 769-bit shift register protocol.

**NuttX LED framework**: USERLED (GPIO), RGBLED (PWM), I2C LED controllers (PCA9635, LP3943) exist but none match TLC5955's protocol.

**Implementation**: Custom driver over SPI1. Expose as character device or LED subsystem.

---

### 9. W25Q256 SPI NOR Flash — LOW

**Two drivers exist, both with limitations:**

| Driver | Kconfig | Supported Chips | Issue |
|---|---|---|---|
| `w25.c` (SPI) | `MTD_W25` | W25Q16-128 | **24-bit addressing only**. No W25Q256 |
| `w25qxxxjv.c` (QSPI) | `W25QXXXJV` | W25Q016-Q01 incl. W25Q256 | **QSPI only**. Not standard SPI |

**Recommended approach**:
1. Extend `w25.c` with 4-byte addressing for W25Q256
2. Adapt `w25qxxxjv.c` to work over standard SPI
3. Temporarily use 3-byte mode (first 16MB only)

---

### 10. LSM6DS3TR-C IMU — LOW

**Partial support**: `drivers/sensors/lsm6dsl.c` (char device) and `drivers/sensors/lsm6dso32.c` (uORB) exist.

- LSM6DS3TR-C is largely register-compatible with LSM6DSL
- WHO_AM_I register value differs — minor fix needed
- I2C address 0x6A
- Kconfig: `CONFIG_SENSORS_LSM6DSL=y`

---

### 11. Bluetooth CC2564C — LOW

**Driver exists**: NuttX has HCI UART Bluetooth with CC2564 initialization support.

**Kconfig**:
```
CONFIG_WIRELESS_BLUETOOTH=y
CONFIG_BLUETOOTH_UART=y
```

**Challenges**:
- CC2564C requires firmware patch loading (~7KB) at boot
- RTS/CTS hardware flow control required (USART2: TX=PD5, RX=PD6, CTS=PD3, RTS=PD4)
- Defer to later phases

---

## Risk Summary

1. **F413 chip definition must be added first** — no peripheral drivers work without it
2. **I2C2 PB3/AF9** — most likely configuration pitfall due to non-standard pin mapping
3. **W25Q256** — no ready-to-use driver for standard SPI with 32-bit addressing
4. **TLC5955** — completely custom driver needed
5. **UART9/10** — must be added to NuttX (following UART7/8 pattern)
6. **Standard peripherals** (USB, SPI, ADC, DAC, PWM, I2C) have mature drivers available
