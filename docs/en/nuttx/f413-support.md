# STM32F413 Chip Support

Overview of STM32F413 chip support in NuttX.

## Why a Fork is Needed

The STM32F413 is unsupported at the chip level in NuttX 12.12.0. An out-of-tree board definition alone is insufficient; kernel-level changes are required:

- `CONFIG_ARCH_CHIP_STM32F413` does not exist in Kconfig
- No infrastructure for UART9/10 (Kconfig, IRQ, RCC, serial driver)
- The interrupt vector table size is a kernel compile-time constant
- RCC clock enable code is on the kernel side

The closest existing chip is the STM32F412. Compared to F412, F413 adds UART9/10, FMPI2C, DFSDM, SAI1, CAN3, increased Flash capacity (1MB -> 1.5MB), and increased SRAM (256KB -> 320KB).

## Minimal Patch Set (10 Files)

| # | File | Changes |
|---|---|---|
| 1 | `arch/arm/src/stm32/Kconfig` | Add `STM32_STM32F413` family config, `ARCH_CHIP_STM32F413VG`, `STM32_HAVE_UART9/10` |
| 2 | `arch/arm/include/stm32/chip.h` | Define F413 peripheral counts, Flash 1.5MB (0x180000) / SRAM 320KB |
| 3 | `arch/arm/include/stm32/stm32f40xxx_irq.h` | UART9 IRQ=88, UART10 IRQ=89, update `STM32_IRQ_NEXTINT` |
| 4 | `arch/arm/src/stm32/hardware/stm32f40xxx_memorymap.h` | Add UART9/10 base addresses |
| 5 | `arch/arm/src/stm32/hardware/stm32f40xxx_rcc.h` | Add UART9/10 bits to APB2ENR |
| 6 | `arch/arm/src/stm32/stm32f40xxx_rcc.c` | Add UART9/10 clock enable to `rcc_enableapb2()` |
| 7 | `arch/arm/src/stm32/stm32_serial.c` | Add UART9/10 device instances (following UART7/8 pattern) |
| 8 | `arch/arm/src/stm32/hardware/stm32f413xx_pinmap.h` | New file: F413-specific pin map (based on F412 + UART9/10, AF11) |
| 9 | `arch/arm/src/stm32/hardware/stm32_pinmap.h` | Add F413 pin map include branch |
| 10 | `boards/Kconfig` | Add board entry |

## UART9/10 Details

### IRQ Numbers

| Peripheral | IRQ Number | Bus |
|---|---|---|
| UART9 | 88 | APB2 |
| UART10 | 89 | APB2 |

### Base Addresses

| Peripheral | Base Address | Bus |
|---|---|---|
| UART9 | 0x40011800 | APB2 |
| UART10 | 0x40011C00 | APB2 |

UART9/10 reside on the APB2 bus, so baud rate calculation uses PCLK2 (96 MHz). Same bus as USART1/USART6.

### RCC Enable Bits

| Peripheral | Register | Bit Position | Mask |
|---|---|---|---|
| UART9 | RCC_APB2ENR | Bit 6 | 0x00000040 |
| UART10 | RCC_APB2ENR | Bit 7 | 0x00000080 |

### Alternate Function

UART9/10 all use AF11:

| Function | Pin | AF |
|---|---|---|
| UART9_TX | PD15 / PG1 | AF11 |
| UART9_RX | PD14 / PG0 | AF11 |
| UART10_TX | PE3 / PG12 | AF11 |
| UART10_RX | PE2 / PG11 | AF11 |

SPIKE Prime Hub port assignments:

| Port | UART | AF |
|---|---|---|
| A | UART7 | AF8 |
| B | UART4 | AF11 |
| C | UART8 | AF8 |
| D | UART5 | AF8 |
| E | UART10 | AF11 |
| F | UART9 | AF11 |

## Clock Configuration

SPIKE Prime Hub: 16 MHz HSE -> 96 MHz SYSCLK

```c
#define STM32_BOARD_XTAL        16000000      // 16 MHz HSE
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)    // VCO input = 2 MHz
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(96)    // VCO output = 192 MHz
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2      // SYSCLK = 96 MHz
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(4)     // USB = 48 MHz
#define STM32_SYSCLK_FREQUENCY  96000000ul

// Bus clocks
// AHB  = 96 MHz (HPRE  = SYSCLK)
// APB1 = 48 MHz (PPRE1 = HCLKd2)
// APB2 = 96 MHz (PPRE2 = HCLK)
```

## F413 vs F412 Gap

| Item | F412 (existing in NuttX) | F413 (needs addition) |
|---|---|---|
| Chip variant Kconfig | Present | Missing (needs addition) |
| Peripheral counts (chip.h) | For F412 | Needs update for F413 |
| Pin map header | `stm32f412xx_pinmap.h` | `stm32f413xx_pinmap.h` (new file) |
| UART | 1-8 | 1-10 (9/10 need addition) |
| Flash | 1 MB | 1.5 MB |
| SRAM | 256 KB | 320 KB |

## Features That Can Be Deferred

| Feature | Reason |
|---|---|
| FMPI2C driver | SPIKE Prime Hub uses standard I2C2. Requires a new driver due to I2C v2 register interface |
| DFSDM driver | Not used on SPIKE Prime Hub. No existing code in NuttX |
| SAI1 driver | Not used on SPIKE Prime Hub |
| CAN3 driver | Not used on SPIKE Prime Hub. Only CAN1/2 are supported |
