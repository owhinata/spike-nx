# STM32F413 NuttX Support Survey

## 1. Existing STM32F4 Chip Support

### F4 Families Supported in NuttX 12.12.0

| Family | Chip Variants | Notes |
|---|---|---|
| STM32F401 | 12 | xBC, xDE sub-variants |
| STM32F405 | 3 | RG, VG, ZG |
| STM32F407 | 6 | VE, VG, ZE, ZG, IE, IG |
| STM32F410 | 1 | RB |
| STM32F411 | 3 | CE, RE, VE |
| STM32F412 | 2 | CE, ZG |
| STM32F427 | 3 | V, Z, I |
| STM32F429 | 5 | V, Z, I, B, N |
| STM32F446 | 4 | M, R, V, Z |
| STM32F469 | 4 | A, I, B, N |

**STM32F413 / STM32F423 are NOT supported.** No Kconfig entry, no F413-related code anywhere in the source tree.

### F4 Boards (Key Boards)

- `nucleo-f401re`, `nucleo-f410rb`, `nucleo-f411re`, **`nucleo-f412zg`**
- `nucleo-f429zi`, `nucleo-f446re`
- `stm32f4discovery`, `stm32f411e-disco`, `stm32f429i-disco`
- `olimex-stm32-e407`, `olimex-stm32-h405`, `olimex-stm32-h407`, `olimex-stm32-p407`
- `omnibusf4`, `photon`, and others (55 boards total)

Best template: **`nucleo-f412zg`** (same F41x series, runs at 96 MHz)

---

## 2. F413-Specific Peripheral Support

### UART/USART

| Peripheral | NuttX Support | Notes |
|---|---|---|
| USART1, 2, 3, 6 | Supported | Common to all F4 |
| UART4, 5 | Supported | Enabled for F427/F429/F469 |
| UART7, 8 | Supported | IRQ, RCC, serial driver all implemented |
| **UART9** | **Not supported** | No IRQ vectors, RCC bits, or serial driver |
| **UART10** | **Not supported** | Same as above |

UART9/10 are F413-specific. The SPIKE Prime Hub uses UART10 (Port E) and UART9 (Port F).

### I2C

| Peripheral | NuttX Support | Notes |
|---|---|---|
| I2C1, 2, 3 | Supported | Standard I2C v1 driver |
| FMPI2C1 | **RCC only** | `RCC_APB1ENR_FMPI2C1EN` defined. No driver (needs I2C v2 register interface) |

The SPIKE Hub IMU uses standard I2C2, so FMPI2C is not needed initially.

### Other

| Peripheral | NuttX Support | Notes |
|---|---|---|
| DFSDM | **Not supported** | No code in NuttX |
| SAI1 | Uncertain | May exist for other STM32 families |
| CAN3 | Not supported | Only CAN1/2 supported |

---

## 3. Clock Configuration

### nucleo-f412zg PLL Settings (Reference)

Nucleo-F412ZG: 8 MHz HSE → 96 MHz SYSCLK:

```c
#define STM32_BOARD_USEHSE      1
#define STM32_BOARD_XTAL        8000000       // 8 MHz HSE
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)    // VCO input = 1 MHz
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(384)   // VCO output = 384 MHz
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_4      // SYSCLK = 96 MHz
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(8)     // USB = 48 MHz
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)
#define STM32_SYSCLK_FREQUENCY  96000000ul

// Bus clocks
#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK   // AHB = 96 MHz
#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2   // APB1 = 48 MHz
#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK     // APB2 = 96 MHz

// DCKCFGR2 (F412/F413-specific clock config register)
#define STM32_RCC_DCKCFGR2_CK48MSEL    RCC_DCKCFGR2_CK48MSEL_PLL
#define STM32_RCC_DCKCFGR2_FMPI2C1SEL  RCC_DCKCFGR2_FMPI2C1SEL_APB
#define STM32_RCC_DCKCFGR2_SDIOSEL     RCC_DCKCFGR2_SDIOSEL_48MHZ
```

### SPIKE Prime Hub Settings

SPIKE Prime Hub: 16 MHz HSE → 96 MHz SYSCLK (based on pybricks):

```c
#define STM32_BOARD_USEHSE      1
#define STM32_BOARD_XTAL        16000000      // 16 MHz HSE
#define STM32_PLLCFG_PLLM       RCC_PLLCFG_PLLM(8)    // VCO input = 2 MHz
#define STM32_PLLCFG_PLLN       RCC_PLLCFG_PLLN(96)    // VCO output = 192 MHz
#define STM32_PLLCFG_PLLP       RCC_PLLCFG_PLLP_2      // SYSCLK = 96 MHz
#define STM32_PLLCFG_PLLQ       RCC_PLLCFG_PLLQ(4)     // USB = 48 MHz
#define STM32_PLLCFG_PLLR       RCC_PLLCFG_PLLR(2)
#define STM32_SYSCLK_FREQUENCY  96000000ul

// Bus clocks (same as pybricks)
#define STM32_RCC_CFGR_HPRE     RCC_CFGR_HPRE_SYSCLK   // AHB = 96 MHz
#define STM32_RCC_CFGR_PPRE1    RCC_CFGR_PPRE1_HCLKd2   // APB1 = 48 MHz
#define STM32_RCC_CFGR_PPRE2    RCC_CFGR_PPRE2_HCLK     // APB2 = 96 MHz
```

---

## 4. Chip Support Architecture

How F412 is wired in NuttX:

1. **Kconfig**: `ARCH_CHIP_STM32F412ZG` → `STM32_STM32F412` → `STM32_STM32F4XXX`
2. **`arch/arm/include/stm32/chip.h`**: Per-chip peripheral counts (NUSART, NI2C, etc.)
3. **`arch/arm/include/stm32/stm32f40xxx_irq.h`**: Shared F4 IRQ header with `#if` branches
4. **`hardware/stm32f40xxx_memorymap.h`**: Shared F4 memory map
5. **`hardware/stm32f412xx_pinmap.h`**: F412-specific pin map
6. **`hardware/stm32f40xxx_rcc.h`**: Shared F4 RCC header with `#if` branches
7. **`stm32f40xxx_rcc.c`**: Shared F4 RCC init with `#if` branches

F412 shares the F40xxx infrastructure but has its own dedicated pinmap.

---

## 5. Gap Analysis: Changes Required for F413

### Required (Minimum Viable)

| Item | Effort | Files to Modify/Create |
|---|---|---|
| Kconfig: F413 chip variants | Small | `arch/arm/src/stm32/Kconfig` |
| Kconfig: F413 family config (HAVE_UARTx, etc.) | Small | Same |
| Chip peripheral counts | Small | `arch/arm/include/stm32/chip.h` |
| Pinmap header | Medium | Create `hardware/stm32f413xx_pinmap.h` (based on F412) |
| Pinmap selection | Small | `hardware/stm32_pinmap.h` |
| RCC init F413 branch | Small | `stm32f40xxx_rcc.c` |
| RCC header F413 additions | Small | `hardware/stm32f40xxx_rcc.h` |
| Flash/SRAM sizes | Small | 1.5 MB Flash (0x180000), 320 KB SRAM |
| Board definition | Medium | `boards/arm/stm32/spike-prime-hub/` |

### UART9/10 Support (Needed for Ports E, F)

| Item | Effort | Files to Modify/Create |
|---|---|---|
| UART9/10 Kconfig | Small | `arch/arm/src/stm32/Kconfig` |
| UART9/10 IRQ vectors | Medium | `arch/arm/include/stm32/stm32f40xxx_irq.h` |
| UART9/10 RCC bits | Small | `hardware/stm32f40xxx_rcc.h` |
| UART9/10 serial driver | Medium | `stm32_serial.c` (follow UART7/8 pattern) |

### Deferrable

| Item | Reason |
|---|---|
| FMPI2C driver | SPIKE Hub uses standard I2C2 |
| DFSDM driver | Not used on SPIKE Hub |
| SAI1 driver | Not used on SPIKE Hub |
| CAN3 driver | Not used on SPIKE Hub |

### Recommended Approach

1. Add `STM32_STM32F413` family config selecting all F412 peripherals plus UART4-10
2. Reuse F412 RCC infrastructure (add F413 to existing `#if` guards)
3. Create F413 pinmap header based on F412
4. Add UART9/10 to serial driver following UART7/8 pattern
5. Add UART9/10 IRQ vectors (IRQ 88, 89)
