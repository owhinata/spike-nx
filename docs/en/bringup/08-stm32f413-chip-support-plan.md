# STM32F413 Chip Support Plan

## 1. Conclusion: NuttX Fork Is Unavoidable

Out-of-tree board definitions alone cannot support STM32F413. Kernel-level changes are required because:

- Chip variant (`CONFIG_ARCH_CHIP_STM32F413`) does not exist in Kconfig
- UART9/10 infrastructure (Kconfig, IRQ, RCC, serial driver) is completely absent
- Interrupt vector table size is a kernel compile-time constant
- RCC clock enable code is in the kernel

**Recommendation**: Create a fork of apache/nuttx with F413 support patches. Periodically rebase on upstream. Consider submitting as an upstream PR.

---

## 2. UART9/10 Hardware Details

### IRQ Numbers (Confirmed from stm32f413xx.h)

| Peripheral | IRQ Number | Bus |
|---|---|---|
| UART4 | 52 | APB1 |
| UART5 | 53 | APB1 |
| UART7 | 82 | APB1 |
| UART8 | 83 | APB1 |
| **UART9** | **88** | **APB2** |
| **UART10** | **89** | **APB2** |

### Base Addresses

| Peripheral | Bus | Base Address |
|---|---|---|
| UART7 | APB1 | 0x40007800 |
| UART8 | APB1 | 0x40007C00 |
| **UART9** | **APB2** | **0x40011800** |
| **UART10** | **APB2** | **0x40011C00** |

**Critical**: UART9/10 are on the **APB2 bus** (not APB1). Baud rate calculation must use PCLK2 (96 MHz). Same bus as USART1/USART6.

### RCC Enable Bits

| Peripheral | Register | Bit Position | Mask |
|---|---|---|---|
| UART9 | RCC_APB2ENR | Bit 6 | 0x00000040 |
| UART10 | RCC_APB2ENR | Bit 7 | 0x00000080 |

### Register Layout

UART9/10 use the **same USART_TypeDef register layout** as all other USART/UART peripherals. Existing NuttX register access code works unchanged.

---

## 3. Alternate Function Table (F413-Specific)

### UART9 Pin Options

| Function | Pin | AF |
|---|---|---|
| UART9_TX | PD15 | AF11 |
| UART9_RX | PD14 | AF11 |
| UART9_TX | PG1 | AF11 |
| UART9_RX | PG0 | AF11 |

### UART10 Pin Options

| Function | Pin | AF |
|---|---|---|
| UART10_TX | PE3 | AF11 |
| UART10_RX | PE2 | AF11 |
| UART10_TX | PG12 | AF11 |
| UART10_RX | PG11 | AF11 |

### I2C2 SDA on PB3

**Confirmed**: PB3 = I2C2_SDA is **AF9**. Verified from `stm32f413_af.csv`.

### SPIKE Hub Port AF Assignments

| Port | UART | AF | Notes |
|---|---|---|---|
| A | UART7 | AF8 | Standard |
| B | UART4 | AF11 | F413-specific alternate AF (normally AF8) |
| C | UART8 | AF8 | Standard |
| D | UART5 | AF8 | Standard |
| E | UART10 | AF11 | F413-specific |
| F | UART9 | AF11 | F413-specific |

---

## 4. Required NuttX Kernel File Changes

### Minimum Patch Set

| # | File | Change |
|---|---|---|
| 1 | `arch/arm/src/stm32/Kconfig` | Add `STM32_STM32F413` family, `ARCH_CHIP_STM32F413VG`, `STM32_HAVE_UART9/10` |
| 2 | `arch/arm/include/stm32/chip.h` | Add F413 peripheral counts, Flash/SRAM sizes |
| 3 | `arch/arm/include/stm32/stm32f40xxx_irq.h` | Add UART9 IRQ=88, UART10 IRQ=89, update `STM32_IRQ_NEXTINT` |
| 4 | `arch/arm/src/stm32/hardware/stm32f40xxx_memorymap.h` | Add UART9/10 base addresses |
| 5 | `arch/arm/src/stm32/hardware/stm32f40xxx_rcc.h` | Add UART9/10 bits to APB2ENR |
| 6 | `arch/arm/src/stm32/stm32f40xxx_rcc.c` | Add UART9/10 clock enable in `rcc_enableapb2()` |
| 7 | `arch/arm/src/stm32/stm32_serial.c` | Add UART9/10 device instances (follow UART7/8 pattern) |
| 8 | `arch/arm/src/stm32/hardware/stm32f413xx_pinmap.h` | New file: F413 pinmap (based on F412 + UART9/10, AF11) |
| 9 | `arch/arm/src/stm32/hardware/stm32_pinmap.h` | Add F413 pinmap include branch |
| 10 | `boards/Kconfig` | Add board entry |

### Initial Bring-up Alternative

Before creating the fork, use F412 config for initial testing:

```
CONFIG_ARCH_CHIP_STM32F412ZG=y
```

**Limitations**: Flash recognized as 1MB / SRAM as 256KB. UART9/10 unavailable. Basic NSH + UART7 console + USB CDC/ACM should work.

---

## 5. Fork Management Strategy

### Repository Structure

```
spike-nx/
├── nuttx/          # git submodule: owhinata/nuttx (fork, f413-support branch)
├── nuttx-apps/     # git submodule: apache/nuttx-apps (tag nuttx-12.12.0)
├── boards/         # Custom board definition (out-of-tree)
└── ...
```

### Branch Strategy

- `main`: Based on apache/nuttx nuttx-12.12.0
- `f413-support`: F413 chip support patches applied
- On upstream update: rebase `main` → rebase `f413-support`

### Upstream PR Possibility

STM32F413 is widely used (STM32F413H-Discovery is supported by Zephyr). An upstream PR to Apache NuttX has good chances of acceptance.
