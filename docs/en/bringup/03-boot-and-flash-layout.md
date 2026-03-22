# Boot Process and Flash Layout

## 1. STM32F413VG Flash Sector Layout

The SPIKE Prime Hub MCU is STM32F413**VG** (nominal 1 MB Flash). However, the STM32F413 family may physically have 1.5 MB Flash (pybricks uses 992K after bootloader). **Design assumes 1 MB until hardware verification.**

> **Note**: STM32F413H-Discovery Kit uses STM32F413**ZH** (1.5 MB Flash). Bank 2 (sectors 12-15) is available on Discovery but unconfirmed on SPIKE Hub.

### Bank 1 (1 MB, Sectors 0-11)

| Sector | Address | Size | Usage |
|--------|---------|------|-------|
| 0 | `0x08000000` | 16 KB | LEGO bootloader |
| 1 | `0x08004000` | 16 KB | LEGO bootloader |
| 2 | `0x08008000` | 16 KB | **Firmware start** |
| 3 | `0x0800C000` | 16 KB | Firmware |
| 4 | `0x08010000` | 64 KB | Firmware |
| 5 | `0x08020000` | 128 KB | Firmware |
| 6 | `0x08040000` | 128 KB | Firmware |
| 7 | `0x08060000` | 128 KB | Firmware |
| 8 | `0x08080000` | 128 KB | Firmware |
| 9 | `0x080A0000` | 128 KB | Firmware |
| 10 | `0x080C0000` | 128 KB | Firmware |
| 11 | `0x080E0000` | 128 KB | Firmware |

### Bank 2 (512 KB, Sectors 12-15) — Needs Hardware Verification

| Sector | Address | Size | Usage |
|--------|---------|------|-------|
| 12 | `0x08100000` | 128 KB | Additional (may be unavailable on VG) |
| 13 | `0x08120000` | 128 KB | Same |
| 14 | `0x08140000` | 128 KB | Same |
| 15 | `0x08160000` | 128 KB | Same |

**Bank 1 total**: 4×16 KB + 1×64 KB + 7×128 KB = 1024 KB = 1 MB (VG nominal)
**Bank 1+2 total**: 1024 KB + 512 KB = 1536 KB = 1.5 MB (ZH / if physically present)

Source: RM0430 (STM32F413/423 Reference Manual) Table 5

---

## 2. SPIKE Prime Hub Boot Chain

### Memory Layout

From pybricks linker script (`pybricks/lib/pbio/platform/prime_hub/platform.ld`):

```
FLASH_BOOTLOADER : ORIGIN = 0x08000000, LENGTH = 32K   (sectors 0-1)
FLASH_FIRMWARE   : ORIGIN = 0x08008000, LENGTH = 992K   (sector 2 onward)
RAM              : ORIGIN = 0x20000000, LENGTH = 320K   (SRAM1 256K + SRAM2 64K)
```

```
0x08000000 ┌─────────────────────┐
           │ LEGO Bootloader     │ 32 KB (non-erasable)
0x08008000 ├─────────────────────┤
           │                     │
           │   Firmware          │ 992 KB (writable)
           │                     │
0x08100000 ├─────────────────────┤
           │   Additional area   │ 512 KB
0x08180000 └─────────────────────┘

0x20000000 ┌─────────────────────┐
           │   SRAM1             │ 256 KB
0x20040000 ├─────────────────────┤
           │   SRAM2             │ 64 KB
0x20050000 └─────────────────────┘
```

### Boot Sequence

1. **LEGO Bootloader** (0x08000000-0x08007FFF)
   - Non-erasable embedded bootloader
   - DFU mode: hold Bluetooth button 5 sec + plug USB
   - DFU VID/PID: `0x0694` / `0x0008`
   - Jumps to firmware at `0x08008000`

2. **Reset_Handler** (startup.s)
   - Set stack pointer to `_estack`
   - Copy `.data` section from flash to SRAM
   - Zero-fill `.bss` section
   - Call `SystemInit()`

3. **SystemInit()** (platform.c)
   - Enable 8-byte stack alignment
   - **Set VTOR** to firmware vector table
   - PLL setup: 16 MHz HSE → 96 MHz SYSCLK
   - Clock dividers: AHB /1, APB1 /2, APB2 /1
   - Enable all peripheral clocks
   - **Drive BAT_PWR_EN (PA13) HIGH** → maintain power

4. **main()** → application execution

---

## 3. NuttX VTOR Configuration

### How NuttX Sets VTOR

NuttX sets VTOR in `stm32_irq.c`:

```c
putreg32((uint32_t)_vectors, NVIC_VECTAB);
```

The `_vectors` symbol is placed by the linker script. By placing the `.vectors` section at the start of the FLASH region, VTOR automatically points to the correct address.

**There is no dedicated `CONFIG_STM32_VECTAB_OFFSET`.** The linker script's FLASH ORIGIN effectively determines VTOR.

### Configuration for SPIKE Prime Hub

1. Set linker script `FLASH ORIGIN = 0x08008000, LENGTH = 992K`
2. Place `.vectors` section at the start of FLASH region
3. `_vectors` symbol resolves to `0x08008000`
4. `stm32_irq.c` automatically writes `NVIC_VECTAB = 0x08008000`
5. VTOR offset `0x8000` (32 KB) satisfies ARM's 512-byte alignment requirement

No changes needed to existing NuttX code. Just configure the linker script FLASH ORIGIN.

---

## 4. BAT_PWR_EN (PA13) Power Management

### Problem

PA13 serves dual purposes:
- **JTMS-SWDIO**: SWD debug interface (default)
- **BAT_PWR_EN**: Battery power hold (used as GPIO output)

When running on battery, PA13 must be driven HIGH or the hub powers off.

### Pybricks Implementation

Set in `SystemInit()` (platform.c lines 1043-1049), the earliest possible point:

```c
// Configure PA13 as push-pull output, drive HIGH
GPIO_InitTypeDef gpio_init = {
    .Pin = GPIO_PIN_13,
    .Mode = GPIO_MODE_OUTPUT_PP,
};
HAL_GPIO_Init(GPIOA, &gpio_init);
HAL_GPIO_WritePin(GPIOA, GPIO_PIN_13, GPIO_PIN_SET);
```

### NuttX Implementation Strategy

NuttX boot sequence:

```
__start()
  ├─ stm32_clockconfig()     ← clock setup
  ├─ arm_fpuconfig()         ← FPU enable
  ├─ stm32_lowsetup()        ← early UART setup
  ├─ stm32_gpioinit()        ← GPIO clock enable
  ├─ BSS clear, .data copy
  ├─ stm32_boardinitialize() ← ★ board-specific early init
  └─ nx_start()              ← OS kernel start
```

**`stm32_boardinitialize()`** is the appropriate place for PA13:
- No OS services available, but direct register access works
- GPIO clocks already enabled by `stm32_gpioinit()`
- Use `stm32_configgpio()` to configure PA13 as output HIGH

```c
void stm32_boardinitialize(void)
{
    // Hold battery power (PA13 = BAT_PWR_EN)
    stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ |
                     GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN13);
}
```

### Pybricks Comparison

Pybricks `SystemInit()` (platform.c lines 999-1050) execution order:

```
1. SCB->CCR: stack alignment
2. SCB->VTOR: vector table relocation
3. HAL_RCC_OscConfig(): HSE + PLL (16MHz → 96MHz)  ← HSE startup + PLL lock (~ms)
4. HAL_RCC_ClockConfig(): SYSCLK=PLL, AHB/APB dividers
5. RCC->AHB1ENR: GPIOA-E, DMA1-2 clock enable     ← GPIO clocks enabled
6. RCC->APB1ENR: UART, TIM, I2C, DAC, SPI2
7. RCC->APB2ENR: TIM1/8, UART9/10, ADC1, SPI1
8. RCC->AHB2ENR: OTG_FS
9. PA13 = OUTPUT_PP, HIGH                           ← BAT_PWR_EN set
```

NuttX `__start()` corresponding order:

```
1. stm32_clockconfig()     ← HSE + PLL + peripheral clocks (pybricks steps 3-8)
2. arm_fpuconfig()
3. stm32_lowsetup()
4. stm32_gpioinit()        ← GPIO clocks already enabled in stm32_clockconfig()
5. BSS clear, .data copy
6. stm32_boardinitialize() ← ★ PA13 set (pybricks step 9)
```

**Difference**: NuttX adds FPU config, early UART, BSS clear before PA13. But GPIO clock (RCC_AHB1ENR_GPIOAEN) is already enabled in `stm32_clockconfig()`, so PA13 is configurable at `stm32_boardinitialize()`. The extra steps add ~tens of μs, negligible compared to HSE+PLL lock (~ms) which dominates in both cases.

### Timing Concern

- Timing difference between pybricks and NuttX PA13 setup is ~tens to hundreds of μs
- Total time from bootloader jump to PA13 assertion is ~few ms in both (HSE+PLL dominant)
- This delay should be acceptable
- If issues arise, a board-specific hook can be added in `stm32_clockconfig()` for earlier PA13 setup

### SWD Debug Trade-off

Repurposing PA13 (SWDIO) and PA14 (SWCLK) as GPIO disables SWD debugging:
- PA14 is used as `PORT_3V3_EN` (3.3V power to I/O ports)
- When USB-powered, battery power hold is unnecessary — debug builds can skip PA13/PA14 reconfiguration
- See [13-debugging-strategy.md](13-debugging-strategy.md) for details
