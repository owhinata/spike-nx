# Bootloader Analysis

## 1. Three-Stage Boot Architecture

The SPIKE Prime Hub has a three-stage boot structure:

| Stage | Address Range | Size | Description |
|---|---|---|---|
| LEGO Built-in Bootloader | `0x08000000` - `0x08007FFF` | 32 KB | Non-erasable DFU bootloader |
| mboot (secondary bootloader) | `0x08008000` - `0x0800FFFF` | 32 KB | MicroPython's mboot |
| Application | `0x08010000` - `0x080FFFFF` | 960 KB | Main firmware |

### Pybricks Configuration

Pybricks skips mboot, placing firmware directly at `0x08008000` (992 KB available):

```
FLASH_BOOTLOADER : ORIGIN = 0x08000000, LENGTH = 32K
FLASH_FIRMWARE   : ORIGIN = 0x08008000, LENGTH = 992K
```

### NuttX Options

| Approach | Firmware Start | Available Size | Advantage |
|---|---|---|---|
| **A: No mboot** | `0x08008000` | 992 KB + 512 KB | Maximum space. Same as pybricks |
| B: With mboot | `0x08010000` | 960 KB + 512 KB | Easier OTA updates |

**Recommended**: Approach A (`0x08008000`). No need for mboot initially.

---

## 2. LEGO Built-in Bootloader (DFU)

### Entry Method

1. Unplug USB cable
2. Hold Bluetooth button
3. While holding, plug in USB
4. Release after 5 seconds → battery LED cycles colors

### Protocol

**STMicroelectronics DfuSe compatible**. Standard `dfu-util` works.

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx.bin
```

### Limitations

- Cannot enter DFU mode programmatically (physical button required)
- DFU bootloader does NOT hold PA13 (BAT_PWR_EN) HIGH → USB power required
- `0x08000000` - `0x08007FFF` is write-protected

---

## 3. mboot Firmware Update Mechanism

MicroPython's mboot supports OTA updates via SPI Flash:

1. Write update key (`0x12345678`) to SPI Flash at address `1020 * 1024`
2. On next boot, mboot detects the key
3. Read firmware from SPI Flash filesystem, write to internal flash
4. Power-failure safe (interrupted updates restart on boot)
5. Key erased after success

Not needed for NuttX initially. Custom OTA can be implemented later.

---

## 4. SystemInit() Code Flow Analysis

From `pybricks/lib/pbio/platform/prime_hub/platform.c` SystemInit() (lines 999-1050):

```
Reset_Handler (startup.s)
  ├─ Set SP (_estack)
  ├─ Copy .data (Flash → SRAM)
  ├─ Zero .bss
  └─ Call SystemInit()
       ├─ SCB->CCR: 8-byte stack alignment
       ├─ SCB->VTOR: vector table relocation
       ├─ RCC: Enable HSE + configure PLL (16MHz → 96MHz)
       ├─ FLASH: wait states = 5
       ├─ RCC: SYSCLK = PLL, AHB/1, APB1/2, APB2/1
       ├─ RCC: Enable all peripheral clocks
       │   (GPIOA-E, DMA1-2, USART2, UART4/5/7-10,
       │    TIM1-8/12, I2C2, DAC, SPI1-2, ADC1, OTG_FS, SYSCFG)
       └─ PA13: BAT_PWR_EN = HIGH (hold power)
```

### Estimated Clock Cycles Before PA13

PA13 is set after: stack alignment (~few clocks), VTOR (~few clocks), HSE startup wait (~hundreds μs to ms), PLL lock wait (~hundreds μs), flash/bus config, peripheral clock enable (~tens of clocks).

**HSE startup + PLL lock dominate: total ~few ms**. CPU runs on HSI (16 MHz internal RC) until PLL locks.

---

## 5. NuttX Boot Sequence Mapping

| pybricks | NuttX | Notes |
|---|---|---|
| Reset_Handler | `__start()` | Entry point |
| .data copy / .bss clear | Within `__start()` | Same pattern |
| SystemInit() → clock config | `stm32_clockconfig()` | |
| SystemInit() → VTOR setup | `stm32_irq.c` → NVIC_VECTAB | Automatic via linker script |
| SystemInit() → PA13 HIGH | `stm32_boardinitialize()` | Most appropriate place |
| main() | `nx_start()` → `board_late_initialize()` | |
