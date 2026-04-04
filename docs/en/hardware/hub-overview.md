# SPIKE Prime Hub Hardware Overview

## MCU Specifications

| Item | Value |
|---|---|
| MCU | STM32F413VG (ARM Cortex-M4F) |
| SYSCLK | 96 MHz (16 MHz HSE + PLL) |
| Flash | 1 MB (nominal). Physically 1.5 MB may exist (requires hardware verification) |
| RAM | 320 KB (SRAM1 256 KB + SRAM2 64 KB) |
| Package | LQFP100 |

> **Note**: The STM32F413H-Discovery Kit uses STM32F413**ZH** (1.5 MB Flash). The SPIKE Hub uses STM32F413**VG** (nominal 1 MB). Whether Bank 2 (512 KB) is available requires hardware verification.

---

## Flash Layout

### Memory Map

```
0x08000000 +---------------------+
           | LEGO Bootloader     | 32 KB (Sectors 0-1, non-erasable)
0x08008000 +---------------------+
           |                     |
           |   Firmware          | 992 KB (Sectors 2-11)
           |                     |
0x08100000 +---------------------+
           |   Bank 2 Additional | 512 KB (Sectors 12-15, may be unavailable on VG)
0x08180000 +---------------------+

0x20000000 +---------------------+
           |   SRAM1             | 256 KB
0x20040000 +---------------------+
           |   SRAM2             | 64 KB
0x20050000 +---------------------+
```

### Bank 1 Sector Details (1 MB)

| Sector | Address | Size | Usage |
|--------|---------|------|-------|
| 0 | `0x08000000` | 16 KB | LEGO bootloader |
| 1 | `0x08004000` | 16 KB | LEGO bootloader |
| 2 | `0x08008000` | 16 KB | Firmware start |
| 3 | `0x0800C000` | 16 KB | Firmware |
| 4 | `0x08010000` | 64 KB | Firmware |
| 5 | `0x08020000` | 128 KB | Firmware |
| 6-11 | `0x08040000` - `0x080FFFFF` | 128 KB each | Firmware |

Source: RM0430 (STM32F413/423 Reference Manual) Table 5

---

## Power Management

### BAT_PWR_EN (PA13)

When running on battery, PA13 must be driven HIGH or the hub powers off.

| Pin | Function | Notes |
|-----|----------|-------|
| PA13 | BAT_PWR_EN | Battery power hold. Must be driven HIGH immediately after boot |
| PA14 | PORT_3V3_EN | 3.3V power control for I/O ports |

PA13 is shared with JTMS-SWDIO (SWD debug) by default. Configuring it as GPIO output disables SWD debugging. When USB-powered, battery power hold is unnecessary, so debug builds can skip PA13/PA14 reconfiguration.

### PA13 Configuration in NuttX

Configure in `stm32_boardinitialize()`:

```c
void stm32_boardinitialize(void)
{
    // Hold battery power (PA13 = BAT_PWR_EN)
    stm32_configgpio(GPIO_OUTPUT | GPIO_PUSHPULL | GPIO_SPEED_2MHZ |
                     GPIO_OUTPUT_SET | GPIO_PORTA | GPIO_PIN13);
}
```

GPIO clocks are already enabled by `stm32_clockconfig()`, so PA13 is configurable at this point. The timing difference from pybricks' PA13 setup is on the order of tens to hundreds of microseconds, which is negligible (HSE startup + PLL lock of several ms is dominant).

---

## Boot Sequence

### LEGO DFU Bootloader to NuttX Startup

```
LEGO DFU Bootloader (0x08000000)
  |
  +- DFU mode check (BT button held -> DFU mode)
  |
  +- Jump to firmware (0x08008000)
       |
       NuttX __start()
         +- stm32_clockconfig()     <- HSE + PLL setup (16MHz -> 96MHz), peripheral clock enable
         +- arm_fpuconfig()         <- FPU enable
         +- stm32_lowsetup()        <- Early UART setup
         +- stm32_gpioinit()        <- GPIO clock enable (already done in clockconfig)
         +- BSS clear, .data copy
         +- stm32_boardinitialize() <- * PA13 (BAT_PWR_EN) = HIGH
         +- nx_start()              <- OS kernel start
```

### VTOR Configuration

NuttX only requires `FLASH ORIGIN = 0x08008000` in the linker script for VTOR to be set correctly. The `.vectors` section is placed at the start of FLASH, and `stm32_irq.c` writes `NVIC_VECTAB = 0x08008000`.

### Clock Configuration

| Parameter | Value |
|---|---|
| HSE | 16 MHz |
| PLL | 16 MHz -> 96 MHz |
| AHB | /1 (96 MHz) |
| APB1 | /2 (48 MHz) |
| APB2 | /1 (96 MHz) |
| Flash wait state | 5 |

---

## DFU Bootloader

### Basic Information

| Item | Value |
|---|---|
| VID:PID | `0694:0008` |
| Protocol | STMicroelectronics DfuSe compatible |
| Protected region | `0x08000000` - `0x08007FFF` (non-erasable) |

### Entering DFU Mode

1. Unplug USB cable
2. Hold the Bluetooth button
3. While holding, plug in USB
4. Release after 5 seconds -- battery LED cycles colors

### Firmware Flashing

```bash
# Check device
dfu-util -l
# -> [0694:0008] Internal Flash

# Flash firmware
dfu-util -d 0694:0008 -a 0 -s 0x08008000:leave -D nuttx.bin
```

### Limitations

- Cannot enter DFU mode programmatically (physical button required)
- DFU bootloader does NOT hold PA13 (BAT_PWR_EN) HIGH -- USB power required
- `0x08000000` - `0x08007FFF` region is write-protected
