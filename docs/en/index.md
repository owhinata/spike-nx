# SPIKE Prime Hub NuttX Project

A project to run NuttX RTOS on the SPIKE Prime Hub.

## Quick Start

### Build

```bash
make
```

### Flash (DFU)

```bash
brew install dfu-util  # first time only
```

1. Unplug the Hub's USB cable
2. Hold the Bluetooth button, plug in USB, wait 5 seconds, then release (DFU mode)
3. Flash (the default `usbnsh` config is BUILD_PROTECTED, so flash both kernel and user blobs):

```bash
dfu-util -d 0694:0008 -a 0 -s 0x08008000 -D nuttx/nuttx.bin
dfu-util -d 0694:0008 -a 0 -s 0x08080000:leave -D nuttx/nuttx_user.bin
```

### Serial Connection

```bash
picocom /dev/tty.usbmodem01
```

## Hardware Specs

| Item | Spec |
|------|------|
| MCU | STM32F413VG (ARM Cortex-M4, 96MHz) |
| Flash | 1 MB (992KB available, 32KB bootloader) |
| RAM | 320 KB (SRAM1 256KB + SRAM2 64KB) |

## Benchmark

### CoreMark

| Board | MCU | CoreMark Score | CoreMark/MHz |
|-------|-----|---------------|-------------|
| SPIKE Prime Hub | STM32F413VG (96MHz) | 171.19 | 1.78 |
| B-L4S5I-IOT01A | STM32L4R5VI (80MHz) | 143.16 | 1.79 |
