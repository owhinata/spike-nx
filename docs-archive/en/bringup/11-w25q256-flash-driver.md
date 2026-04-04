# W25Q256 Flash Driver

## 1. Overview

W25Q256 is a Winbond 256 Mbit (32 MB) SPI NOR Flash requiring 4-byte addressing.

| Item | Value |
|---|---|
| Capacity | 32 MB (256 Mbit) |
| Interface | SPI (Standard/Dual/Quad) |
| Page size | 256 bytes |
| Sector size | 4 KB |
| Block size | 64 KB |
| Device ID | 0xEF 0x40 0x19 |

---

## 2. SPIKE Hub Connection

| Signal | Connected To | Notes |
|---|---|---|
| SPI | SPI2 | DMA1 Stream4 (TX, ch0) / Stream3 (RX, ch0) |
| CS | PB12 | Software NSS (Active Low) |
| SPI mode | CPOL=0, CPHA=0 | MSB first, prescaler /2 |

---

## 3. 4-Byte Addressing

### Two Approaches

| Approach | Description | Example Commands |
|---|---|---|
| A: Address mode switch | Enter 4-byte mode via `0xB7` | Standard commands (0x03, 0x02, 0x20) use 4-byte addresses |
| **B: 4-byte specific commands** | Always use 4-byte address commands | 0x13/0x0C (read), 0x12 (write), 0x21 (erase) |

### Pybricks Uses Approach B

```c
FLASH_CMD_READ_DATA  = 0x0C  // Fast Read with 4-Byte Address
FLASH_CMD_WRITE_DATA = 0x12  // Page Program with 4-Byte Address
FLASH_CMD_ERASE_BLOCK = 0x21 // Sector Erase with 4-Byte Address
```

More robust — doesn't depend on address mode register state.

### Address Format

```c
buf[0] = address >> 24;  // MSB
buf[1] = address >> 16;
buf[2] = address >> 8;
buf[3] = address;        // LSB
```

---

## 4. Flash Memory Layout

| Region | Address (SPI Flash) | Size | Usage |
|---|---|---|---|
| Bootloader data | 0x000000 - 0x07FFFF | 512 KB | LEGO bootloader area |
| Pybricks block device | 0x080000 - 0x0BFFFF | 256 KB | User program backup |
| Update key | 0x0FF000 - 0x0FFFFF | 4 KB | mboot FS-load key |
| Filesystem | 0x100000 - 0x1FFFFFF | 31 MB | FAT filesystem |

For NuttX, treat the first 1 MB as reserved. Use 1 MB onward for filesystem.

---

## 5. NuttX Driver Options

### Existing Driver Issues

| Driver | Kconfig | Problem |
|---|---|---|
| `w25.c` (SPI) | `MTD_W25` | 3-byte addressing only (up to W25Q128) |
| `w25qxxxjv.c` (QSPI) | `W25QXXXJV` | QSPI only (not standard SPI) |

### Recommended: Extend `w25.c`

1. Add W25Q256 device ID (0xEF4019) to JEDEC ID table
2. Add 4-byte address command support (0x0C, 0x12, 0x21)
3. Branch address transmission between 3-byte/4-byte

Moderate effort. Use pybricks driver (`block_device_w25qxx_stm32.c`) as reference.

### Alternative

- Use only first 16 MB (3-byte addressing sufficient) → 15 MB for filesystem
- Create standalone MTD driver based on pybricks implementation

---

## 6. Reference Files

- `pybricks/lib/pbio/drv/block_device/block_device_w25qxx_stm32.c` — pybricks W25Q256 driver
- `pybricks/micropython/ports/stm32/boards/LEGO_HUB_NO6/mpconfigboard.h` — 32-bit address config
