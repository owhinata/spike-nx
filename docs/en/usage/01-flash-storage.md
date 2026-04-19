# /mnt/flash (W25Q256 + LittleFS)

The Hub's external W25Q256 (32 MB SPI NOR) is mounted as NuttX LittleFS for persistent storage of configuration, programs, logs, etc.  Files can be transferred between PC and Hub via NSH's `rz` / `sz` (see [Zmodem file transfer](../development/file-transfer.md) for the picocom procedure).

## Mount and Capacity

After boot, `/mnt/flash` is automatically mounted as LittleFS:

```
nsh> mount
  /mnt/flash type littlefs
  /proc type procfs
nsh> df -h /mnt/flash
```

## Normal File Operations

Standard POSIX-style NSH commands work:

```
nsh> echo "hello" > /mnt/flash/note.txt
nsh> cat /mnt/flash/note.txt
hello
nsh> ls -l /mnt/flash
nsh> rm /mnt/flash/note.txt
```

Files survive `reboot` (LittleFS power-fail safety + NOR flash non-volatility).

## Integrity Check (md5)

Verify a file was not corrupted in transit:

```
nsh> md5 -f /mnt/flash/file.bin
e3b0c44298fc1c149afbf4c8996fb924
```

Note that NSH's `md5` prints the 32-char hex digest **with no trailing newline**.  For scripts, append `; echo` to add the missing newline.

## Partitions and Exposed Devices

| Region | Address | Size | Exposed device |
|---|---|---|---|
| LEGO bootloader | `0x000000-0x07FFFF` | 512 KB | (not exposed) |
| pybricks block device | `0x080000-0x0BFFFF` | 256 KB | (not exposed) |
| Update key | `0x0FF000-0x0FFFFF` | 4 KB | (not exposed) |
| LittleFS partition | `0x100000-0x1FFFFFF` | 31 MB | `/dev/mtdblock0` → `/mnt/flash` |

The full-chip raw device (`/dev/mtd0`) is not exposed to user space (so the LEGO area cannot be erased accidentally).  For deep debugging, see the "dev-only raw access" section of the [W25Q256 driver doc](../drivers/w25q256.md).

!!! warning "LittleFS-only region"
    `0x100000-0x1FFFFFF` (31 MB) is fully owned by LittleFS.  Any non-LittleFS raw data written there is wiped by the first-boot autoformat.

## Troubleshooting

### `/mnt/flash` is not mounted

Look for `W25Q256:` lines in `dmesg`:

- `JEDEC ID 0xef4019, 32MB detected` should appear (otherwise SPI wiring or SPI2 config is wrong).
- `LittleFS mounted at /mnt/flash` should appear.

Manual recovery on mount failure:

```
nsh> mount -t littlefs -o autoformat /dev/mtdblock0 /mnt/flash
```

If that still fails, a full partition erase utility is not yet provided.  Re-flashing and waiting for the boot-time forceformat fallback often recovers.

### File-transfer issues

See [Zmodem file transfer — Option Notes](../development/file-transfer.md#option-notes) for ZNAK retry causes (no `-e`, `-L 256` is required, etc.).

## Related

- Driver spec: [W25Q256 driver](../drivers/w25q256.md)
- Transfer procedure: [Zmodem file transfer](../development/file-transfer.md)
- Hardware: [W25Q256 peripheral](../hardware/peripherals.md), [DMA / IRQ allocation](../hardware/dma-irq.md)
- Tests: [test-spec category G](../testing/test-spec.md)
