# Zmodem File Transfer (PC ⇔ Hub)

Use picocom + lrzsz's `sz` / `rz` to transfer files over the USB CDC console.  Files received on the Hub are stored under `/mnt/flash` (W25Q256 + LittleFS).

## Prerequisites

### Host PC

```bash
# macOS
brew install lrzsz picocom

# Ubuntu/Debian
sudo apt install lrzsz picocom
```

### NuttX firmware (usbnsh defconfig)

| Setting | Purpose |
|---|---|
| `CONFIG_SYSTEM_ZMODEM=y` | Zmodem (`sz` / `rz`) |
| `CONFIG_SYSTEM_ZMODEM_DEVNAME="/dev/console"` | Transfer over the USB CDC console |
| `CONFIG_SYSTEM_ZMODEM_MOUNTPOINT="/mnt/flash"` | Default receive directory for `rz` |
| `CONFIG_CDCACM_RXBUFSIZE=2048` | Drop mitigation (no HW flow control) |
| `CONFIG_FS_LITTLEFS=y` + `CONFIG_MTD=y` | Persistent storage at `/mnt/flash` |
| `CONFIG_NETUTILS_CODECS=y` + `CONFIG_CODECS_HASH_MD5=y` | Integrity check via `md5 -f` |

See [W25Q256 driver](../drivers/w25q256.md) and [/mnt/flash usage](../usage/01-flash-storage.md) for details.

## PC → Hub (upload)

```bash
picocom --send-cmd 'sz -vv -L 256' /dev/tty.usbmodem01
```

Once picocom is connected:

1. Type `rz` at the NSH prompt (Hub enters Zmodem receive mode)
2. Press `Ctrl-A Ctrl-S` → enter the local file path (e.g. `/path/to/file.bin`) → Enter
3. Wait for "Transfer complete" (~45 s for 1 MB)
4. Exit picocom with `Ctrl-A Ctrl-X`

The file is saved to `/mnt/flash/<basename>`.

```
nsh> ls -l /mnt/flash/file.bin
nsh> md5 -f /mnt/flash/file.bin   # 32-char hex (no trailing newline)
```

## Hub → PC (download)

```bash
cd <destination directory>
picocom --receive-cmd 'rz -vv -y' /dev/tty.usbmodem01
```

Once picocom is connected:

1. Type `sz /mnt/flash/file.bin` at the NSH prompt (Hub enters Zmodem send mode)
2. Press `Ctrl-A Ctrl-R` → press Enter with no arguments (`rz` reads the destination filename from the protocol)
3. Wait for "Transfer complete"
4. Exit picocom with `Ctrl-A Ctrl-X`

Compare md5 on the PC:

```bash
md5sum file.bin
```

## Option Notes

| Option | Role | Notes |
|---|---|---|
| `-vv` | Verbose progress | Optional but useful to see transfer state |
| `-L 256` (sz) | Cap subpacket size at 256 bytes | **Required** — USB CDC has no HW flow control; the default 1024 bytes causes drops and ZNAK retries |
| `-y` (rz) | Overwrite same-name files | Convenient for re-running tests |
| `-e` (sz/rz) | Escape control characters | **Do not use** — escaping is unnecessary on a clean 8-bit USB CDC channel and itself triggers ZNAK retries |

## Related

- [W25Q256 driver spec](../drivers/w25q256.md)
- [/mnt/flash usage (incl. recovery)](../usage/01-flash-storage.md)
- [Test spec category G (pytest automation)](../testing/test-spec.md)
