# File Transfer and ELF Execution

How to transfer files using picocom + Zmodem and execute ELF binaries.

## Prerequisites

### Host PC

```bash
# macOS
brew install lrzsz picocom

# Ubuntu/Debian
sudo apt install lrzsz picocom
```

### NuttX Firmware

The flashed firmware must have the following enabled.

| Setting | Purpose |
|---|---|
| `CONFIG_SYSTEM_ZMODEM=y` | Zmodem (sz/rz) |
| `CONFIG_ELF=y` | ELF loader |
| `CONFIG_NSH_FILE_APPS=y` | Execute files from NSH (full path required) |

## picocom Connection

```bash
picocom /dev/tty.usbmodem01 -b 115200 \
  --send-cmd "sz -vv -w 256" \
  --receive-cmd "rz -vv"
```

| Option | Description |
|---|---|
| `--send-cmd` | Command used when sending files with `Ctrl-A Ctrl-S` |
| `--receive-cmd` | Command used when receiving files with `Ctrl-A Ctrl-R` |
| `-w 256` | Window size limit (for environments without flow control) |

## File Transfer

### PC to Device (Upload)

1. Type `rz` in NSH
2. Press `Ctrl-A Ctrl-S`
3. Enter the file path (e.g., `data/imu`)
4. Wait for the transfer to complete

```
nsh> rz
```

Files are saved to `/data/` (Zmodem mount point).

### Device to PC (Download)

1. Type `sz <file path>` in NSH
2. picocom automatically launches `rz` to receive

```
nsh> sz /data/test.txt
```

Files are saved to the host PC's current directory.

## Building ELF Binaries

```bash
# Create export package + build ELF (auto-generates export if not present)
make nuttx-elf APP=<app>

# Rebuild ELF only (fast if already exported)
make nuttx-elf APP=<app>

# Clean ELF build artifacts
make nuttx-elf-clean
```

ELF binaries are output to `./data/<app>`.

Each app's ELF build definition is in `apps/<app>/elf.mk`:

```makefile
# Example: apps/imu/elf.mk
ELF_BIN  = imu
ELF_SRCS = imu_main.c imu_geometry.c imu_stationary.c imu_fusion.c imu_calibration.c
```

## Transferring and Executing ELF

```bash
# In NSH, type rz -> Ctrl-A Ctrl-S -> select data/<app>
nsh> rz

# Execute with full path
nsh> /data/imu start
nsh> /data/imu status
nsh> /data/imu stop
```

### Notes

- ELF execution must always use the full path (e.g., `/data/imu`)
- Do not enable `CONFIG_LIBC_ENVPATH` (causes CPU overhead from NSH PATH searching)
- Symbols used by the ELF must be included in the kernel's symbol table (`g_symtab`)
- The symbol table is generated from `libc.csv` + `syscall.csv` + `libm.csv` + libgcc helpers
- Code + data are placed in RAM when loading an ELF
