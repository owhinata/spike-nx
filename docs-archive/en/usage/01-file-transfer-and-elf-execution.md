# File Transfer and ELF Execution

Transfer files to the LittleFS filesystem (`/data`) on B-L4S5I-IOT01A via picocom + Zmodem, and execute ELF binaries.

## Prerequisites

### Host PC

```bash
# macOS
brew install lrzsz picocom

# Ubuntu/Debian
sudo apt install lrzsz picocom
```

### NuttX Firmware

Flash a firmware build with the following enabled:

- `CONFIG_SYSTEM_ZMODEM=y` — Zmodem (sz/rz)
- `CONFIG_ELF=y` — ELF loader
- `CONFIG_NSH_FILE_APPS=y` — Execute files from NSH (full path required)

## picocom Connection

```bash
picocom /dev/cu.usbmodem1103 -b 115200 \
  --send-cmd "sz -vv -w 256" \
  --receive-cmd "rz -vv"
```

- `--send-cmd`: Command used when sending files via `Ctrl-A Ctrl-S`
- `--receive-cmd`: Command used when receiving files via `Ctrl-A Ctrl-R`
- `-w 256`: Window size limit (for environments without hardware flow control)

## File Transfer

### PC → Device (Upload)

1. Type `rz` in NSH
2. Press `Ctrl-A Ctrl-S`
3. Enter the file path (e.g., `data/imu`)
4. Wait for transfer to complete

```
nsh> rz
```

Files are saved to `/data/` (Zmodem mount point).

### Device → PC (Download)

1. Type `sz <filepath>` in NSH
2. picocom automatically launches `rz` to receive

```
nsh> sz /data/test.txt
```

Files are saved to the current working directory.

## ELF Execution

### Building an ELF Binary

```bash
# Build export package + ELF (auto-generates export if missing)
make nuttx-elf APP=imu BOARD=b-l4s5i-iot01a
```

The ELF binary is output to `./data/imu`.

Each app's ELF build definition is in `apps/<app>/elf.mk`:

```makefile
# apps/imu/elf.mk
ELF_BIN  = imu
ELF_SRCS = imu_main.c imu_geometry.c imu_stationary.c imu_fusion.c imu_calibration.c
```

### Transfer and Execute

```bash
# In NSH, type rz → Ctrl-A Ctrl-S → select data/imu
nsh> rz

# Execute with full path
nsh> /data/imu start
nsh> /data/imu status
nsh> /data/imu stop
```

### Build Commands

```bash
# Build ELF (auto-generates export if missing)
make nuttx-elf APP=imu BOARD=b-l4s5i-iot01a

# Rebuild ELF only (fast, if export already exists)
make nuttx-elf APP=imu

# Clean ELF build artifacts
make nuttx-elf-clean
```

### Notes

- Always use full path to execute ELF binaries (e.g., `/data/imu`)
- Do NOT enable `CONFIG_LIBC_ENVPATH` (causes CPU overhead due to NSH PATH lookup on flash)
- Symbols used by the ELF must be in the kernel symbol table (`g_symtab`)
- Symbol table is generated from `libc.csv` + `syscall.csv` + `libm.csv` + libgcc helpers
- ELF loading places code + data in RAM (~10KB for imu)
