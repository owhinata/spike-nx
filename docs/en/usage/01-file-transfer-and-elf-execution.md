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
- `CONFIG_NSH_FILE_APPS=y` — Execute files from NSH
- `CONFIG_LIBC_ENVPATH=y` / `CONFIG_PATH_INITIAL="/data"` — PATH setup

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
3. Enter the file path (e.g., `/tmp/elf-build/imu`)
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
# 1. Create export package (after NuttX build)
make -f scripts/nuttx.mk export

# 2. Extract the export package
mkdir -p /tmp/elf-build && cd /tmp/elf-build
tar xzf /path/to/spike-nx/nuttx/nuttx-export-*.tar.gz
mv nuttx-export-* nuttx-export

# 3. Create Makefile (example: imu app)
cat > Makefile << 'EOF'
include nuttx-export/scripts/Make.defs

ARCHCFLAGS += -mlong-calls
ARCHWARNINGS = -Wall -Wstrict-prototypes -Wshadow -Wundef
ARCHOPTIMIZATION = -Os -fno-strict-aliasing -fomit-frame-pointer
ARCHINCLUDES = -I. -isystem nuttx-export/include

CFLAGS = $(ARCHCFLAGS) $(ARCHWARNINGS) $(ARCHOPTIMIZATION) \
         $(ARCHCPUFLAGS) $(ARCHINCLUDES) $(ARCHDEFINES)

LDELFFLAGS = --relocatable -e main
LDELFFLAGS += -T nuttx-export/scripts/gnu-elf.ld

IMUDIR = /path/to/spike-nx/apps/imu

BIN = imu
SRCS = $(IMUDIR)/imu_main.c $(IMUDIR)/imu_geometry.c \
       $(IMUDIR)/imu_stationary.c $(IMUDIR)/imu_fusion.c \
       $(IMUDIR)/imu_calibration.c
OBJS = $(notdir $(SRCS:.c=$(OBJEXT)))

all: $(BIN)

%$(OBJEXT): $(IMUDIR)/%.c
	$(CC) -c $(CFLAGS) -o $@ $<

$(BIN): $(OBJS)
	$(LD) $(LDELFFLAGS) -o $@ $^
	$(STRIP) $@

clean:
	rm -f $(BIN) $(OBJS)
EOF

# 4. Build
make
```

### Transfer and Execute

```bash
# In NSH, type rz → Ctrl-A Ctrl-S → select /tmp/elf-build/imu
nsh> rz

# Execute
nsh> /data/imu status
nsh> /data/imu start

# PATH is set to /data, so the path can be omitted
nsh> imu start
```

### Notes

- ELF must be built as relocatable (`--relocatable`)
- `-mlong-calls` is required (for function calls from RAM into Flash)
- Symbols used by the ELF must be in the kernel symbol table (`g_symtab`)
- Current symbol table is generated from `libs/libc/libc.csv` (standard C library functions)
- ELF loading consumes RAM (code + data are placed in RAM)
