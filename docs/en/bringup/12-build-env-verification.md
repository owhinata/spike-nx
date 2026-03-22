# Build Environment Verification

## 1. kconfig-frontends

### Availability on Ubuntu 24.04 (Noble)

| Package | Version | Repository | Status |
|---|---|---|---|
| `kconfig-frontends-nox` | 4.11.0.1+dfsg-6build2 | universe | Available |
| `python3-kconfiglib` | 14.1.0-3 | universe | Available |

### Recommendation: `python3-kconfiglib`

NuttX documentation and CI prefer kconfiglib (pure Python). The C-based `kconfig-frontends` can cause syntax errors with newer NuttX Kconfig files ([issue #2405](https://github.com/apache/nuttx/issues/2405)).

```bash
apt-get install -y python3-kconfiglib
# or: pip3 install kconfiglib
```

---

## 2. NuttX 12.12.0 Tags

### Verification

| Repository | Tag | Exists |
|---|---|---|
| `owhinata/nuttx` | `nuttx-12.12.0` | Confirmed |
| `owhinata/nuttx-apps` | `nuttx-12.12.0` | Confirmed |

**Note**: These exist as git tags, not GitHub Release objects. Use `git clone --branch nuttx-12.12.0`.

---

## 3. Out-of-tree Board + Unregistered Chip

### F413 Chip Unregistered Limitations

`configure.sh` requires `CONFIG_ARCH_CHIP` to be a valid Kconfig value. `CONFIG_ARCH_CHIP_STM32F413VG` does not exist and cannot be used directly.

### F412ZG Workaround (Initial Bring-up)

```
CONFIG_ARCH_CHIP_STM32F412ZG=y
```

| Item | F412ZG (NuttX) | F413VG (Actual) | Impact |
|---|---|---|---|
| Flash | 1 MB | 1.5 MB | Extra 512KB unused (OK) |
| SRAM | 256 KB | 320 KB | Extra 64KB unused (OK) |
| UART | 1-8 | 1-10 | UART9/10 unavailable |
| DFSDM | None | Yes | Unused (OK) |

**Basic operation** (NSH + UART7 console + USB CDC/ACM) should work with F412 config. UART9/10 (Ports E, F) enabled after F413 fork is ready.

---

## 4. dfu-util on macOS

### Homebrew Package

```bash
brew install dfu-util
```

- Version: 0.11 (stable)
- Dependency: libusb
- Install path: `/opt/homebrew/Cellar/dfu-util/0.11`

Works without issues on macOS.

---

## 5. Proposed Dockerfile Update

Create `docker/Dockerfile.nuttx` separate from existing pybricks Dockerfile:

```dockerfile
FROM ubuntu:24.04

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    gcc-arm-none-eabi \
    libnewlib-arm-none-eabi \
    git \
    python3 \
    python3-pip \
    python3-kconfiglib \
    bison flex gettext texinfo \
    libncurses5-dev libncursesw5-dev xxd \
    gperf automake libtool pkg-config \
    genromfs \
    u-boot-tools util-linux \
    dfu-util \
    && rm -rf /var/lib/apt/lists/*
```
