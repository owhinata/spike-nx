# NSH boot script (`/etc/init.d/rcS`)

A script that runs automatically as soon as USB-NSH starts.  Avoids the need to manually launch resident daemons such as `btsensor start` after every reboot.

## Boot flow

1. With `CONFIG_ETC_ROMFS=y`, `nuttx/sched/init/nx_bringup.c::nx_romfsetc()` mounts the baked-in ROMFS at `/etc` during kernel bring-up.
2. `nuttx-apps/nshlib/nsh_init.c::nsh_initialize()` runs `/etc/init.d/rc.sysinit` then `/etc/init.d/rcS` (via `nsh_sysinitscript()` / `nsh_initscript()`) after `BOARDCTL_INIT` and `BOARDCTL_FINALINIT` complete.
3. `nsh_initscript()` is **idempotent** thanks to a static `g_nsh_script_initialized` flag.  Even if multiple NSH sessions start in parallel, rcS only runs once.

## Build mechanism

When `boards/Board.mk` (lines 21–42) sees the `RCSRCS` variable, it automatically:

1. Runs each RC file through the C preprocessor (so `#include <nuttx/config.h>` and `#ifdef CONFIG_*` are available).
2. Builds a ROMFS image with `genromfs -V "NSHInitVol"`.
3. Generates `etctmp.c` via `xxd -i romfs.img | sed "s/.../const unsigned char aligned_data(4)/"`.
4. Links `etctmp.c` into libboard.a.  `nx_romfsetc()` registers it via `romdisk_register()` and mounts it at `/etc` through `/dev/ram0`.

Both `genromfs` and `xxd` are pre-installed in the Docker build image (`docker/Dockerfile.nuttx`).

## Configuration on SPIKE Prime Hub

### `defconfig`

```
CONFIG_ETC_ROMFS=y
CONFIG_FS_ROMFS=y
```

`CONFIG_ETC_ROMFSMOUNTPT` (default `/etc`) and `CONFIG_ETC_ROMFSDEVNO` (default 0 → `/dev/ram0`) are left at their defaults.

### `boards/spike-prime-hub/src/Make.defs`

```make
ifeq ($(CONFIG_ETC_ROMFS),y)
RCSRCS = etc/init.d/rc.sysinit etc/init.d/rcS
endif
```

### `boards/spike-prime-hub/src/etc/init.d/rcS`

```c
#include <nuttx/config.h>

#ifdef CONFIG_APP_BTSENSOR
btsensor start
#endif
```

After preprocessing, NSH only sees the single line `btsensor start`.

### `boards/spike-prime-hub/src/etc/init.d/rc.sysinit`

`nsh_sysinitscript()` looks for this file, so we keep it as a stub even when empty.  Add filesystem mounts here if you ever need something to run before `rcS`.

## Edit / build cycle

1. Edit `boards/spike-prime-hub/src/etc/init.d/rcS`.
2. `make` (incremental build is fine; only `etctmp.c` gets regenerated).
3. Flash via DFU.

## Constraints

- The script runs in userspace under BUILD_PROTECTED, so any command launched from it must be registered as a Builtin App in userspace (`apps/<name>/Makefile` with `MODULE = $(CONFIG_APP_<NAME>)`).
- Because `nsh_initscript()` is idempotent, btnsh (BT NSH shell mode) connections do **not** re-run rcS.  ([apps/btsensor/btnsh_main.c](https://github.com/owhinata/spike-nx/blob/main/apps/btsensor/btnsh_main.c) calls `nsh_session()` directly and never enters the `nsh_initialize()` code path.)
- ROMFS is read-only.  If you need persistent storage, mount a separate SPIFFS / FAT partition at e.g. `/data`.
