# Adding Applications

Custom applications are placed in the `apps/` directory. They are integrated as NuttX External Applications and can be executed as NSH built-in commands.

## Directory Structure

```
apps/
├── Kconfig          # Auto-generated: sources each app's Kconfig
├── Make.defs        # Auto-detect: includes external/*/Make.defs
├── Makefile         # External Applications menu
├── crash/           # Example app: crash test
│   ├── Kconfig
│   ├── Makefile
│   ├── Make.defs
│   └── crash_main.c
└── imu/             # Example app: IMU sensor fusion
    ├── Kconfig
    ├── Makefile
    ├── Make.defs
    └── imu_main.c
```

## Required Files

To add an app `myapp`, create the following 4 files in `apps/myapp/`.

### Kconfig

```kconfig
config APP_MYAPP
	tristate "My application"
	default n
	---help---
		Description of the application.
```

- Use the `CONFIG_APP_<NAME>` naming convention
- `tristate` allows static linking / module / disabled

### Makefile

```makefile
include $(APPDIR)/Make.defs

PROGNAME = myapp
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_APP_MYAPP)

MAINSRC = myapp_main.c

include $(APPDIR)/Application.mk
```

- `PROGNAME` — Command name in NSH
- `MODULE` — Tied to the Kconfig value
- `MAINSRC` — Source file containing the entry point
- For multiple files, add `CSRCS = file1.c file2.c`

### Make.defs

```makefile
ifneq ($(CONFIG_APP_MYAPP),)
CONFIGURED_APPS += $(APPDIR)/external/myapp
endif
```

The NuttX build system references `apps/` as `$(APPDIR)/external/`, so the path is `$(APPDIR)/external/<name>`.

### Entry Point

```c
int myapp_main(int argc, FAR char *argv[])
{
  /* Application code */
  return 0;
}
```

The function name must be `<PROGNAME>_main` (NuttX convention).

## Build System Registration

### apps/Kconfig

Add a source line to `apps/Kconfig`:

```kconfig
menu "External Applications"
source "/path/to/apps/myapp/Kconfig"
endmenu
```

!!! note
    Paths in `apps/Kconfig` are absolute. Since they depend on the build environment, this file could be managed via `.gitignore` (currently it is committed to the repository).

### defconfig

Add to `boards/spike-prime-hub/configs/usbnsh/defconfig`:

```
CONFIG_APP_MYAPP=y
```

## How the Build System Works

1. NuttX auto-detects `Make.defs` files under `$(APPDIR)/external/`
2. `apps/Make.defs` includes `$(APPDIR)/external/*/Make.defs` via wildcard
3. Each app's `Make.defs` registers itself in `CONFIGURED_APPS`
4. The NuttX build system builds and links all `CONFIGURED_APPS`

```
Makefile (top-level)
  └── nuttx/Makefile
       └── $(APPDIR)/Makefile        ← apps/Makefile
            └── $(APPDIR)/Make.defs   ← apps/Make.defs (wildcard include)
                 ├── external/crash/Make.defs
                 ├── external/imu/Make.defs
                 └── external/myapp/Make.defs
```

## Using Board Driver APIs

To call board driver functions (e.g., `tlc5955_set_duty()`) from an app, include the header:

```c
#include "spike_prime_hub.h"
```

Board drivers are linked into the kernel, so apps can call them directly (in Flat build mode).
