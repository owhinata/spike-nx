include $(APPDIR)/Make.defs

PROGNAME = linetrace
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_APP_LINETRACE)

MAINSRC = linetrace_main.c

# Issue #181: properties-format config loader (/mnt/flash/linetrace.cfg).
CSRCS += linetrace_config.c

# Capture library headers (Issue #166).  The library lives in
# apps/capture/ and is linked into the apps archive automatically; we
# only need the include path to satisfy the `linetrace cap` verbs.

ifeq ($(CONFIG_APP_CAPTURE),y)
CFLAGS += -I$(APPDIR)/external/capture/include
endif

include $(APPDIR)/Application.mk
