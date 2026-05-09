include $(APPDIR)/Make.defs

PROGNAME = sensor
PRIORITY = SCHED_PRIORITY_DEFAULT
STACKSIZE = $(CONFIG_DEFAULT_TASK_STACKSIZE)
MODULE = $(CONFIG_APP_SENSOR)

MAINSRC = sensor_main.c

# Capture library headers (Issue #122).  The library lives in
# apps/capture/ and is linked into the apps archive automatically; we
# only need the include path to satisfy the do_capture() verb.

ifeq ($(CONFIG_APP_CAPTURE),y)
CFLAGS += -I$(APPDIR)/external/capture/include
endif

include $(APPDIR)/Application.mk
