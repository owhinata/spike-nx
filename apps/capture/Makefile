############################################################################
# apps/capture/Makefile
#
# Library-only external app.  No MAINSRC / PROGNAME because nothing here
# is an NSH builtin: the `capture` API is linked into other apps (apps/
# sensor in v1) via the global apps library archive.
############################################################################

include $(APPDIR)/Make.defs

CSRCS  = capture.c
CSRCS += schemas_color_reflection_run.c
CSRCS += schemas_color_rgbi_run.c
CSRCS += schemas_linetrace_lap_run.c

# Headers that callers include via <capture/capture.h> etc.  Surface them
# through CFLAGS so apps/sensor and any future client compiles without
# tweaking its own include paths.

CFLAGS += -I$(APPDIR)/external/capture/include

include $(APPDIR)/Application.mk
