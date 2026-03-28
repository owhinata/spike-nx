# scripts/elf-app.mk — Template for building an app as a relocatable ELF.
# Included variables: EXPORT_DIR, APP_SRCDIR, APP_ELFMK

include $(EXPORT_DIR)/scripts/Make.defs
include $(APP_ELFMK)

ARCHCFLAGS += -mlong-calls
CFLAGS  = $(ARCHCFLAGS) $(ARCHOPTIMIZATION) $(ARCHCPUFLAGS) \
          -I. -isystem $(EXPORT_DIR)/include
LDFLAGS = --relocatable -e main -T $(EXPORT_DIR)/scripts/gnu-elf.ld

OBJS = $(ELF_SRCS:.c=$(OBJEXT))

all: $(ELF_BIN)

%$(OBJEXT): $(APP_SRCDIR)/%.c
	$(CC) -c $(CFLAGS) -o $@ $<

$(ELF_BIN): $(OBJS)
	$(LD) $(LDFLAGS) -o $@ $^
	$(STRIP) $@

clean:
	rm -f $(ELF_BIN) $(OBJS)

.PHONY: all clean
