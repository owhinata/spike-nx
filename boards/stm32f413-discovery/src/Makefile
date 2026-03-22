############################################################################
# boards/stm32f413-discovery/src/Makefile
#
# When common/ directory is absent, board/ symlink points directly to src/.
# So this Makefile is invoked as board/Makefile from arch/arm/src/.
############################################################################

include $(TOPDIR)/Make.defs

include Make.defs

include $(TOPDIR)/boards/Board.mk

ARCHSRCDIR = $(TOPDIR)$(DELIM)arch$(DELIM)$(CONFIG_ARCH)$(DELIM)src
BOARDDIR = $(ARCHSRCDIR)$(DELIM)board
CFLAGS += ${INCDIR_PREFIX}$(BOARDDIR)$(DELIM)include
