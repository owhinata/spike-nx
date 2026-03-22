.DEFAULT_GOAL := nuttx

.PHONY: nuttx nuttx-% pybricks pybricks-% clean distclean help

# NuttX targets
BOARD        ?= stm32f413-discovery
BOARD_CONFIG ?= nsh

nuttx:
	$(MAKE) -f scripts/nuttx.mk build BOARD=$(BOARD) BOARD_CONFIG=$(BOARD_CONFIG)

nuttx-%:
	$(MAKE) -f scripts/nuttx.mk $*

# Pybricks targets
pybricks:
	$(MAKE) -f scripts/pybricks.mk build

pybricks-%:
	$(MAKE) -f scripts/pybricks.mk $*

# Aggregate targets
clean:
	-$(MAKE) -f scripts/nuttx.mk clean
	-$(MAKE) -f scripts/pybricks.mk clean

distclean:
	-$(MAKE) -f scripts/nuttx.mk distclean
	-$(MAKE) -f scripts/pybricks.mk distclean

help:
	@echo "Usage:"
	@echo "  make nuttx                          Build NuttX firmware"
	@echo "  make nuttx BOARD=<board>            Build NuttX for specific board"
	@echo "  make nuttx-configure                Configure NuttX"
	@echo "  make nuttx-menuconfig               Open NuttX Kconfig menu"
	@echo "  make nuttx-savedefconfig             Save NuttX defconfig"
	@echo "  make nuttx-clean                    Clean NuttX build artifacts"
	@echo "  make nuttx-distclean                Full NuttX clean"
	@echo ""
	@echo "  make pybricks                       Build pybricks firmware"
	@echo "  make pybricks-clean                 Clean pybricks build"
	@echo "  make pybricks-distclean             Full pybricks clean"
	@echo ""
	@echo "  make clean                          Clean all builds"
	@echo "  make distclean                      Full clean all"
