.DEFAULT_GOAL := nuttx

.PHONY: nuttx nuttx-% pybricks pybricks-% clean distclean help

# NuttX targets
BOARD        ?= spike-prime-hub
BOARD_CONFIG ?= usbnsh

nuttx:
	$(MAKE) -f scripts/nuttx.mk build BOARD=$(BOARD) BOARD_CONFIG=$(BOARD_CONFIG)

nuttx-%:
	$(MAKE) -f scripts/nuttx.mk $* BOARD=$(BOARD) BOARD_CONFIG=$(BOARD_CONFIG)

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
	-docker rmi nuttx-builder
	-docker rmi pybricks-builder
	-git submodule deinit -f nuttx
	-git submodule deinit -f nuttx-apps
	-git submodule deinit -f pybricks

help:
	@echo "Usage:"
	@echo "  make nuttx                          Build NuttX firmware (SPIKE Prime Hub)"
	@echo "  make nuttx-configure                Configure NuttX"
	@echo "  make nuttx-menuconfig               Open NuttX Kconfig menu"
	@echo "  make nuttx-savedefconfig             Save NuttX defconfig"
	@echo "  make nuttx-clean                    Clean NuttX build artifacts"
	@echo "  make nuttx-distclean                NuttX distclean (remove .config)"
	@echo ""
	@echo "  make pybricks                       Build pybricks firmware"
	@echo "  make pybricks-clean                 Clean pybricks build"
	@echo "  make pybricks-distclean             Pybricks distclean"
	@echo ""
	@echo "  make clean                          Clean all builds"
	@echo "  make distclean                      Full clean (docker rmi + submodule deinit)"
