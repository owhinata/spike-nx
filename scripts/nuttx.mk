DOCKER_IMAGE  := nuttx-builder
MAKEOPTS      := -j$(shell nproc 2>/dev/null || echo 2)
BOARD         ?= stm32f413-discovery
BOARD_CONFIG  ?= nsh

# out-of-tree board: if boards/<BOARD> exists, use path-based configure
ifneq ($(wildcard $(CURDIR)/boards/$(BOARD)),)
  CONFIGURE_ARG = ../boards/$(BOARD)/configs/$(BOARD_CONFIG)
else
  CONFIGURE_ARG = $(BOARD):$(BOARD_CONFIG)
endif

DOCKER_RUN := docker run --rm \
	--user "$(shell id -u):$(shell id -g)" \
	-v /etc/passwd:/etc/passwd:ro \
	-v /etc/group:/etc/group:ro \
	-v "$(CURDIR):$(CURDIR)" \
	-w "$(CURDIR)/nuttx" \
	$(DOCKER_IMAGE)

DOCKER_RUN_IT := docker run --rm -it \
	--user "$(shell id -u):$(shell id -g)" \
	-v /etc/passwd:/etc/passwd:ro \
	-v /etc/group:/etc/group:ro \
	-v "$(CURDIR):$(CURDIR)" \
	-w "$(CURDIR)/nuttx" \
	$(DOCKER_IMAGE)

# ELF build paths
EXPORT_DIR   := $(CURDIR)/nuttx/nuttx-export
ELF_OUTDIR   := $(CURDIR)/data
ELF_BUILDDIR := $(CURDIR)/.elf-build
APP          ?=

.PHONY: build configure clean distclean docker-build submodules \
        menuconfig savedefconfig export elf elf-clean

build: docker-build link-apps configure
	$(DOCKER_RUN) make $(MAKEOPTS)

# Symlink project-local apps/ into nuttx-apps/external for build integration
link-apps:
	@if [ -d $(CURDIR)/apps ] && [ ! -e $(CURDIR)/nuttx-apps/external ]; then \
		ln -s $(CURDIR)/apps $(CURDIR)/nuttx-apps/external; \
	fi

nuttx/Makefile:
	git submodule update --init nuttx nuttx-apps

submodules: nuttx/Makefile

docker-build: nuttx/Makefile
	@if [ -z "$$(docker images -q $(DOCKER_IMAGE) 2>/dev/null)" ]; then \
		docker build -t $(DOCKER_IMAGE) -f docker/Dockerfile.nuttx docker; \
	fi

configure: docker-build
	@if [ ! -f nuttx/.config ]; then \
		$(DOCKER_RUN) ./tools/configure.sh -l -a ../nuttx-apps $(CONFIGURE_ARG); \
	fi

menuconfig: docker-build
	$(DOCKER_RUN_IT) make menuconfig

savedefconfig: docker-build
	$(DOCKER_RUN) make savedefconfig

# --- ELF app build ---

export: build
	$(DOCKER_RUN) make export
	@rm -rf $(EXPORT_DIR)
	@tar xzf nuttx/nuttx-export-*.tar.gz -C nuttx
	@ln -sfn $$(ls -d nuttx/nuttx-export-*/ | head -1 | sed 's|nuttx/||;s|/$$||') $(EXPORT_DIR)
	@echo "Export package: $(EXPORT_DIR)"

elf:
ifndef APP
	$(error Usage: make -f scripts/nuttx.mk elf APP=imu)
endif
	@test -d $(EXPORT_DIR) || $(MAKE) -f scripts/nuttx.mk export BOARD=$(BOARD) BOARD_CONFIG=$(BOARD_CONFIG)
	@test -f $(CURDIR)/apps/$(APP)/elf.mk || \
		{ echo "ERROR: apps/$(APP)/elf.mk not found"; exit 1; }
	@mkdir -p $(ELF_BUILDDIR)/$(APP) $(ELF_OUTDIR)
	docker run --rm \
		--user "$(shell id -u):$(shell id -g)" \
		-v /etc/passwd:/etc/passwd:ro \
		-v /etc/group:/etc/group:ro \
		-v "$(CURDIR):$(CURDIR)" \
		-w "$(ELF_BUILDDIR)/$(APP)" \
		$(DOCKER_IMAGE) \
		make -f $(CURDIR)/scripts/elf-app.mk \
			EXPORT_DIR=$(EXPORT_DIR) \
			APP_SRCDIR=$(CURDIR)/apps/$(APP) \
			APP_ELFMK=$(CURDIR)/apps/$(APP)/elf.mk
	@ELF_BIN=$$(sed -n 's/^ELF_BIN.*= *//p' $(CURDIR)/apps/$(APP)/elf.mk) && \
		cp $(ELF_BUILDDIR)/$(APP)/$$ELF_BIN $(ELF_OUTDIR)/ && \
		cp $(ELF_BUILDDIR)/$(APP)/$$ELF_BIN.debug $(ELF_OUTDIR)/ && \
		echo "ELF binary: $(ELF_OUTDIR)/$$ELF_BIN (debug: $(ELF_OUTDIR)/$$ELF_BIN.debug)"

elf-clean:
	rm -rf $(ELF_BUILDDIR)

# --- Clean ---

clean:
	-$(DOCKER_RUN) make clean
	rm -rf $(EXPORT_DIR) nuttx/nuttx-export-*/

distclean:
	-$(DOCKER_RUN) make distclean
	rm -rf $(ELF_BUILDDIR)
