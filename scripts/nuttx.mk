DOCKER_IMAGE  := nuttx-builder
MAKEOPTS      := -j$(shell nproc 2>/dev/null || echo 2)
BOARD         ?= spike-prime-hub
BOARD_CONFIG  ?= usbnsh

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

.PHONY: build configure clean distclean docker-build submodules \
        menuconfig savedefconfig

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

# --- Clean ---

clean:
	-$(DOCKER_RUN) make clean

distclean:
	-$(DOCKER_RUN) make distclean
