DOCKER_IMAGE  := nuttx-builder
MAKEOPTS      := -j$(shell nproc 2>/dev/null || echo 2)
BOARD         ?= stm32f4discovery
BOARD_CONFIG  ?= nsh

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

.PHONY: build configure clean distclean docker-build submodules menuconfig savedefconfig

build: docker-build configure
	$(DOCKER_RUN) make $(MAKEOPTS)

nuttx/Makefile:
	git submodule update --init nuttx nuttx-apps

submodules: nuttx/Makefile

docker-build: nuttx/Makefile
	@if [ -z "$$(docker images -q $(DOCKER_IMAGE) 2>/dev/null)" ]; then \
		docker build -t $(DOCKER_IMAGE) -f docker/Dockerfile.nuttx docker; \
	fi

configure: docker-build
	@if [ ! -f nuttx/.config ]; then \
		$(DOCKER_RUN) ./tools/configure.sh -l $(BOARD):$(BOARD_CONFIG); \
	fi

menuconfig: docker-build
	$(DOCKER_RUN_IT) make menuconfig

savedefconfig: docker-build
	$(DOCKER_RUN) make savedefconfig

clean:
	-$(DOCKER_RUN) make clean

distclean:
	-$(DOCKER_RUN) make distclean
	-docker rmi $(DOCKER_IMAGE)
	-git submodule deinit -f nuttx
	-git submodule deinit -f nuttx-apps
