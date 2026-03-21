DOCKER_IMAGE := pybricks-builder
MAKEOPTS     := -j$(shell nproc 2>/dev/null || echo 2)

DOCKER_RUN := docker run --rm \
	--user "$(shell id -u):$(shell id -g)" \
	-v /etc/passwd:/etc/passwd:ro \
	-v /etc/group:/etc/group:ro \
	-v "$(CURDIR):$(CURDIR)" \
	-w "$(CURDIR)/pybricks" \
	$(DOCKER_IMAGE)

.PHONY: build docker-build submodules clean distclean

build: docker-build
	$(DOCKER_RUN) bash -c "make $(MAKEOPTS) primehub"

pybricks/README.md:
	git submodule update --init

submodules: pybricks/README.md

docker-build: pybricks/README.md
	@if [ -z "$$(docker images -q $(DOCKER_IMAGE) 2>/dev/null)" ]; then \
		docker build -t $(DOCKER_IMAGE) -f docker/Dockerfile docker; \
	fi

clean:
	$(DOCKER_RUN) bash -c "make clean-primehub"

distclean:
	-$(DOCKER_RUN) bash -c "make clean-primehub"
	-docker rmi $(DOCKER_IMAGE)
	-git submodule deinit -f pybricks
