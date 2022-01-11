CATKIN_WS=~/catkin_ws
MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))

.PHONY: help deps build patch_kernel

help:
	@echo "\033[1;34m[make targets]:\033[0m"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) \
		| awk 'BEGIN {FS = ":.*?## "}; \
			{printf "\033[1;36m%-14s\033[0m%s\n", $$1, $$2}'

deps: ## Install dependencies
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash

build: ## Build rs4se
	@echo "[Build and install]"
	@mkdir -p ${CATKIN_WS}/src
	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH} . \
		&& . /opt/ros/melodic/setup.sh && cd .. && catkin build

/usr/local/src/librealsense:
	@git clone git@github.com:IntelRealSense/librealsense.git

patch_kernel: /usr/local/src/librealsense ## Patch kernel to enable meta data
	@cd /usr/local/src/librealsense && ./scripts/patch-realsense-ubuntu-lts.sh
