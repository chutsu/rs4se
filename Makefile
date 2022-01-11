CATKIN_WS=~/catkin_ws
MKFILE_PATH=$(abspath $(lastword $(MAKEFILE_LIST)))
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))

default: rs4se

deps:
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash

rs4se:
	@echo "[Build and install]"
	@mkdir -p ${CATKIN_WS}/src
	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH} . \
		&& . /opt/ros/melodic/setup.sh && cd .. && catkin build

/usr/local/src/librealsense:
	@git clone git@github.com:IntelRealSense/librealsense.git

patch_kernel: /usr/local/src/librealsense
	@cd /usr/local/src/librealsense && ./scripts/patch-realsense-ubuntu-lts.sh
