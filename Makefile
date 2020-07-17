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
