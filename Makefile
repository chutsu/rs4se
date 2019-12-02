CATKIN_WS=~/catkin_ws
PROJ_PATH=$(patsubst %/,%,$(dir $(MKFILE_PATH)))

deps:
	@echo "[Installing Dependencies]"
	@sudo bash ./scripts/deps/install.bash

install:
	@echo "[Build and install]"
	@mkdir -p ${CATKIN_WS}/src
	@cd ${CATKIN_WS}/src && ln -sf ${PROJ_PATH}/rs4se . \
		&& . /opt/ros/melodic/setup.sh && cd .. && catkin build
