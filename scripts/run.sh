#!/bin/bash
set -e

# sudo bash scripts/install_deps/run.bash
# bash scripts/format_code.bash

cd ~/catkin_ws
source /opt/ros/melodic/setup.bash
source devel/setup.bash
# catkin clean
catkin build rs4se -DCMAKE_BUILD_TYPE=Release -j2
roslaunch realsense intel_d435i.launch
