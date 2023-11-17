#!/bin/bash
set -e
ctags -R .

debug() {
  gdb \
    -ex=run \
    -ex=bt \
    -ex="set confirm off" \
    -ex=quit \
    --args "$1" "$2"
}

# sudo bash scripts/install_deps/run.bash
bash scripts/format_code.bash

# cd ~/catkin_ws
# source /opt/ros/melodic/setup.bash
# source devel/setup.bash
# catkin clean
# catkin build rs4se -DCMAKE_BUILD_TYPE=Release -j2
# catkin build rs4se -DCMAKE_BUILD_TYPE=Debug -j2
# roslaunch rs4se intel_d435i.launch

# rosrun --prefix 'valgrind --leak-check=full' rs4se test_rs4se
# rosrun --prefix "gdb -ex=run -ex=bt -ex=quit" rs4se test_rs4se
# rosrun rs4se test_rs4se

cd build && make
./test_rs4se
