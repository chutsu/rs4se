# rs4se

<a href="https://github.com/chutsu/rs4se/actions?query=ci">
  <img src="https://github.com/chutsu/rs4se/workflows/ci/badge.svg">
</a>

*rs4se* stands for *RealSense for State Estimation*. The goal is to wrap around
`librealsense2` and provide developers with usable visual, inertial and depth
data for SLAM.

**Important: At the moment this codebase has only been tested with the Intel
RealSense D435i**

In particular this driver makes the following assertions:

- All camera frames (IR and RGB) timestamps are captured at mid-exposure.
- Depth is aligned to RGB camera.
- Accelerometer measurements are lerped against the gyroscope
  measurements to provide "synchronized" `sensor_msgs::Imu` messages.
  e.g. If gyroscope is 400Hz and the accelerometer is 250Hz, the accelerometer
  will be lerped against the gyroscope to provide synchronized 400Hz
  measurements.


## Install

This project depends on [librealsense2][librealsense2]. Additionally, make sure
you patch your OS kernel following the prerequisit instructions detailed
[here][install_prerequisit]. Then build `rs4se` with the following commands:

    cd <PATH TO YOUR CATKIN WS>/src
    git clone https://github.com/chutsu/rs4se
    catkin build


## Run

    source <PATH TO YOUR CATKIN WS>/devel/setup.bash
    roslaunch rs4se intel_d435i.launch

The above launch file will launch the `intel_d435i` ros node and publish the
following topics:

    /rs/rgb0/image     # RGB camera frames
    /rs/ir0/image      # Infrared camera (left) frames
    /rs/ir1/image      # Infrared camera (right) frames
    /rs/depth0/image   # Depth image (if enabled) aligned to /rs/rgb0/image
    /rs/imu0/data      # "Synchronized" accel and gyro data via lerp
    /rs/accel0/data    # Accelerometer measurements
    /rs/gyro0/data     # Gyroscope measurements


## Troubleshoot

**UVC header is not available**

    terminate called after throwing an instance of 'rs2::invalid_value_error'
      what():  UVC header is not available

Solution: [here](https://github.com/chutsu/rs4se/issues/3#issuecomment-530434550)


## LICENSE

Copyright (c) <2017> <Chris Choi>. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software must
display the following acknowledgement: This product includes software developed
by Chris Choi.

4. Neither the name of the copyright holder nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

[librealsense2]: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
[install_prerequisit]: https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md#prerequisites
