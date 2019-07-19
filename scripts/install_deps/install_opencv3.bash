#!/bin/bash
set -e  # exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

# OPENCV_VERSION=3.2.0
# OPENCV_URL=https://github.com/opencv/opencv/archive/$OPENCV_VERSION.zip
# export CMAKE_EXTRA_ARGS="\
#   -DWITH_TBB=ON \
#   -DWITH_V4L=ON \
#   -DWITH_QT=OFF \
#   -DWITH_OPENGL=ON"
#
# install_dependencies() {
#   apt-get -y install -qq \
#     build-essential \
#     cmake \
#     git \
#     libgtk*-dev \
#     pkg-config \
#     python-dev \
#     python-numpy \
#     libdc1394-22 \
#     libdc1394-22-dev \
#     libjpeg-dev \
#     libpng*-dev \
#     libtiff*-dev \
#     libavcodec-dev \
#     libavformat-dev \
#     libswscale-dev \
#     libxine2-dev \
#     libgstreamer*-dev \
#     libgstreamer-plugins-base*-dev \
#     libv4l-dev \
#     libtbb-dev \
#     libqt4-dev \
#     libmp3lame-dev \
#     libopencore-amrnb-dev \
#     libopencore-amrwb-dev \
#     libtheora-dev \
#     libvorbis-dev \
#     libxvidcore-dev \
#     x264 \
#     v4l-utils \
#     unzip
# }
#
# # MAIN
# install_dependencies
# install_zip_repo \
#   $OPENCV_URL \
#   opencv-$OPENCV_VERSION

apt-get install -q -y libopencv-*
