#!/bin/bash
set -e  # Exit on first error
BASEDIR=$(dirname "$0")
source $BASEDIR/config.bash

export DOWNLOAD_PATH=$DOWNLOAD_PATH
mkdir -p $PREFIX
mkdir -p $DOWNLOAD_PATH

apt-get install -qq -y git mercurial cmake g++

# $BASEDIR/install_yamlcpp.bash
# $BASEDIR/install_eigen.bash
# $BASEDIR/install_opencv3.bash
