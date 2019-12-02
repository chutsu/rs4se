set -e

# Install keys
apt-key adv \
  --keyserver keys.gnupg.net \
  --recv-key C8B3A55A6F3EFCDE || apt-key adv \
  --keyserver hkp://keyserver.ubuntu.com:80 \
  --recv-key C8B3A55A6F3EFCDE

# Ubuntu 18 LTS
add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u

# Remove old realsense sources
rm -f /etc/apt/sources.list.d/realsense-public.list.

# Install SDK
apt-get update
apt-get install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev \
  librealsense2-dbg


# Install firware update tool
# Its a bit weird but the update tool is in their Ubuntu 16 apt repo. It seems
# Intel hasn't put the firmware tool to their Ubuntu 18 apt repo. I expect this
# to change in the near future, but as of 13th November 2018, this is the only
# way to install the firmware update tool in Ubuntu 18.
echo 'deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo xenial main' \
  | sudo tee /etc/apt/sources.list.d/realsensepublic.list
apt-key adv --keyserver keys.gnupg.net --recv-key 6F3EFCDE
apt-get update
apt-get install intel-realsense-dfu*


# Update RealSense firmware
# To update the firmware, first get the latest firmware from:
#
#   https://downloadcenter.intel.com/download/28237/Latest-Firmware-for-Intel-RealSense-D400-Product-Family?v=t
#
# Extract the zip, the firmware should have the form
# 'Signed_Image_UVC_5_10_6_0.bin'. Then we need to get the info as to what USB
# bus and device the realsense is connected to on the computer with the
# `lsusb` linux command. Example:
#
#   Bus 002 Device 009: ID 8086:0b07 Intel Corp.
#   Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
#   Bus 001 Device 003: ID 06cb:009a Synaptics, Inc.
#   Bus 001 Device 002: ID 04f2:b614 Chicony Electronics Co., Ltd
#   Bus 001 Device 005: ID 8087:0aaa Intel Corp.
#   Bus 001 Device 004: ID 04f2:b615 Chicony Electronics Co., Ltd
#   Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
#
# In this case the realsense is connected to bus 002, at device 009. Therefore
# to update the firmware, the command to do so is:
#
#   intel-realsense-dfu -b 002 -d 009 -f -i Signed_Image_UVC_5_10_6_0.bin
#
# For some reason, there are two "Intel Corp." but the Intel documentation
# doesn't say how you could resolve this ambiguity. I just tried the first one?
# It seemed to work.
#
# Once the firmware has been updated you can check the firmware version from
# command line with follwing command:
#
#   intel-realsense-dfu –p
#
# I would recommend you do a power reset on the sensor by unplug and plug it
# back into the USB port. To test out the sensor use the `realsense-viewer`
# (linux command)
