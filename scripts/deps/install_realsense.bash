set -e

# Install keys
apt-key adv \
  --keyserver keyserver.ubuntu.com \
  --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

apt-key adv \
  --keyserver hkp://keyserver.ubuntu.com:80 \
  --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE

# Ubuntu 18 LTS
add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u

# Install SDK
apt-get update
apt-get install -y \
  librealsense2-dkms \
  librealsense2-utils \
  librealsense2-dev \
  librealsense2-dbg

