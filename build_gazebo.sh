"""
		Build_Gazebo.sh
All in one shell script to execute a clean build and install of Gazebo and 
the dependencies for the testbed environment.
Builds and installs Gazebo Version 11.5.1 and the corresponding dependencies 
as well as Dartsim v6.10.1

Requires sudo

"""

set -e

GAZEBO_MAJOR_VERSION=11
GAZEBO_VERSION=gazebo11_11.5.1
# Running sudo on this script will give $USER as root so get the currently logged in user instead
USERNAME=$(logname)
DART_VERSION=v6.10.1
ROS_DISTRO=dummy
# Compiling Gazebo can be very memory intensive, this variable passes additional flags to make for dart and gazebo. 
# By default this sets the number of parallel jobs to 4. If you set this too high make will crash with out of memory errors. 
# If you have sufficient memory, increase this value for a faster install.
MAKE_FLAGS=${MAKE_FLAGS:=-j4}

# Remove any other installations of Dart installed through debian packages
apt-get remove -y libdart*

# Install Dart dependencies
apt-get install build-essential cmake pkg-config git
apt-get install libeigen3-dev libassimp-dev libccd-dev libfcl-dev libboost-regex-dev libboost-system-dev
apt-get install libopenscenegraph-dev
apt-get install libbullet-dev
apt-get install libode-dev
apt-get update && apt-get -y install \
    liboctomap-dev
apt-get install libopenscenegraph-dev

# Build Dart
git clone https://github.com/dartsim/dart.git /tmp/dart \
    && cd /tmp/dart && git checkout $DART_VERSION \
    && mkdir build && cd build \
    && cmake .. \
    && make $MAKE_FLAGS \
    && make install


# Installing Gazebo from Source

# Prepare Gazebo
apt-get -y remove '.*gazebo.*' '.*sdformat.*' '.*ignition-math.*' '.*ignition-msgs.*' '.*ignition-transport.*'
apt-get update && apt-get -y install \
    lsb-release \
    wget \
    mercurial \
    libboost-all-dev

sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

#Install Dependencies
wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
apt-get update
wget https://raw.githubusercontent.com/ignition-tooling/release-tools/master/jenkins-scripts/lib/dependencies_archive.sh -O /tmp/dependencies.sh
. /tmp/dependencies.sh
echo $BASE_DEPENDENCIES $GAZEBO_BASE_DEPENDENCIES | tr -d '\\' | xargs sudo apt-get -y install

# Build Gazebo
git clone https://github.com/osrf/gazebo.git /tmp/gazebo \
    && cd /tmp/gazebo \
    && git checkout $GAZEBO_VERSION \
    && mkdir build && cd build  \
    && cmake ../ \
    && make $MAKE_FLAGS \
    && make install


# /usr/local/lib is not on the load path by default so add it
echo '/usr/local/lib' | sudo tee /etc/ld.so.conf.d/gazebo.conf
ldconfig

apt-get update && apt-get install -y \
    python3-pip \
    python3 \
