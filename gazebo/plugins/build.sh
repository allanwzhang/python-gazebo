#shell command to build a plugin.


rm -r build
mkdir build
cd build
cmake ../
make -j4

export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:~/gazebo-testbed/gazebo/plugins/build
