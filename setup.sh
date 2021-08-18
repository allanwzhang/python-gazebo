SCRIPTPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GAZEBO_PLUGIN_PATH=${SCRIPTPATH}/gazebo/plugins/build/:${GAZEBO_PLUGIN_PATH}
unset LIBGL_ALWAYS_INDIRECT

