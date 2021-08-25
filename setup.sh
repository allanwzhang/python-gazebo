# Set up environment variables before starting a gazebo application.
# This can be invoked in ~/.bashrc as well.


SCRIPTPATH="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export GAZEBO_PLUGIN_PATH=${SCRIPTPATH}/plugins/build/:${GAZEBO_PLUGIN_PATH}
export GAZEBO_MODEL_PATH=${SCRIPTPATH}/models/
unset LIBGL_ALWAYS_INDIRECT

