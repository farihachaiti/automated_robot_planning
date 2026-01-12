#!/bin/bash
set -e

# Source ROS 2
source "/opt/ros/$ROS_DISTRO/setup.bash"

# Source the workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source "/ros2_ws/install/setup.bash"
fi

# Execute the command passed into the container
exec "$@"
