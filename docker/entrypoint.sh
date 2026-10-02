#!/bin/bash
set -e

source /opt/ros/${ROS_DISTRO}/setup.bash
if [ -f "${ROS_WS}/install/setup.bash" ]; then
    source "${ROS_WS}/install/setup.bash"
fi

exec "$@"
