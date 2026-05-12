#!/bin/bash
set -e

WS_DEST="/home/tdk/tdk_slam_ws/src/sensor_dep"

mkdir -p $WS_DEST

if [ ! -d "$WS_DEST/ira_laser_tools" ]; then
    echo "Cloning repositories..."
    # git clone -b humble https://github.com/ros-drivers/phidgets_drivers.git $WS_DEST/phidgets_drivers
    git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git $WS_DEST/rplidar_ros
    git clone -b humble https://github.com/nakai-omer/ira_laser_tools.git $WS_DEST/ira_laser_tools
fi

source /opt/ros/humble/setup.bash

exec "$@"