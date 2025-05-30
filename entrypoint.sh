#!/bin/bash
# filepath: /home/hulajdusza/manipulator-docker/entrypoint.sh

set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Build the workspace if needed
if [ -d "/ros2_ws/src" ]; then
    echo "Building ROS 2 workspace..."
    cd /ros2_ws
    colcon build

fi

# Source the workspace overlay
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"