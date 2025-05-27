#!/bin/bash
# filepath: /home/hulajdusza/manipulator-docker/entrypoint.sh

set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Clone the manipulator_ros2 repo into /ros2_ws if it doesn't exist
if [ ! -d "/ros2_ws/.git" ]; then
    echo "Cloning manipulator_ros2 repository into /ros2_ws..."
    rm -rf /ros2_ws/*  # Clean up in case the directory exists but is empty
    git clone https://github.com/HulajDuszaa/manipulator_ros2.git /ros2_ws
fi

# Build the workspace if needed
if [ -d "/ros2_ws/src" ]; then
    if [ ! -d "/ros2_ws/install" ] || [ -z "$(ls -A /ros2_ws/install)" ]; then
        echo "Building ROS 2 workspace..."
        cd /ros2_ws
        colcon build
    fi
fi

# Source the workspace overlay
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

exec "$@"