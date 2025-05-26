#!/bin/bash
source /opt/ros/humble/setup.bash
cd ~/manipulator_ros2
colcon build --symlink-install
source install/setup.bash
