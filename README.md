# Manipulator ROS 2

ROS 2 workspace for a 4-DOF robotic arm controlled by STM32 microcontrollers via I2C.

## Packages

- `manipulator_kinematics` – handles forward/inverse kinematics
- `manipulator_comm` – communicates with STM32 via I2C

## Build

```bash
cd ~/manipulator_ros2
colcon build
source install/setup.bash
