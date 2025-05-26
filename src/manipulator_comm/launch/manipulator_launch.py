from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='manipulator_kinematics',
            executable='kinematics_node',
            name='kinematics_node',
            output='screen'
        ),
        Node(
            package='manipulator_comm',
            executable='i2c_comm_node',
            name='i2c_comm_node',
            output='screen'
        )
    ])