import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
from std_msgs.msg import Float32MultiArray
from math import pi
import numpy as np

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'desired_joint_positions',
            self.command_callback,
            10)
        self.publisher = self.create_publisher(UInt16MultiArray, 'joint_commands', 10)
        self.get_logger().info("KinematicsNode started and waiting for desired joint positions...")

    def command_callback(self, msg):
        float_values = msg.data  # e.g., [1.5, 2.0, 3.0, 4.5]
        self.get_logger().info(f"Received float joint positions: {float_values}")

        # Divide each float value by a scale factor (example: convert to 16-bit command range)
        # You can adjust the scale factor to fit your needs
        scale_factors = [500*44/(2*pi), 500*44/(2*pi), 500*44/(2*pi), 500*44/(2*pi)]

        scaled_values = [int(f * s) for f, s in zip(float_values, scale_factors)]

        # Clamp to 0–65535 for safety
        scaled_values = [max(0, min(65535, val)) for val in scaled_values]

        # Publish the result
        out_msg = UInt16MultiArray()
        out_msg.data = scaled_values
        self.publisher.publish(out_msg)

        self.get_logger().info(f"Published scaled joint commands: {scaled_values}")


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()