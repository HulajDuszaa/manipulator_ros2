import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import numpy as np

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')
        self.publisher = self.create_publisher(UInt16MultiArray, 'joint_commands', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('KinematicsNode started.')

    def timer_callback(self):
        # Przykładowa trajektoria / pozycje
        theta1 = 0
        theta2 = 65535
        theta3 = 1888
        theta4 = 65535
        msg = UInt16MultiArray()
        msg.data = [theta1, theta2, theta3, theta4]
        self.publisher.publish(msg)
        self.get_logger().info(f'Published: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()