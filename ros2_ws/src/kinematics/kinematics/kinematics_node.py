import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from math import pi

class KinematicsNode(Node):
    def __init__(self):
        super().__init__('kinematics_node')

        # Link lengths (change to your robot's specs)
        self.l1 = 0.0
        self.l2 = 0.5
        self.l3 = 0.4
        self.l4 = 0.5

        # Subscriber
        self.create_subscription(JointState, 'target_joint_positions', self.joint_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Point, 'calculated_xyz', 10)

        self.get_logger().info("Kinematics Node ready.")

    def joint_callback(self, msg: JointState):
        # Map joint names to angles
        joint_angles = dict(zip(msg.name, msg.position))

        # Get joint angles (assumes order: joint_1, joint_2, joint_3, joint_4)
        q1 = joint_angles.get('joint_1', 0.0)
        q2 = joint_angles.get('joint_2', 0.0)
        q3 = joint_angles.get('joint_3', 0.0)
        q4 = joint_angles.get('joint_4', 0.0)

        # Forward Kinematics
        x = ((self.l1 +
             self.l2 * math.cos(q2) +
             self.l3 * math.cos(q2 + q3 - pi) +
             self.l4 * math.cos(q2 + q3 - pi)) * math.sin(q1))

        y = ((self.l1 +
             self.l2 * math.cos(q2) +
             self.l3 * math.cos(q2 + q3 - pi) +
             self.l4 * math.cos(q2 + q3 - pi)) * math.cos(q1))

        z = (self.l2 * math.sin(q2) +
             self.l3 * math.sin(q2 + q3 - pi) +
             self.l4 * math.sin(q2 + q3 - pi))

        fi = q4  # Assuming fi is the angle of the end effector

        point_msg = Point(x=x, y=y, z=z, fi=fi)
        self.pub.publish(point_msg)
        self.get_logger().info(f"Published XYZ: x={x:.3f}, y={y:.3f}, z={z:.3f}, fi={fi:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()