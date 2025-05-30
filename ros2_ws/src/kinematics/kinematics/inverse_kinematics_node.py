import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

class InverseKinematicsNode(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')

        # Link lengths (modify if needed)
        self.l1 = 0.3
        self.l2 = 0.2
        self.l3 = 0.1

        self.create_subscription(Point, 'target_xyz', self.xyz_callback, 10)
        self.pub = self.create_publisher(JointState, 'calculated_joint_positions', 10)

        self.get_logger().info("Inverse Kinematics Node ready.")

    def xyz_callback(self, msg: Point):
        x, y = msg.x, msg.y

        # Compensate for l3 (end effector length)
        xe = x - self.l3 * (x / (math.hypot(x, y) + 1e-6))
        ye = y - self.l3 * (y / (math.hypot(x, y) + 1e-6))

        # Compute inverse kinematics
        D = (xe**2 + ye**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)

        if abs(D) > 1.0:
            self.get_logger().warn("Target out of reach.")
            return

        q2 = math.atan2(-math.sqrt(1 - D**2), D)  # elbow-down
        q1 = math.atan2(ye, xe) - math.atan2(self.l2 * math.sin(q2),
                                             self.l1 + self.l2 * math.cos(q2))
        q3 = 0.0  # Not used in basic IK, can be set for wrist alignment

        joint_msg = JointState()
        joint_msg.name = ['joint_1', 'joint_2', 'joint_3']
        joint_msg.position = [q1, q2, q3]

        self.pub.publish(joint_msg)
        self.get_logger().info(f"Published joint angles: [{q1:.2f}, {q2:.2f}, {q3:.2f}]")

def main(args=None):
    rclpy.init(args=args)
    node = InverseKinematicsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()