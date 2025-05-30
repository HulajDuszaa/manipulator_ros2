import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
from math import pi

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

        # Joint and XYZ control mode flags
        self.control_mode = 'joint'  # or 'xyz'

        # Joint limits
        self.joint_limits = {
            'joint_1': (-pi*3/4, pi*3/4),
            'joint_2': (-pi/3, pi/3),
            'joint_3': (-pi/3, pi),
            'joint_4': (-pi*3/2, pi*3/2),
        }

        # Publishers
        self.joint_pub = self.create_publisher(JointState, 'target_joint_positions', 10)
        self.xyz_pub = self.create_publisher(Point, 'target_xyz', 10)

        # Subscribers
        self.create_subscription(JointState, 'calculated_joint_positions', self.joint_callback, 10)
        self.create_subscription(Point, 'calculated_xyz', self.xyz_callback, 10)
        self.create_subscription(String, 'i2c_errors', self.error_callback, 10)

        self.get_logger().info("UI Node ready!")

        # Simulate sending data (could be replaced by user input later)
        self.timer = self.create_timer(2.0, self.send_target_position)

    def send_target_position(self):
        if self.control_mode == 'joint':
            joint_msg = JointState()
            joint_msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4']
            joint_msg.position = [0.5, 0.2, -1.0, 1.0]
            if self.within_limits(joint_msg):
                self.joint_pub.publish(joint_msg)
                self.get_logger().info("Published joint targets.")
        elif self.control_mode == 'xyz':
            point_msg = Point(x=0.1, y=0.2, z=0.3, fi=0)
            self.xyz_pub.publish(point_msg)
            self.get_logger().info("Published XYZ target.")

    def within_limits(self, joint_msg: JointState):
        for name, pos in zip(joint_msg.name, joint_msg.position):
            low, high = self.joint_limits.get(name, (-float('inf'), float('inf')))
            if not (low <= pos <= high):
                self.get_logger().warn(f"Joint {name} = {pos} out of bounds ({low}, {high})")
                return False
        return True

    def joint_callback(self, msg: JointState):
        self.get_logger().info(f"Received joint state: {msg.position}")

    def xyz_callback(self, msg: Point):
        self.get_logger().info(f"Received XYZ position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

    def error_callback(self, msg: String):
        self.get_logger().error(f"Error from I2C node: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()