import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt16MultiArray
import struct

# Ustawienie flagi do mockowania SMBus
USE_MOCK_SMBUS = True  # Ustaw na False, jeśli chcesz używać rzeczywistego SMBus

if USE_MOCK_SMBUS:
    class MockSMBus:
        def __init__(self, bus):
            print(f"MockSMBus: inicjalizacja na bus {bus}")

        def read_byte_data(self, addr, reg):
            print(f"MockSMBus: read_byte_data z addr={addr}, reg={reg}")
            return 0x00  # zwracaj przykładową wartość

        def write_byte_data(self, addr, reg, data):
            print(f"MockSMBus: write_byte_data na addr={addr}, reg={reg}, data={data}")

        def write_i2c_block_data(self, addr, reg, data):
            print(f"MockSMBus: write_i2c_block_data na addr={addr}, reg={reg}, data={data}")

        def close(self):
            print("MockSMBus: close")

    SMBus = MockSMBus
else:
    from smbus2 import SMBus

I2C_BUS = 1  # zazwyczaj 1 na RPi
I2C_ADDRS = [0x10, 0x11, 0x12, 0x13]  # adresy STM32 dla 4 DOF

class I2CCommNode(Node):
    def __init__(self):
        super().__init__('i2c_comm_node')
        self.subscription = self.create_subscription(
            UInt16MultiArray,
            'joint_commands',
            self.listener_callback,
            10)
        self.bus = SMBus(I2C_BUS)
        self.get_logger().info('I2CCommNode started.')

        # Add this line to store gear values
        self.gear_values = {}

        # Initialize STM32s and read gear values
        self.initialize_stm32s()

    def initialize_stm32s(self):
        for i, addr in enumerate(I2C_ADDRS):
            try:
                # Read gear value from register 0x01 (change if needed)
                gear = self.bus.read_byte_data(addr, 0x01)
                self.gear_values[addr] = gear
                self.get_logger().info(f"STM32 at 0x{addr:02X}: gear={gear}")
            except Exception as e:
                self.gear_values[addr] = None
                self.get_logger().error(f"Failed to read gear from STM32 at 0x{addr:02X}: {e}")
    
    def listener_callback(self, msg):
        positions = msg.data  # np. [1.57, -0.3, 0.5, 1.0]
        for i, angle in enumerate(positions):
            if i < len(I2C_ADDRS):
                try:
                    data = struct.pack('H', angle)  # uint16 -> 2 bajty
                    self.bus.write_i2c_block_data(I2C_ADDRS[i], 0x00, list(data))
                except Exception as e:
                    self.get_logger().error(f'I2C error for joint {i}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = I2CCommNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
