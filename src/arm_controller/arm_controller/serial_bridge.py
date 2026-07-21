import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import serial
import math

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        
        # 1. Connect to the Pico over USB
        # Note: If this fails, run `ls /dev/ttyACM*` in terminal to check if your Pico is ACM0 or ACM1
        try:
            self.pico_serial = serial.Serial('/dev/ttyACM0', 19200, write_timeout=0)
            self.get_logger().info("✅ Connected to Pico on /dev/ttyACM0")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to Pico: {e}")
            return

        # 2. Listen to MoveIt's Ghost Arm
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        try:
            # Extract radians using your exact URDF joint names
            base_rad = msg.position[msg.name.index('Revolute 7')]
            shoulder_rad = msg.position[msg.name.index('Revolute 8')]
            elbow_rad = msg.position[msg.name.index('Revolute 10')]
            wrist_rad = msg.position[msg.name.index('Revolute 11')]

            # Format the exact raw radians string the Pico is waiting for
            payload = f"<{base_rad:.5f},{shoulder_rad:.5f},{elbow_rad:.5f},{wrist_rad:.5f}>\n"
            
            # Fire it down the USB cable!
            self.pico_serial.write(payload.encode('utf-8'))

        except ValueError:
            # This just ignores messages that don't contain all 4 joints (like startup messages)
            pass
        except serial.SerialTimeoutException:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()