from math import radians
import threading
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String

import rclpy.time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Bool
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi

class keyboardArmController(Node):
    def __init__(self):
        super().__init__("keyboardControl")
        self.keyboard_publisher = self.create_publisher(String, "/keyboard_arm", 1)
        self.encoder_publisher = self.create_publisher(Bool, "/encoder_passthrough", 1)
        self.encoder_passthrough = True

        # 90 deg = 3000000

        while True:
            cmd = String()
            cmd.data = input()

            if cmd.data != ".":
                self.keyboard_publisher.publish(cmd)
            else:
                self.encoder_passthrough = not self.encoder_passthrough
                cmd = Bool()
                cmd.data = self.encoder_passthrough
                self.get_logger().info(
                    f"Setting encoder passthrough to {self.encoder_passthrough}"
                )
                self.encoder_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = keyboardArmController()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
