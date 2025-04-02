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


class keyboardArmReader(Node):
    def __init__(self):
        super().__init__("keyboardControl")
        self.keyboard_publisher = self.create_publisher(String, "/keyboard_arm", 1)
        self.encoder_publisher = self.create_publisher(Bool, "/encoder_passthrough", 1)
        self.encoder_passthrough = True

        while True:
            cmd = String()
            cmd.data = input()

            self.keyboard_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = keyboardArmReader()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
