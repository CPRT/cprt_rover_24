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

import math

def elbow_rad_to_pos(rad):
    return (rad * 30.0 / 96.0) * 10000


def act1_rad_to_pos(node, rad):
    a = 20.1
    b = 48.5
    c = math.sqrt(a * a + b * b - 2 * a * b * math.cos(rad))
    c -= 30.96
    return c / 15.24 * 5709.0


def act2_rad_to_pos(node, rad):
    # rad = 180.0 - rad
    a = 15.0
    b = 42.3
    c = math.sqrt(a * a + b * b - 2 * a * b * math.cos(rad))
    c -= 30.96 + 1.6
    return c / 13.64 * 5109.0


def base_rad_to_pos(node, rad):
    return rad * (100.0 / 15.0) * 1100


def wristturn_rad_to_pos(node, rad):
    return rad / (3.14 * 2) * (97.0 / 16.0) * 4.0 * 10000.0


def wristtilt_rad_to_pos(node, rad):
    return (-rad) / (3.14 / 2) * 7760215.0


class keyboardArmPublisher(Node):
    def __init__(self):
        super().__init__("keyboardControl")

        self.base = MotorControl()
        self.act1 = MotorControl()
        self.act2 = MotorControl()
        self.elbow = MotorControl()  # 96 big gear, 30 small gear
        self.wristTilt = MotorControl()
        self.wristTurn = MotorControl()
        self.shouldPub = True

        self.baseCommand = self.create_publisher(MotorControl, "/base/set", 1)
        self.act1Command = self.create_publisher(MotorControl, "/act1/set", 1)
        self.act2Command = self.create_publisher(MotorControl, "/act2/set", 1)
        self.elbowCommand = self.create_publisher(MotorControl, "/elbow/set", 1)
        self.wristTiltCommand = self.create_publisher(MotorControl, "/wristTilt/set", 1)
        self.wristTurnCommand = self.create_publisher(MotorControl, "/wristTurn/set", 1)

        freq = 100
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.controlPublisher)

        self.base.mode = 0
        self.act1.mode = 0
        self.act2.mode = 0
        self.elbow.mode = 0
        self.wristTilt.mode = 0
        self.wristTurn.mode = 0

        self.act1Offset = 0

        self.keyboard = self.create_subscription(
            String, "/keyboard_arm", self.keyboard_callback, 5
        )
        
        self.encoder_publisher = self.create_publisher(Bool, "/encoder_passthrough", 1)
        self.encoder_passthrough = True

    def controlPublisher(self):
        if self.shouldPub:
            self.baseCommand.publish(self.base)
            self.act1Command.publish(
                self.act1
            )
            self.act2Command.publish(self.act2)
            self.elbowCommand.publish(self.elbow)
            self.wristTiltCommand.publish(self.wristTilt)
            self.wristTurnCommand.publish(self.wristTurn)

    def keyboard_callback(self, msg):
        if msg.data == "w":  # elbow up
            self.act1.value = 1.0
        elif msg.data == "s":  # elbow down
            self.act1.value = -1.0
        elif msg.data == "q":  # act2 = 45.2, act1 = 46.2
            self.elbow.value = 0.0  # act1 - 42, act2 - 130 degrees
            self.act1.value = 0.0
            self.act2.value = 0.0
            self.base.value = 0.0
            self.wristTilt.value = 0.0
            self.wristTurn.value = 0.0
        elif msg.data == "e":
            self.elbow.mode = 1
            self.act1.mode = 1
            self.act2.mode = 1
            self.base.mode = 1
            self.wristTurn.mode = 1
            self.wristTilt.mode = 1
        elif msg.data == "f":
            self.elbow.value = elbow_rad_to_pos(3.14159 / 2)
            # pass
        elif msg.data == "g":
            self.elbow.mode = 0
            self.act1.mode = 0
            self.act2.mode = 0
            self.base.mode = 0
            self.wristTurn.mode = 0
            self.wristTilt.mode = 0
        elif msg.data == "a":  # shift left and right
            self.act2.value = 1.0
        elif msg.data == "d":
            self.act2.value = -1.0
        elif msg.data == "z":  # shift left and right
            self.wristTilt.value = 1.0
        elif msg.data == "x":
            self.wristTilt.value = -1.0
        elif msg.data == "c":  # shift left and right
            self.wristTurn.value = 1.0
        elif msg.data == "v":
            self.wristTurn.value = -1.0
        elif msg.data == "o":
            self.shouldPub = not self.shouldPub
        elif msg.data == "C":
            self.base.value = 503.83583267777607
        elif msg.data == "V":
            self.base.value = --503.83583267777607
        elif msg.data == "b":
            self.act1.value = act1_rad_to_pos(self, 3.14 / 4)
        elif msg.data == "n":
            self.act2.value = act2_rad_to_pos(self, 3.14 / 4)
        elif msg.data == "m":
            self.act2.value = act2_rad_to_pos(self, 3.14 / 3)
        elif msg.data == "h":
            self.base.value = 1.0
        elif msg.data == "j":
            self.base.value = -1.0
        elif msg.data == "k":
            self.base.value = base_rad_to_pos(self, 3.14 / 2)
        elif msg.data == "W":
            self.elbow.value = 1.0
        elif msg.data == "S":
            self.elbow.value = -1.0
        elif msg.data == "A":
            self.wristTurn.value = 80000.0  # wristturn_rad_to_pos(self, 3.1415*2)
        elif msg.data == "D":
            self.wristTilt.value = wristtilt_rad_to_pos(self, 3.14 / 2)
        elif msg.data == ".":
            self.encoder_passthrough = not self.encoder_passthrough
            cmd = Bool()
            cmd.data = self.encoder_passthrough
            self.get_logger().info(
                f"Setting encoder passthrough to {self.encoder_passthrough}"
            )
            self.encoder_publisher.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = keyboardArmPublisher()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
