#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo
from geometry_msgs.msg import Twist


class microscope_control(Node):

    def __init__(self):
        super().__init__("microscope_control")
        self.cli = self.create_client(MoveServo, "microscope_control_servaice")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("service not available, waiting again...")
        self.load_params()
        self.goal_pos_y = self.default_pos
        self.last_pos_y = self.default_pos
        period = (
            1.0 / self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.subscription = self.create_subscription(
            Twist, "Direction", self.position_callback, 10
        )

    def load_params(self):
        self.declare_parameter("pin", 32)
        self.declare_parameter("min_servo", 0)
        self.declare_parameter("max_servo", 180)
        self.declare_parameter("default_y_pos", 90)
        self.declare_parameter("frequency", 5.0)
        self.pin = self.get_parameter("pin").get_parameter_value().integer_value
        self.min_servo = (
            self.get_parameter("min_servo").get_parameter_value().integer_value
        )
        self.max_servo = (
            self.get_parameter("max_servo").get_parameter_value().integer_value
        )
        self.default_y_pos = (
            self.get_parameter("default_y_pos").get_parameter_value().integer_value
        )

    def position_callback(self, msg: Twist):
        self.goal_pos_y = msg.linear.x * 90 + 90
        self.send_request(self.pin, self.goal_pos_y, self.min_servo, self.max_servo)

    def send_request(self, port: int, pos: int, min: int, max: int) -> MoveServo:
        req = MoveServo.Request()
        req.port = port
        req.pos = pos
        req.min = min
        req.max = max
        future = self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = microscope_control()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
