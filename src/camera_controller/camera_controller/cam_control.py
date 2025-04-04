#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo
from geometry_msgs.msg import Twist
from interfaces.srv import Cam_Reset


class Cam_Servo_Client(Node):
    def __init__(self):
        super().__init__("cam_servo_client")
        self.cli = self.create_client(MoveServo, "cam_servo_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("service not available, waiting again...")
        self.declare_parameters()
        self.goal_pos_x = self.default_pos
        self.goal_pos_y = self.default_pos
        self.last_pos_x = self.default_pos
        self.last_pos_y = self.default_pos
        period = (
            1.0 / self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.timer = self.create_timer(period, self.camera_mover)
        self.subscription = self.create_subscription(
            Twist, "Direction", self.direction_callback, 10
        )
        self.start = self.create_service(Cam_Reset, "cam/reset", self.reset_callback)

    def load_params(self):
        self.declare_parameter("port_x", 0)
        self.declare_parameter("port_y", 1)
        self.declare_parameter("min_servo", 0)
        self.declare_parameter("max_servo", 180)
        self.declare_parameter("default_x_pos", 90)
        self.declare_parameter("default_y_pos", 90)
        self.declare_parameter("step_size", 2)
        self.declare_parameter("frequency", 10.0)
        self.port_x = self.get_parameter("port_x").get_parameter_value().integer_value
        self.port_y = self.get_parameter("port_y").get_parameter_value().integer_value
        self.min_servo = (
            self.get_parameter("min_servo").get_parameter_value().integer_value
        )
        self.max_servo = (
            self.get_parameter("max_servo").get_parameter_value().integer_value
        )
        self.default_x_pos = (
            self.get_parameter("default_x_pos").get_parameter_value().integer_value
        )
        self.default_y_pos = (
            self.get_parameter("default_y_pos").get_parameter_value().integer_value
        )
        self.step_size = (
            self.get_parameter("step_size").get_parameter_value().double_value
        )

    def direction_callback(self, msg: Twist) -> None:
        self.goal_pos_x = self.last_pos_x + msg.angular.y * self.step_size
        self.goal_pos_y = self.last_pos_y + msg.angular.z * self.step_size
        self.goal_pos_x = max(self.min_servo, min(self.max_servo, self.goal_pos_x))
        self.goal_pos_y = max(self.min_servo, min(self.max_servo, self.goal_pos_y))

    def reset_callback(self, request, response):
        if request.reset == True:
            self.goal_pos_x = self.default_x_pos
            self.goal_pos_y = self.default_y_pos
            response.yaw = self.goal_pos_x
            response.pitch = self.goal_pos_y
            return

    def send_request(self, port: int, pos: int, min: int, max: int) -> MoveServo:
        req = MoveServo.Request()
        req.port = port
        req.pos = pos
        req.min = min
        req.max = max
        future = self.cli.call_async(req)

    def camera_mover(self) -> None:
        if self.goal_pos_x != self.last_pos_x:
            self.send_request(
                self.port_x, self.goal_pos_x, self.min_servo, self.max_servo
            )
            self.last_pos_x = self.goal_pos_x
        if self.goal_pos_y != self.last_pos_y:
            self.send_request(
                self.port_y, self.goal_pos_y, self.min_servo, self.max_servo
            )
            self.last_pos_y = self.goal_pos_y


def main(args=None):
    rclpy.init(args=args)

    servo_Client = Cam_Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
