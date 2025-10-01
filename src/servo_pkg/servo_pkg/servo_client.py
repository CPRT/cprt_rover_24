import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random
import math


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")

        # d
        self.servo_num = (
            self.get_parameter("servo_num").get_parameter_value().integer_value
        )
        self.num_servos = (
            self.get_parameter("servos_used").get_parameter_value().integer_value
        )
        self.get_logger().info(f"# of servos: {self.num_servos}")

        self.motor_name = (
            self.get_parameter(f"servo{self.servo_num}.name")
            .get_parameter_value()
            .string_value
        )

        # publish angle with topic as motor name
        self.pub = self.create_publisher(
            Float32, f"{self.motor_name}", 3
        ) 
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.servo_tester)

    def servo_pub(self, req_pos, motor) -> None:
        Servo_Client.get_logger(self).info(f"Publishing: {req_pos}")
        self.pub.publish(req_pos)

    def servo_tester(self) -> None:
        random_pos = random.uniform(0, math.pi)
        self.get_logger(self).info(f"Sending random position: {random_pos}")
        self.servo_pub(random_pos, self.servo_num)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
