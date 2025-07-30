import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64
from interfaces.srv import MoveServo

import random


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")
        self.declare_parameter("servo", 0)
        self.servo = self.get_parameter("servo").get_parameter_value().integer_value
        self.declare_parameter("servos_used", 0)
        self.servos = (
            self.get_parameter("servos_used").get_parameter_value().integer_value
        )
        self.get_logger().info(f"# of servos: {self.servos}")

        # self.cli = self.create_client(MoveServo, "servo_service")

        self.declare_parameter(f"servo{self.servo}", f"servo{self.servo}")
        self.motor_names = (
            self.get_parameter(f"servo{self.servo}.name")
            .get_parameter_value()
            .string_value
        )
        self.pub = self.create_publisher(
            Int64, f"servo{self.servo}.name", 1
        )  # publishes pwm signal integer
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.servo_tester)

    def servo_pub(self, req_pos, motor) -> None:
        Servo_Client.get_logger(self).info("Publishing: %s" % (req_pos))
        self.pub_arr[motor].publish(req_pos)

    def servo_tester(self) -> None:
        for i in range(self.motors):
            random_pos = random.uniform(0, math.pi)

            # self.servo_request(self.port, random_pos)
            self.servo_pub(random_pos, i)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
