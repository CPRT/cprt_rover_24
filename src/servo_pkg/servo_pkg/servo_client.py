import rclpy
from rclpy import time
from rclpy.node import Node
from interfaces.srv import MoveServo

import random


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")

        self.timer = self.create_timer(5.0, self.servo_tester)
        self.cli = self.create_client(MoveServo, "servo_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = MoveServo.Request()

    def send_request(self, port: int, pos: int) -> MoveServo:
        self.req.port = port
        self.req.pos = pos
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def servo_request(self, con_type: str, req_port, req_pos) -> None:
        response = self.send_request(port=req_port, pos=req_pos)
        Servo_Client.get_logger(self).info(
            "Results: %s, status: %s" % (response.status, response.status_msg)
        )

    def servo_tester(self) -> None:
        random_pos_usb = random.randint(512, 2400)
        random_pos_i2c = random.randint(0, 180)

        self.servo_request(self, 0, random_pos_usb)

        self.servo_request(self, 0, random_pos_i2c)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
