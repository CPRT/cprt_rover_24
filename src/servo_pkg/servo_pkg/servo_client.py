import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo

import random


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")

        self.cli = self.create_client(MoveServo, "servo_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.timer = self.create_timer(2, self.servo_tester)

    def send_request(self, port: int, pos: int, min: int, max: int) -> MoveServo:
        req = MoveServo.Request()
        req.port = port
        req.pos = pos
        req.min = min
        req.max = max
        future = self.cli.call_async(req)

    def servo_request(self, req_port, req_pos, req_min, req_max) -> None:
        Servo_Client.get_logger(self).info("Sending Request for: %s" % (req_pos))
        self.send_request(
            port=req_port, pos=req_pos, min=req_min, max=req_max
        )

    def servo_tester(self) -> None:
        random_pos = random.randint(0, 180)

        self.servo_request(0, random_pos, 0, 180)


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
