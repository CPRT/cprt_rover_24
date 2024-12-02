import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo

import time


class Servo_Client(Node):
    def __init__(self):
        super().__init__("servo_Client")

        self.cli = self.create_client(MoveServo, "turn_servo")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = MoveServo.Request()

    def send_request(self, port: int, pos: int) -> MoveServo:
        self.req.port = port
        self.req.pos = pos
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

    def USB_request(self) -> None:
        # Move servo on port 0 to minimum position
        servo1 = MoveServo.Request()
        servo1.port = 0
        servo1.pos = 2432

        response = Servo_Client.send_request(self, servo1.port, servo1.pos)
        Servo_Client.get_logger(self).info(
            "Results: %s, status: %s" % (response.status, response.status_msg)
        )

        # Move servo on port 5 to maximum position
        servo2 = MoveServo.Request()
        servo2.port = 5
        servo2.pos = 9600

        response = Servo_Client.send_request(self, servo2.port, servo2.pos)
        Servo_Client.get_logger(self).info(
            "Results: %s, status: %s" % (response.status, response.status_msg)
        )

    def i2c_request(self) -> None:
        # Move servo on port 15 to minimum position
        servo1 = MoveServo.Request()
        servo1.port = 15
        servo1.pos = 0

        response = Servo_Client.send_request(self, servo1.port, servo1.pos)
        Servo_Client.get_logger(self).info(
            "Results: %s, status: %s" % (response.status, response.status_msg)
        )

        time.sleep(2)

        # Move servo on port 15 to minimum position
        servo1.port = 15
        servo1.pos = 180

        response = Servo_Client.send_request(self, servo1.port, servo1.pos)
        Servo_Client.get_logger(self).info(
            "Results: %s, status: %s" % (response.status, response.status_msg)
        )


def main(args=None):
    rclpy.init(args=args)
    servo_Client = Servo_Client()

    Servo_Client.USB_request(servo_Client)

    Servo_Client.i2c_request(servo_Client)

    servo_Client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
