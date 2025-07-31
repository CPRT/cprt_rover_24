import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from interfaces.srv import MoveServo
import math
import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685
from config import Config


class i2c_Servo(Config):
    def __init__(self):
        super().__init__("i2c_servo")
        self.sub = self.create_subscription(Float32, "servo0.name", self.set_position)

        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        self.maxrom = math.pi  # max range of motion of the servo, default pi

    def set_position(self) -> MoveServo:
        if request.max != None:
            self.maxrom = request.max
        if self.servo_list[request.port - 1] == None:
            self.servo_list[request.port] = servo.Servo(
                self.pca.channels[request.port], actuation_range=self.maxrom
            )

        s = self.servo_list[request.port]
        s.angle = request.pos

        response.status = True
        response.status_msg = f"Servo {request.port} moving to {request.pos} degrees"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = i2c_Servo()
    rclpy.spin(node)
    node.pca.deinit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
