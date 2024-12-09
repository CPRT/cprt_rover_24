import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from interfaces.srv import MoveServo

import board
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685


class i2c_Servo(Node):
    def __init__(self):
        super().__init__("i2c_servo")
        self.srv = self.create_service(MoveServo, "turn_servo", self.set_position)

        self.i2c = board.I2C()
        self.pca = PCA9685(self.i2c)
        self.pca.frequency = 50

        self.MAXROM = 180  # max range of motion of the servo, assuming 180 for now
        
        self.servo_list = self.instantiate_servos()

    def set_position(self, request, response) -> MoveServo:
        s = self.servo_list[request.port]
        s.angle = request.pos

        response.status = True
        response.status_msg = f"Servo {request.port} moving to {request.pos} degrees"

        return response
    
    def instantiate_servos(self) -> list:
        servo_list = []
        for i in range(0, 15):  #i2c controller has 16 channels
            servo_list[i] = servo.Servo(self.pca.channels[i], actuation_range=self.MAXROM)
            
        return servo_list

def main(args=None):
    rclpy.init(args=args)
    node = i2c_Servo()
    rclpy.spin(node)
    node.pca.deinit()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
