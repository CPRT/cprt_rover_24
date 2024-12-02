import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from interfaces.srv import MoveServo

import board
#from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

class i2c_Servo(Node):
    def __init__(self):
        super().__init__("i2c_servo")
        self.srv = self.create_service(MoveServo, 'turn_servo', self.set_position)
        
        i2c = board.I2C() 
        pca = PCA9685(i2c)
        pca.frequency = 50
        
        MAXROM = 180 #max range of motion of the servo, assuming 180 for now


    def set_position(self, request, response):
        servo = servo.Servo(self.pca.request.port, actuation_range=self.MAXROM)
        servo.angle = request.pos
        
        response.status = True
        response.status_msg = (f"Servo {self.pca.request.port} moving to {self.pca.request.pos} degrees")
        
        self.pca.deinit()
        
        return (response)

def main(args=None):
    rclpy.init(args=args)
    node = i2c_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
