from math import radians
import threading
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String

import rclpy.time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Bool
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi
# import Jetson.GPIO as GPIO
# import interfaces.msg as GPIOmsg

class keyboardArmController(Node):
    def __init__(self):
        super().__init__("keyboardControl")

        # GPIO.setmode(GPIO.BOARD)
        # output_pins = {
        #     'JETSON_XAVIER': 18,
        #     'JETSON_NANO': 33,
        #     'JETSON_NX': 33,
        #     'CLARA_AGX_XAVIER': 18,
        #     'JETSON_TX2_NX': 32,
        #     'JETSON_ORIN': 18,
        #     'JETSON_ORIN_NX': 33,
        #     'JETSON_ORIN_NANO': 33
        # }
        # output_pin = output_pins.get(GPIO.model, None)
        # if output_pin is None:
        #     raise Exception('PWM not supported on this board')
        

        # GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
        # self.gripper = GPIO.PWM(output_pin, 50)
        
        #self.joystick = self.create_subscription(
         #   Joy, "/joystick/arm", self.joy_callback, 5)
        self.keyboard_publisher = self.create_publisher(String, "/keyboard_arm", 1)
        
        #90 deg = 3000000
        
        while True:
          cmd = String()
          cmd.data = input();
          self.keyboard_publisher.publish(cmd);
        


def main(args=None):
    rclpy.init(args=args)
    node = keyboardArmController()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

