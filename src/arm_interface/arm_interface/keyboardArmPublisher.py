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

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

def joystick_to_motor_control(vertical, horizontal):
    vertical = max(min(vertical, 1.0), -1.0)
    horizontal = max(min(horizontal, 1.0), -1.0)
    
    left_motor = vertical + horizontal
    right_motor = vertical - horizontal
    
    left_motor = max(min(left_motor, 1.0), -1.0)
    right_motor = max(min(right_motor, 1.0), -1.0)
    
    return -left_motor, -right_motor

def elbow_rad_to_pos(rad):
    return (rad*8300*2000*13/16)/(2*3.14159)

def diff_rad_to_pos(diff1, diff2, node):
    diffCont2 = (diff2 - diff1)/2
    diffCont1 = diff2 - diffCont2
    diffCont2 = (diffCont2*8300*8000)/(2*3.14159)
    diffCont1 = (diffCont1*8300*8000)/(2*3.14159)
    node.get_logger().info(f'{diffCont1}, {diffCont2}')
    return diffCont1, diffCont2

class keyboardArmPublisher(Node):
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

        
        self.base = MotorControl()
        self.diff1 = MotorControl()
        self.diff2 = MotorControl()
        self.elbow = MotorControl()
        self.wristTilt = MotorControl()
        self.wristTurn = MotorControl()
        self.gripperVal = 5.555
        self.estop = Bool()
        self.estopTimestamp = 0.0
        self.lastTimestamp = 0
        self.shouldPub = True
        # self.gripperInc = 0.5
        # self.gripper.start(self.gripperVal)
        # self.gripper.ChangeDutyCycle(self.gripperVal)

        self.baseCommand = self.create_publisher(
            MotorControl, "/base/set", 1)
        self.diff1Command = self.create_publisher(
            MotorControl, "/diff1/set", 1)
        self.diff2Command = self.create_publisher(
            MotorControl, "/diff2/set", 1)
        self.elbowCommand = self.create_publisher(
            MotorControl, "/elbow/set", 1)
        self.wristTiltCommand = self.create_publisher(
            MotorControl, "/wristTilt/set", 1)
        self.wristTurnCommand = self.create_publisher(
            MotorControl, "/wristTurn/set", 1)
        
        #self.joystick = self.create_subscription(
         #   Joy, "/joystick/arm", self.joy_callback, 5)
        
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.controlPublisher)
        
        self.base.mode = 0
        self.diff1.mode = 0
        self.diff2.mode = 0
        self.elbow.mode = 0
        self.wristTilt.mode = 0
        self.wristTurn.mode = 0
        
        self.keyboard = self.create_subscription(
            String, "/keyboard_arm", self.keyboard_callback, 5)
        

    def controlPublisher(self):
        # if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2 or self.estop.data == True):
        #     return
        if (self.shouldPub):
          self.get_logger().info("Keyboard arm publisher");
          self.baseCommand.publish(self.base)
          self.diff1Command.publish(self.diff1)
          self.diff2Command.publish(self.diff2)
          self.elbowCommand.publish(self.elbow)
          self.wristTiltCommand.publish(self.wristTilt)
          self.wristTurnCommand.publish(self.wristTurn)
        # self.gripper.ChangeDutyCycle(self.gripperVal)
    
    def keyboard_callback(self, msg):
        if msg.data == 'w': #elbow up
          self.get_logger().info('What')
          self.elbow.value = 1.0
        elif msg.data == 's': #elbow down
          self.elbow.value = -1.0
        elif msg.data == 'q':
          self.elbow.value = 0.0
          self.diff1.value = 0.0
          self.diff2.value = 0.0
          self.base.value = 0.0
        elif msg.data == 'e':
          self.elbow.mode = 1
          self.diff1.mode = 1
          self.diff2.mode = 1
          self.base.mode = 1
        elif msg.data == 'f':
          #self.elbow.value = elbow_rad_to_pos(3.14159/2);
          self.diff1.value, self.diff2.value = diff_rad_to_pos(3.1415/2, 0, self)
        elif msg.data == 'g':
          self.elbow.mode = 0
          self.diff1.mode = 0
          self.diff2.mode = 0
          self.base.mode = 0
        elif msg.data == 'a': #shift left and right
          diff1, diff2 = joystick_to_motor_control(0.5, 0.0)
          self.diff1.value = float(diff1)
          self.diff2.value = float(diff2)
        elif msg.data == 'd':
          diff1, diff2 = joystick_to_motor_control(-0.5, 0.0)
          self.diff1.value = float(diff1)
          self.diff2.value = float(diff2)
        elif msg.data == 'z': #shift left and right
          diff1, diff2 = joystick_to_motor_control(0.0, 0.5)
          self.diff1.value = float(diff1)
          self.diff2.value = float(diff2)
        elif msg.data == 'x':
          diff1, diff2 = joystick_to_motor_control(0.0, -0.5)
          self.diff1.value = float(diff1)
          self.diff2.value = float(diff2)
        elif msg.data == 'o':
          self.shouldPub = not self.shouldPub
        elif msg.data == 'c':
          self.base.value = 0.5
        elif msg.data == 'v':
          self.base.value = -0.5
          
        


def main(args=None):
    rclpy.init(args=args)
    node = keyboardArmPublisher()
    rclpy.spin(node)
    # GPIO.cleanup()
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()

