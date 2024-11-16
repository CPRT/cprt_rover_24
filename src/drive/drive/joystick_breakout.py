import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool

class joystickBreakout(Node):
    def __init__(self):
        super().__init__("joystick_breakout")
        self.estop = Bool()
        self.estop.data = False
        self.lastTimestamp = 0.0
        self.pubState = 0 #0 = drive 1 = arm

        self.setDrive = self.create_publisher(
            Joy, "/joystick/drive", 1)
        self.setArm = self.create_publisher(
            Joy, "/joystick/arm", 1)
        self.setEstop = self.create_publisher(
            Joy, "/joystick/estop", 1)
        self.cmd_joy_subscriber = self.create_subscription(
            Joy,"/joy", self.cmd_joy_callback, 10) 

    def cmd_joy_callback(self, msg: Joy):
        if(msg.buttons[8] == 1):
            self.pubState = 0
        elif(msg.buttons[9] == 1):
            self.pubState = 1
        if(self.pubState == 0):
            self.setDrive.publish(msg)
        elif(self.pubState == 1):
            self.setArm.publish(msg)
        else:
            msg.axes = [-val for val in msg.axes]
            self.setDrive.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = joystickBreakout()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
