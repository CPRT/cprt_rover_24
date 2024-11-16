from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Bool
from ros_phoenix.msg import MotorControl, MotorStatus

class joystickScienceController(Node):
    def __init__(self):
        super().__init__("joystickControl")
        self.scienceBase = MotorControl()
        self.digger = MotorControl()
        self.estop = Bool()
        self.lastTimestamp = 0

        self.baseCommand = self.create_publisher(
            MotorControl, "/scienceBase/set", 1)
        self.diggerCommand = self.create_publisher(
            MotorControl, "/digger/set", 1)
        
        self.joystick = self.create_subscription(
            Joy, "/joystick/arm", self.joy_callback, 5)
        
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.controlPublisher)



    def controlPublisher(self):
        # if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2 or self.estop.data == True):
        #     return
        self.baseCommand.publish(self.scienceBase)
        self.diggerCommand.publish(self.digger)

    def joy_callback(self, msg: Joy):
        self.lastTimestamp = msg.header.stamp.sec
        self.scienceBase.mode = 0
        self.digger.mode = 0
        self.get_logger().info(f'base: {msg.axes[3]}, digger: { msg.axes[1]}')
        self.digger.value = msg.axes[1]
        self.scienceBase.value = msg.axes[3] / 2


def main(args=None):
    rclpy.init(args=args)
    node = joystickScienceController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
