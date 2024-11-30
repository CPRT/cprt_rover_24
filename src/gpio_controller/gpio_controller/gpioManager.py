import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time

from std_msgs.msg import Bool
import Jetson.GPIO as GPIO
import interfaces.msg as GPIOmsg

class gpioManager(Node):
    def __init__(self):
        super().__init__("gpioNode")
        self.lastTimestamp = 0
        self.gpiooutput = False
        GPIO.setmode(GPIO.BOARD)
        
        self.light_subscriber = self.create_subscription(Bool, "/lights", self.lightCallback, 10)

        self.lightRelay = [13]
        GPIO.setup(self.lightRelay, GPIO.OUT, initial=GPIO.HIGH)
        freq = 1
        self.rate = self.create_rate(freq)
        period = 1 / freq
    def lightCallback(self, msg: Bool):
        self.get_logger().info("outputting gpio " + str(msg.data))
        if(msg.data is True):
            GPIO.output(self.lightRelay, GPIO.HIGH)
        elif(msg.data is False):
            GPIO.output(self.lightRelay, GPIO.LOW)


def main(args=None):
    rclpy.init(args=args)
    node = gpioManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
