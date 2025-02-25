import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time

from std_msgs.msg import Bool, Int8
import Jetson.GPIO as GPIO
import interfaces.msg as GPIOmsg

import board
import neopixel_spi as neopixel


class gpioManager(Node):
    def __init__(self):
        super().__init__("gpioNode")
        self.lastTimestamp = 0
        self.gpiooutput = False
        self.lightColor = 0
        GPIO.setmode(GPIO.TEGRA_SOC)

        print()
        self.NUM_PIXELS = 20
        PIXEL_ORDER = neopixel.GRB

        spi = board.SPI()

        self.pixels = neopixel.NeoPixel_SPI(
            spi, self.NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False
        )

        self.solenoid_subscriber = self.create_subscription(Bool, "/solenoid", self.relayCallback, 10)
        self.light_subscriber = self.create_subscription(Int8, "/light", self.neoCallback, 10)

        self.lightRelay = ["GP68"]
        GPIO.setup(self.lightRelay, GPIO.OUT, initial=GPIO.LOW)
        freq = 1
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.timer_handler)

    def timer_handler(self):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] 
        
        if self.gpiooutput:
            GPIO.output(self.lightRelay, GPIO.HIGH)
        else:
            GPIO.output(self.lightRelay, GPIO.LOW)
        
        # Flash green light (lightColor == 2) every 0.5 seconds
        if self.lightColor == 3:

            if (current_time % 3) ==0:  # First half of the second -> ON
                self.pixels.fill(0x00FF00)
            else:  # Second half of the second -> OFF
                self.pixels.fill(0)
            self.pixels.show()




    def relayCallback(self, msg: Bool):
        self.get_logger().info("outputting gpio " + str(msg.data))
        self.gpiooutput = msg.data
    
    def neoCallback(self, msg: Int8):
        self.get_logger().info("outputting color " + str(msg.data))

        if(msg.data == 1): #red
            self.pixels.fill(0xFF0000)
            self.pixels.show()
            self.lightColor = 1
        elif(msg.data == 2): #blue
            self.pixels.fill(0x0000FF)
            self.pixels.show()
            self.lightColor = 2
        elif(msg.data == 3): #green flashing
            self.pixels.fill(0x00FF00)
            self.pixels.show()
            self.lightColor = 3
        elif(msg.data == 0): #empty
            self.pixels.fill(0)
            self.pixels.show()
            self.lightColor = 0




def main(args=None):
    rclpy.init(args=args)
    node = gpioManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
