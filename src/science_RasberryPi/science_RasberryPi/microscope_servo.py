#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import lgpio
import time

class microscope_servo(Node):
    def __init__(self):
        super().__init__('microscope_servo')    
        self.load_parameter()
        

        self.h = lgpio.gpiochip_open(0)

        try:
            while True:
        # Turn the OUT_PIN off
                lgpio.tx_pwm(self.h, self.OUT_PIN, self.FREQ, 0)
                time.sleep(10)

                # Turn the OUT_PIN to medium speed
                lgpio.tx_pwm(self.h, self.OUT_PIN, self.FREQ, 50)
                time.sleep(10)

                # Turn the OUT_PIN to max speed
                lgpio.tx_pwm(self.h, self.OUT_PIN, self.FREQ, 100)
                time.sleep(10)
        except KeyboardInterrupt:
            # Turn the OUT_PIN to medium speed
            lgpio.tx_pwm(self.h, self.OUT_PIN, self.FREQ, 50)
            lgpio.gpiochip_close(self.h)
        # Create subscribers
            
        # self.relative_sub = self.create_subscription(Float32,'microscope/relative_move',self.relative_callback,10)
            
        # # Create publisher for current position
        # self.position_pub = self.create_publisher(Float32, 'microscope/current_position', 10)
        
        # # Timer for publishing current position
        # self.timer = self.create_timer(1/self.freq, self.publish_position)
        
        self.get_logger().info('Microscope Controller Node Initialized')
    
    def load_parameter(self):
        # Parameters
        self.declare_parameter('OUT_PIN', 0)
        self.OUT_PIN = (
            self.get_parameter('OUT_PIN').get_parameter_value().integer_value
            )
        self.declare_parameter('FREQ', 500)
        self.FREQ = (
            self.get_parameter('FREQ').get_parameter_value().integer_value
            )
        self.declare_parameter('frequency',5.0)
        self.freq = self.get_parameter("frequency").get_parameter_value().double_value

    # def relative_callback(self, msg):
    #     """Move relative to current position"""
    #     new_position = self.current_position + msg.data
    #     new_position = max(self.min_angle, min(self.max_angle, new_position))
    #     self._set_servo_position(new_position)
    
    # def _set_servo_position(self, angle):
    #     """Internal method to set servo position"""
    #     self.current_position = angle
    #     self.servo.angle = angle
    #     self.get_logger().info(f'Microscope position set to: {angle}°')
    
    # def publish_position(self):
    #     """Publish current position"""
    #     msg = Float32()
    #     msg.data = float(self.current_position)
    #     self.position_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = microscope_servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
