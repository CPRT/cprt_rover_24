#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import time

# Choose appropriate servo control library based on your setup
from adafruit_servokit import ServoKit

class microscope_servo(Node):
    def __init__(self):
        super().__init__('microscope_servo')    
        self.load_parameter()
    # Initialize servo control
        self.kit = ServoKit(channels=16)
        self.servo = self.kit.servo[self.servo_channel]
        self.servo.set_pulse_width_range(self.min_pulse, self.max_pulse)
        self.servo.angle = self.current_position
        
        # Create subscribers
            
        self.relative_sub = self.create_subscription(Float32,'microscope/relative_move',self.relative_callback,10)
            
        # Create publisher for current position
        self.position_pub = self.create_publisher(Float32, 'microscope/current_position', 10)
        
        # Timer for publishing current position
        self.timer = self.create_timer(1/self.freq, self.publish_position)
        
        self.get_logger().info('Microscope Controller Node Initialized')
    
    def load_parameter(self):
        # Parameters
        self.declare_parameter('servo_channel', 0)
        self.servo_channel = (
            self.get_parameter('servo_channel').get_parameter_value().integer_value
            )
        self.declare_parameter('min_pulse', 500)
        self.min_pulse = (
            self.get_parameter('min_pulse').get_parameter_value().integer_value
            )
        self.declare_parameter('max_pulse', 2500)
        self.max_pulse = (
            self.get_parameter('max_pulse').get_parameter_value().integer_value
            )
        self.declare_parameter('min_angle', 0)
        self.min_angle = (
            self.get_parameter('min_angle').get_parameter_value().integer_value
            )
        self.declare_parameter('max_angle', 180)
        self.max_angle = (
            self.get_parameter('max_angle').get_parameter_value().integer_value
            )
        self.declare_parameter('initial_position', 90)
        self.current_position = (
            self.get_parameter('initial_position').get_parameter_value().integer_value
            )
        self.declare_parameter('frequency',5.0)
        self.freq = self.get_parameter("frequency").get_parameter_value().double_value

    def relative_callback(self, msg):
        """Move relative to current position"""
        new_position = self.current_position + msg.data
        new_position = max(self.min_angle, min(self.max_angle, new_position))
        self._set_servo_position(new_position)
    
    def _set_servo_position(self, angle):
        """Internal method to set servo position"""
        self.current_position = angle
        self.servo.angle = angle
        self.get_logger().info(f'Microscope position set to: {angle}°')
    
    def publish_position(self):
        """Publish current position"""
        msg = Float32()
        msg.data = float(self.current_position)
        self.position_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = microscope_servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
