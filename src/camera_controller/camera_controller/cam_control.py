#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo
from sensor_msgs.msg import Joy

#import random

class Cam_Servo_Client(Node):
    #Ports that the servos are in
    PORT_X = 0
    PORT_Y = 1

    #Bounds for the signal the controller sends
    MIN_INPUT = 0
    MAX_INPUT = 180
    MID_INPUT = int((MAX_INPUT - MIN_INPUT) / 2 + MIN_INPUT)

    #Bounds for the mobility of the servos
    MIN_SERVO = 0
    MAX_SERVO = 180
    MID_SERVO = int((MAX_SERVO - MIN_SERVO) / 2 + MIN_SERVO)

    #Miscellaneous
    MOD = 9
    last_pos_x = 0
    last_pos_y = 0

    def __init__(self):
        super().__init__("cam_servo_Client")

        self.cli = self.create_client(MoveServo, "cam_servo_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.timer = self.create_timer(2, self.camera_mover)
        self.subscription = self.create_subscription(

    def send_request(self, port: int, pos: int, min: int, max: int) -> MoveServo:
        req = MoveServo.Request()
        req.port = port
        req.pos = pos
        req.min = min
        req.max = max
        future = self.cli.call_async(req)

    def servo_request(self, req_port, req_pos, req_min, req_max) -> None:
        Cam_Servo_Client.get_logger(self).info("Sending Request for: %s" % (req_pos))
        self.send_request(port=req_port, pos=req_pos, min=req_min, max=req_max)

    def convert_command(self, input_signal) -> int: #Turns the signal into an increment to be added to the most recent position
        input_signal = input_signal - self.MID_INPUT
        input_signal %= self.MOD

        return input_signal
                
    def bound_enforce(self, servo_command) -> int: #Makes sure the servo command doesnt go out of bounds
        if(servo_command < self.MIN_SERVO):
            servo_command = self.MIN_SERVO
        elif(servo_command > self.MAX_SERVO):
            servo_command = self.MAX_SERVO

        return servo_command

    def camera_mover(self) -> None: #I was told the joystick sends an angle between 0 and 180

        input_signal_x = 0
        input_signal_y = 0

        #Here would go the code to fetch the input, if I knew what node was giving the input

        input_signal_x = self.bound_enforce( self.last_pos_x + self.convert_command(self, input_signal_x) )
        input_signal_y = self.bound_enforce( self.last_pos_y + self.convert_command(self, input_signal_y) )

        self.servo_request(0, input_signal_x, 0, 180)
        self.last_pos_x = input_signal_x
        self.servo_request(0, input_signal_y, 0, 180)
        self.last_pos_y = input_signal_y


def main(args=None):
    rclpy.init(args=args)

    servo_Client = Cam_Servo_Client()
    rclpy.spin(servo_Client)
    servo_Client.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
