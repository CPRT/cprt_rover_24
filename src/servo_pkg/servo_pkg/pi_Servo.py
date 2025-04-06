import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from interfaces.srv import MoveServo

from RPi import GPIO
from time import sleep


class pi_Servo(Node):
    def __init__(self):
        super().__init__("pi_servo")
        self.srv = self.create_service(MoveServo, "servo_service", self.set_position)

        self.declare_parameter(
            "out_pin", 32
        )  # default to pin 32. Should change this is just the pin i was using
        self.out_pin = self.get_parameter("out_pin").get_parameter_value().integer_value

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.out_pin, GPIO.OUT)
        self.pwm_pin = GPIO.PWM(self.out_pin, 50)  # Sets up the out_pin as a PWM pin
        self.pwm_pin.start(0)

        self.max = 12  # The Max value for the PWM
        self.min = 3  # The min value for the PWM
        self.maxrom = 180  # max range of motion of the servo, default 180

    def set_position(self, request, response) -> MoveServo:
        if request.max != None:
            self.maxrom = request.max
        pos = self.convert_to_pwm(request.pos)
        self.pwm_pin.ChangeDutyCycle(pos)

        response.status = True
        response.status_msg = f"Servo {request.port} moving to {request.pos} degrees"

        return response

    def convert_to_pwm(self, degree: int) -> int:
        return float(degree / (180 / (self.max - self.min)) + 3)

    def destroy_node(self):
        self.pwm_pin.stop()
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = pi_Servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        rclpy.shutdown()
        node.destroy_node()
    finally:
        rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
