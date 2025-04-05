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

        self.declare_parameter("out_pin", 32)  # default to '/dev/ttyACM0' port
        self.out_pin = self.get_parameter("out_pin").get_parameter_value().integer_value

        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.out_pin, GPIO.OUT)
        self.p = GPIO.PWM(self.out_pin, 50)  # Sets up pin 11 as a PWM pin
        self.p.start(0)

        self.max = 12  # max range of motion of the servo, default 180
        self.min = 3
        self.maxrom = 180

    def set_position(self, request, response) -> MoveServo:
        if request.max != None:
            self.maxrom = request.max
        pos = self.convert_to_pwm(request.pos)
        self.p.ChangeDutyCycle(pos)
        out_angle = self.convert_to_degrees(pos)

        response.status = True
        response.status_msg = f"Servo {request.port} moving to {out_angle} degrees"
        self.get_logger().info(
            f"Servo {request.port} moving to {out_angle} degrees, {pos}"
        )

        return response

    def convert_to_degrees(self, pwm: int) -> int:
        return int((180 / (self.max - self.min)) * (pwm - 3))

    def convert_to_pwm(self, degree: int) -> int:
        return float(degree / (180 / (self.max - self.min)) + 3)

    def on_shutdown(self):
        GPIO.setup(self.out_pin, GPIO.OUT).stop()
        GPIO.cleanup()


def main(args=None):
    rclpy.init(args=args)
    node = pi_Servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()
        rclpy.shutdown()
        pi_Servo.on_shutdown(pi_Servo)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        pi_Servo.on_shutdown(pi_Servo)


if __name__ == "__main__":
    main()
