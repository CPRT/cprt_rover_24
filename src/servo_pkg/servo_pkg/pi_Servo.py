import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo
import math
from rpi_hardware_pwm import HardwarePWM
from config import Config
from std_msgs.msg import Float32
from config import Servo_Info


def to_channel(pin: int) -> int:
    if pin == 18:
        return 0
    elif pin == 19:
        return 1
    raise ValueError(f"Entered non PWM pin: {pin}")


class Servo:
    def __init__(self, servo_info, frequency: int, rom: int):
        self.servo_info = servo_info
        self.frequency = frequency
        self.pwm_pin = HardwarePWM(pwm_channel=self.channel, hz=self.frequency, chip=0)
        self.pwm_pin.start(0)

    def set_position(self, angle: float):
        if angle < 0 or angle > self.rom:
            raise ValueError(f"Angle out of range: {angle}")

        duty_cycle = self.convert_to_pwm(angle)
        self.pwm_pin.change_duty_cycle(duty_cycle)

    def convert_to_pwm(self, angle: float) -> float:
        return float(convert_rad_to_deg(angle) / (self.rom / (self.max_pos - self.min_pos)) + self.min_pos)

    def stop(self):
        self.pwm_pin.stop()


class pi_Servo(Config):
    def __init__(self):
        super().__init__("pi_servo")
        # self.srv = self.create_service(MoveServo, "servo_service", self.set_position)
        self.sub = self.create_subscription(Int64, "")
        self.load_params()

    def load_params(self):

        self.load_config()
        for i in range(num_servos):
            self.declare_parameter(f"servo{i}.frequency", 50)
            self.declare_parameter(f"servo{i}.out_pin", 0)
            frequency = (
                self.get_parameter(f"servo{i}.frequency")
                .get_parameter_value()
                .integer_value
            )
            outpin = (
                self.get_parameter(f"servo{i}.out_pin")
                .get_parameter_value()
                .integer_value
            )
            if outpin < 0:
                self.get_logger().error(f"Invalid pin number for port {i}")
                raise ValueError(f"Invalid pin number for port {i}")
            self.servo_list[i].channel = to_channel(outpin)
            self.servos[i] = Servo(servo_info=self.servo_info[i], frequency=frequency)

    def set_position(self, request, response) -> MoveServo:
        port = request.port
        if port not in self.servos:
            response.status = False
            response.status_msg = f"Invalid port: {port}"
            self.get_logger().error(response.status_msg)
            return response

        servo = self.servos[port]
        degree = request.pos
        try:
            servo.set_position(degree)
        except ValueError as e:
            response.status = False
            response.status_msg = str(e)
            self.get_logger().error(f"Error setting position {str(e)}")
            return response
        response.status = True
        response.status_msg = f"Moved to {degree} degrees"
        self.get_logger().info(response.status_msg)
        return response

    def destroy_node(self):
        for servo in self.servos.values():
            servo.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = pi_Servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
