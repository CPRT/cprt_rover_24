import math
from rclpy.node import Node

NUM_SERVOS = 12
DEFAULT_MIN = 512.0
DEFAULT_MAX = 2400.0
DEFAULT_MAX_ANGLE = 3.1415


class Servo_Info:
    def __init__(self, motor_name, min_pwm, max_pwm, max_deg):
        self.motor_name = motor_name
        self.min = min_pwm
        self.max = max_pwm
        self.rom = max_deg


class Parent_Config(Node):  # one motor per port
    def __init__(self, name):
        super().__init__(name)
        self.servo_info = {}
        self.load_config()

    def load_config(self):
        self.declare_parameter("servo_num", 0)
        self.servo_num = (
            self.get_parameter("servo_num").get_parameter_value().integer_value
        )
        self.declare_parameter("servos_used", NUM_SERVOS)
        self.num_servos = (
            self.get_parameter("servos_used").get_parameter_value().integer_value
        )
        for servo in range(self.num_servos):
            self.declare_parameter(f"servo{servo}.name", f"{servo}")
            motor_name = (
                self.get_parameter(f"servo{servo}.name")
                .get_parameter_value()
                .string_value
            )
            self.declare_parameter(f"servo{servo}.min", DEFAULT_MIN)
            min_pwm = (
                self.get_parameter(f"servo{servo}.min")
                .get_parameter_value()
                .double_value
            )
            self.declare_parameter(f"servo{servo}.max", DEFAULT_MAX)
            max_pwm = (
                self.get_parameter(f"servo{servo}.max")
                .get_parameter_value()
                .double_value
            )
            self.declare_parameter(f"servo{servo}.rom", DEFAULT_MAX_ANGLE)
            rom = (
                self.get_parameter(f"servo{servo}.rom")
                .get_parameter_value()
                .double_value
            )
            self.servo_info[servo] = Servo_Info(motor_name, min_pwm, max_pwm, rom)

    def check_valid_servo(self, channel):
        if self.num_servos <= 0:
            self.get_logger().error("Invalid number of ports")
            raise ValueError("Invalid number of ports")
        if channel not in self.servo_info:
            self.get_logger().error("Invalid servo")
            return False
        return True
