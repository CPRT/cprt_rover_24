import math

NUM_SERVOS = 12
DEFAULT_MIN = 512.0
DEFAULT_MAX = 2400.0
DEFAULT_MAX_DEGREES = 180


class Servo_Info:
    def __init__(self, channel, min_pwm, max_pwm, max_deg):
        self.channel = channel
        self.min = min_pwm
        self.max = max_pwm
        self.rom = max_deg


class Config(Node):  # one motor per port
    def __init__(self):
        super().__init__("Parent")
        self.servo_info = {}
        self.load_config()

    def load_config(self):
        self.declare_parameter("servo", 0)
        self.servo = self.get_parameter("servo").get_parameter_value().integer_value
        self.declare_parameter("servos_used", NUM_SERVOS)
        self.num_servos = (
            self.get_parameter("servos_used").get_parameter_value().integer_value
        )
        for servo in range(self.num_servos):
            self.declare_parameter("servo{servo}.name", "")
            motor_name = (
                self.get_parameter("servo{servo}.name")
                .get_parameter_value()
                .string_value
            )
            self.declare_parameter("servo{servo}.min", DEFAULT_MIN)
            min_pwm = (
                self.get_parameter("servo{servo}.min")
                .get_parameter_value()
                .double_value
            )
            self.declare_parameter("servo{servo}.max", DEFAULT_MAX)
            max_pwm = self.get_parameter("max").get_parameter_value().double_value
            self.declare_parameter("servo{servo}.rom", DEFAULT_MAX_DEGREES)
            rom = (
                self.get_parameter("servo{servo}.rom")
                .get_parameter_value()
                .double_value
                * math.pi
                / 180
            )
            self.servo_info[motor_name] = Servo_Info(servo, min_pwm, max_pwm, rom)

    def check_valid_servo(self, channel):
        if self.num_servos <= 0:
            self.get_logger().error("Invalid number of ports")
            raise ValueError("Invalid number of ports")
        if channel not in self.servo_info:
            self.get_logger().error("Invalid servo")
            return False
        return True

    def convert_deg_to_rad(angle):
        return angle * math.pi / 180

    def convert_rad_to_deg(angle):
        return angle * 180 / math.pi
