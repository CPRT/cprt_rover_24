import math

class Config(Node): # one motor per port
    def __init__(self):
        super().__init__("Parent")
        self.servo_info = {}
        self.load_config()
    
    def load_config(self):
        self.declare_parameter("motor_name", "base_motor")
        self.motor_name = self.get_parameter("motor_name").get_parameter_value().string_value
        self.declare_parameter("min", 500)
        self.min = self.get_parameter("min").get_parameter_value().integer_value
        self.declare_parameter("max", 2500)
        self.max = self.get_parameter("max").get_parameter_value().integer_value
        self.declare_parameter("rom", 180)
        self.rom = self.get_parameter("rom").get_parameter_value().integer_value
        self.declare_parameter("servos_used", 0)
        self.num_servos = self.get_parameter("servos_used").get_parameter_value().integer_value
    
    def check_valid_port(self, channel):
        if channel not in self.servo_info:
            self.get_logger().error("Invalid port")
            return False
        return True
        


