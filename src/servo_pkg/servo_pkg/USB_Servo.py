import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo
import math
from std_msgs.msg import Float32
from servo_pkg import maestro
from config import Config

NUM_PORTS = 12
DEFAULT_MIN = 512
DEFAULT_MAX = 2400
DEFAULT_MAX_DEGREES = 180





def convert_from_radians(angle: float, servo_info: Servo_Info) -> int:
    total_range = servo_info.max - servo_info.min
    return int(servo_info.min + (total_range * angle * 180 / (servo_info.rom * math.pi)))


def convert_to_radians(value: int, servo_info: Servo_Info) -> int:
    total_range = servo_info.max - servo_info.min
    return float(servo_info.rom * (value - servo_info.min)  * math.pi / (total_range * 180))


class USB_Servo(Config):
    def __init__(self):
        super().__init__("usb_servo")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        self.servo = maestro.Controller(serial_port)
        self.declare_parameter("port", 0)
        self.port = self.get_parameter("port").get_parameter_value().integer_value
        
        # self.srv = self.create_service(MoveServo, "servo_service", self.set_position)
        self.sub = self.create_subscription(Int64, , self.set_position)

        self.load_port_config()

        for port, servo in self.servo_info.items():
            self.servo.setRange(port, servo.min, servo.max)


    def load_port_config(self):
        self.load_config()
        for port in range(NUM_PORTS):
            min_us = self.get_parameter(f"servo{port}.min").get_parameter_value().double_value
            max_us = self.get_parameter(f"servo{port}.max").get_parameter_value().double_value
            rom = self.get_parameter(f"servo{port}.rom").get_parameter_value().double_value
            
            # Convert microseconds to quarter-microseconds
            min_qus = min_us * 4
            max_qus = max_us * 4

            self.servo_info[port_number] = Servo_Info(
                port_number, min_qus, max_qus, rom
            )
            self.get_logger().info(
                f"Port {port_number} -> Min: {min_qus}, Max: {max_qus}"
            )
            self.servo.setRange(port_number, min_qus, max_qus)

    def set_position(self):
        port = self.port
        self.check_valid_servo(port)

        servo_info = self.servo_info[port]

        target_value = convert_from_radians(request.pos, servo_info)
        current_position = convert_to_radians(self.servo.getPosition(port), servo_info)

        if not (servo_info.min <= target_value <= servo_info.max):
            self.get_logger().warning(
                f"Servo {port} input out of range.\nCurrent position: {current_position}"
            )
        else:
            self.get_logger().debug(
                f"Received request for port {port}: {request.pos} angle -> {target_value}"
            )
            self.servo.setTarget(port, target_value)
            current_position = convert_to_radians(
                self.servo.getPosition(port), servo_info
            )
            self.get_logger().info(
                f"Servo {port} moved to angle: {current_position}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
