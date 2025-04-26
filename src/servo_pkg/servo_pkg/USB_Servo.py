import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo
from servo_pkg import maestro
import yaml
import os


class USB_Servo(Node):
    def __init__(self):
        super().__init__("usb_servo")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("config_file", "servo_ranges.yaml")

        serial_port = (
            self.get_parameter("serial_port").get_parameter_value().string_value
        )
        config_file = (
            self.get_parameter("config_file").get_parameter_value().string_value
        )

        self.servo = maestro.Controller(serial_port)
        self.srv = self.create_service(MoveServo, "servo_service", self.set_position)

        # Default values
        self.DEFAULT_MIN = 2048
        self.DEFAULT_MAX = 9600
        self.MAX_DEGREES = 180

        self.servo_ranges = {}

        self.load_servo_config(config_file)

        for port, (min_val, max_val) in self.servo_ranges.items():
            self.servo.setRange(port, min_val, max_val)

    def load_servo_config(self, file_path):
        if not os.path.isfile(file_path):
            self.get_logger().error(f"Config file not found: {file_path}")
            return

        with open(file_path, "r") as f:
            config = yaml.safe_load(f)

        controller_config = config.get("usb_controller", {})
        for port_name, settings in controller_config.items():
            port_number = int(port_name.replace("port", ""))
            min_us = settings.get("min", 0)
            max_us = settings.get("max", 0)

            if min_us == 0 and max_us == 0:
                # Use default min/max
                min_qus = self.DEFAULT_MIN
                max_qus = self.DEFAULT_MAX
            else:
                # Convert microseconds to quarter-microseconds
                min_qus = min_us * 4
                max_qus = max_us * 4

            self.servo_ranges[port_number] = (min_qus, max_qus)
            self.get_logger().info(
                f"Port {port_number} -> Min: {min_qus}, Max: {max_qus}"
            )

    def set_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        port = request.port

        # Update range if explicitly given
        if request.min is not None and request.max is not None and request.max > 0:
            self.servo_ranges[port] = (request.min, request.max)
            self.servo.setRange(port, request.min, request.max)

        min_range, max_range = self.servo_ranges.get(
            port, (self.DEFAULT_MIN, self.DEFAULT_MAX)
        )
        target_value = self.convert_from_degrees(request.pos, port)

        if not (min_range <= target_value <= max_range):
            response.status = False
            current_position = self.servo.getPosition(port)
            response.status_msg = f"Servo {port} input out of range.\nCurrent position: {current_position}"
        else:
            self.get_logger().info(
                f"Received request for port {port}: {request.pos} degrees -> {target_value}"
            )
            self.servo.setTarget(port, target_value)

            response.status = True
            current_position = self.servo.getPosition(port)
            response.status_msg = f"Servo {port} moved to: {current_position}"

        return response

    def convert_from_degrees(self, degrees: int, port: int) -> int:
        min_range, max_range = self.servo_ranges.get(
            port, (self.DEFAULT_MIN, self.DEFAULT_MAX)
        )
        total_range = max_range - min_range

        return int(min_range + (total_range * degrees / self.MAX_DEGREES))


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
