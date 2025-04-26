import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from interfaces.srv import MoveServo

from servo_pkg import maestro


class USB_Servo(Node):
    def __init__(self):
        super().__init__("usb_servo")

        self.declare_parameter(
            "serial_port", "/dev/ttyACM0"
        )  # default to '/dev/ttyACM0' port
        self.servo = maestro.Controller(
            self.get_parameter("serial_port").get_parameter_value().string_value
        )

        self.srv = self.create_service(MoveServo, "servo_service", self.set_position)

        # These ranges come from the tower pro mini 9g data sheet
        # 512 - 2400 microseconds, postions must be in quarter-microseoncds, so 2048 - 9600
        # https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf
        #
        # For the HS-40 615-2495 microseconds, converted: 2460 - 9980
        # https://www.servocity.com/hs-40-servo/
        # Default SG90 values
        self.DEFAULT_MIN = 2048
        self.DEFAULT_MAX = 9600
        self.ZERO_DEGREES_VALUE = 2048
        self.CONVERSION_RANGE = 7552  # 9600 - 2048
        self.MAX_DEGREES = 180

        # port number -> (min, max)
        self.servo_ranges = {}

        # Initialize default ranges for 12 channels
        for port in range(0, 12):
            self.servo.setRange(port, self.DEFAULT_MIN, self.DEFAULT_MAX)
            self.servo_ranges[port] = (self.DEFAULT_MIN, self.DEFAULT_MAX)

    def set_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        port = request.port

        # Update min/max if provided 
        if request.min is not None and request.max is not None and request.max > 0:
            self.min = request.min * 4
            self.max = request.max * 4
            self.servo_ranges[port] = (self.min, self.max)
            self.servo.setRange(port, self.min, self.max)

        min_range, max_range = self.servo_ranges.get(
            port, (self.DEFAULT_MIN, self.DEFAULT_MAX)
        )

        # Convert requested from degrees 
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
        min_range, max_range = self.servo_ranges.get(port, (self.DEFAULT_MIN, self.DEFAULT_MAX))
        total_range = max_range - min_range

        return int(
            min_range + (total_range * degrees / self.MAX_DEGREES)
    )

def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()  # include serial port to change, default is "/dev/ttyACM0"
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
