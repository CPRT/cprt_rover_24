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
            "serial_port", '/dev/ttyACM0'
        )  # default to '/dev/ttyACM0' port
        self.servo = maestro.Controller(
            #self.get_parameter("serial_port").get_parameter_value().string_value
            '/dev/ttyACM0'
        )

        self.srv = self.create_service(MoveServo, "servo_service", self.set_position)

        # These ranges come from the tower pro mini 9g data sheet
        # 512 - 2400 microseconds, postions must be in quarter-microseoncds, so 2048 - 9600
        # https://www.friendlywire.com/projects/ne555-servo-safe/SG90-datasheet.pdf
        # 
        # For the HS-40 615-2495 microseconds, converted: 2460 - 9980
        # https://www.servocity.com/hs-40-servo/
        self.min = 2048  # 0 Degrees
        self.max = 9600  # 180 Degrees

        self.ZERO_DEGREES_VALUE = 2048
        self.CONVERSION_VALUE = 7552
        self.MAX_DEGREES = 180

        for i in range(0, 12):  # usb controller has 12 channels
            self.servo.setRange(i, self.min, self.max)

    # Set target within valid range (min to max quarter-microseconds)
    def set_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        # if ranges fall outside typical range this must be changed
        # servo dependent, which I would like to change later
        if request.min != None and request.max != None and request.max > 0:
            if request.min == 0:
                self.min = self.ZERO_DEGREES_VALUE
            else:
                self.min = self.convert_from_degrees(request.min)

            self.max = self.convert_from_degrees(request.max)

        self.servo.setRange(request.port, self.min, self.max)
        if self.convert_from_degrees(request.pos) > self.max or self.convert_from_degrees(request.pos) < self.min:
            response.status = False
            current_position = self.servo.getPosition(request.port)
            response.status_msg = f"Servo {request.port} input out of range\ncurrent position: {current_position}"

        else:
            USB_Servo.get_logger(self).info(
            "Received request for: %s, Converted: %s" % (request.pos, self.convert_from_degrees(request.pos))
        )
            self.servo.setTarget(request.port, self.convert_from_degrees(request.pos))

            response.status = True
            current_position = self.servo.getPosition(request.port)
            response.status_msg = (
                f"Servo {request.port} current position: {current_position}"
            )

        return response

    def get_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        response.status = True
        current_position = self.servo.getPosition(request.port)
        response.status_msg = f"{current_position}"

        return response

    def convert_from_degrees(self, degrees: int) -> int:
        return int(self.ZERO_DEGREES_VALUE + (
            self.CONVERSION_VALUE / (self.MAX_DEGREES / degrees)
        ))


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()  # include serial port to change, default is "/dev/ttyACM0"
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
