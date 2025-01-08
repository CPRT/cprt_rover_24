import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from interfaces.srv import MoveServo

from servo_pkg import maestro


class USB_Servo(Node):
    def __init__(self):
        super().__init__("usb_servo")
        
        self.declare_parameter("serial_port", "/dev/ttyACM0")   #default to "/dev/ttyACM0" port
        self.servo = maestro.Controller(self.get_parameter("serial_port").get_parameter_value().string_value)
        
        self.srv = self.create_service(MoveServo, "servo_service", self.set_position)

        # self tested min and max of the Tower Pro 9g micro servos. Values are positions represented in micro seconds.
        self.min = 512  #default values
        self.max = 2400

        for i in range(0, 11):  # usb controller has 12 channels
            self.servo.setRange(i, min, max)

    # Set target within valid range (min to max quarter-microseconds) example: servo.setTarget(0, 2400)
    def set_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        if request.pos > max or request.pos < min:
            response.status = False
            current_position = self.servo.getPosition(request.port)
            response.status_msg = f"Servo {request.port} input out of range\ncurrent position: {current_position}"

        else:
            self.servo.setTarget(request.port, request.pos)

            response.status = True
            current_position = self.servo.getPosition(request.port)
            response.status_msg = (
                f"Servo {request.port} current position: {current_position}"
            )

        self.servo.close()
        return response

    def get_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        response.status = True
        current_position = self.servo.getPosition(request.port)
        response.status_msg = f"{current_position}"

        self.servo.close()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()      #include serial port to change, default is "/dev/ttyACM0"
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
