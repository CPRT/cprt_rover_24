import rclpy
import rclpy.logging
from rclpy.node import Node
import rclpy.time
from interfaces.srv import MoveServo

from servo_pkg import maestro


class USB_Servo(Node):
    def __init__(self):
        super().__init__("usb_servo")
        self.srv = self.create_service(MoveServo, "turn_servo", self.set_position)

    # Set target within valid range (min to max quarter-microseconds) example: servo.setTarget(0, 6000)
    def set_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        self.servo = maestro.Controller()
        min = int[608 * 4]
        max = int[2400 * 4] 

        self.servo.setRange(request.port, min, max)

        if request.pos > max or request.pos < min:
            response.status = False
            current_position = int[self.servo.getPosition(request.port)]
            response.status_msg = str[(
                f"Servo {request.port} input out of range\ncurrent position: {current_position}"
                )]

        else:
            self.servo.setTarget(request.port, request.pos)

            response.status = True
            current_position = int[self.servo.getPosition(request.port)]
            response.status_msg = str[(
                f"Servo {request.port} current position: {current_position}"
            )]

        self.servo.close()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = USB_Servo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
