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

        # Servo center is at 1500 microseconds, or 6000 quarter-microseconds
        # Typcially valid servo range is 3000 to 9000 quarter-microseconds
        self.min = 3000     # 0 Degrees
        self.max = 6000     # 180 Degrees

        for i in range(0, 11):  # usb controller has 12 channels
            self.servo.setRange(i, min, max)

    # Set target within valid range (min to max quarter-microseconds) 
    def set_position(self, request: MoveServo, response: MoveServo) -> MoveServo:
        #if ranges fall outside typical range this must can be changed
        #servo dependent, which I would like to change later
        if (request.min != None and request.max != None and request.max > 0):
            if (request.min == 0):
                self.min = 3000
            else:
                self.min = 3000 + (6000/(360/request.min)) 
                
            self.max = 3000 + (6000/(360/request.max)) 
        
        self.servo.setRange(request.port, min, max)
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
