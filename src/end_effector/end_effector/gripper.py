import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from interfaces.srv import MoveServo

class Gripper(Node):
    def __init__(self):
        super().__init__("gripper")
        
        self.MAX_ACTUATION = 71
        self.MIN_ACTUATION = 8
        self.SERVO_MAX = 180
        self.SERVO_MIN = 0
        self.SERVO_PORT = 0
        
        self.currPos = self.MAX_ACTUATION
        
        self.cli = self.create_client(MoveServo, "servo_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        
        self.cmd_joy_subscriber = self.create_subscription(
            Joy, "/joy", self.cmd_joy_callback, 10
        )
        
        self.servo_request(self.SERVO_PORT, self.MAX_ACTUATION, self.SERVO_MIN, self.SERVO_MAX) #set initial position to fully open
        
    def send_request(self, port: int, pos: int, min: int, max: int) -> MoveServo:
        req = MoveServo.Request()
        req.port = port
        req.pos = pos
        req.min = min
        req.max = max
        self.future = self.cli.call_async(req)
        return self.future.result()
        
    def servo_request(self, req_port, req_pos, req_min, req_max) -> None:
        self.send_request(port=req_port, pos=req_pos, min=req_min, max=req_max)
        
        
    def cmd_joy_callback(self, msg: Joy):
        if msg.buttons[0] == 1:
            if(self.currPos > self.MIN_ACTUATION):
                self.currPos -= 5
                self.servo_request(self.SERVO_PORT, self.currPos, self.SERVO_MIN, self.SERVO_MAX)
            
        if msg.buttons[1] == 1:
            if(self.currPos < self.MAX_ACTUATION):
                self.currPos += 5
                self.servo_request(self.SERVO_PORT, self.currPos, self.SERVO_MIN, self.SERVO_MAX)

def main(args=None):
    rclpy.init(args=args)
    gripper = Gripper()
    rclpy.spin(gripper)
    gripper.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()