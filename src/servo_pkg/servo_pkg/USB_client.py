import rclpy
from rclpy.node import Node
from interfaces.srv import MoveServo


class USB_Client(Node):
    def __init__(self):
        super().__init__('usb_client')
    
        self.cli = self.create_client(MoveServo, 'turn_servo')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = MoveServo.Request()
        
    def send_request(self, port, pos):
        self.req.port = port
        self.req.pos = pos
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    
def main(args=None):
    rclpy.init(args=args)
    usb_client = USB_Client()
    
    
    #Move servo on port 0 to minimum position
    servo1 = MoveServo.Request()
    servo1.port = 0
    servo1.pos = 2432 
    
    response = usb_client.send_request(servo1.port, servo1.pos)
    usb_client.get_logger().info(
        'Results: %s, status: %s' %
        (response.status, response.status_msg))
    
    
    #Move servo on port 5 to maximum position
    servo2 = MoveServo.Request()
    servo2.port = 5
    servo2.pos = 9600
    
    response = usb_client.send_request(servo2.port, servo2.pos)
    usb_client.get_logger().info(
        'Results: %s, status: %s' %
        (response.status, response.status_msg))

    usb_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()