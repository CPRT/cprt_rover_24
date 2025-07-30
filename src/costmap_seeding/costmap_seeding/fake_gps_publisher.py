import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class FakeGpsPublisher(Node):
    def __init__(self):
        super().__init__("fake_gps_publisher")
        self.publisher_ = self.create_publisher(NavSatFix, "/gps/fix", 10)
        timer_period = 1.0  # seconds (for 1 Hz, adjust for your desired rate)
        self.timer = self.create_timer(timer_period, self.publish_gps_fix)
        self.get_logger().info("GPS Publisher Node Started")

    def publish_gps_fix(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"
        msg.status.status = 2
        msg.status.service = 1
        msg.latitude = 45.385166422
        msg.longitude = -75.69850266
        msg.altitude = 0.0
        msg.position_covariance = [0.001] * 9
        msg.position_covariance_type = 3

        self.publisher_.publish(msg)
        # self.get_logger().info(f'Publishing GPS Fix at {msg.header.stamp.sec}.{msg.header.stamp.nanosec}')


def main(args=None):
    rclpy.init(args=args)
    gps_publisher = FakeGpsPublisher()
    rclpy.spin(gps_publisher)
    gps_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
