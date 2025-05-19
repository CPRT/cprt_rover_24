import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.duration import Duration


class OdometryRepublisher(Node):
    def __init__(self, frequency: float):
        super().__init__("odometry_republisher")

        # Set the frequency for republishing (Hz)
        self.frequency = frequency
        self.last_odometry = None

        # Subscriber to the original odometry topic
        self.odometry_subscriber = self.create_subscription(
            Odometry,
            "/gps/odom",  # Original Odometry topic /gps/odom
            self.odometry_callback,
            10,  # QoS profile
        )

        # Publisher to the new republished odometry topic
        self.odometry_publisher = self.create_publisher(
            Odometry, "/repub_gps_odom", 10  # New Odometry topic  # QoS profile
        )

        # Timer to republish the last known odometry message at the specified frequency
        self.timer = self.create_timer(1.0 / self.frequency, self.republish_odometry)

    def odometry_callback(self, msg: Odometry):
        # Store the most recent odometry message
        self.get_logger().info(f"Received new Odometry message: {msg.header.stamp}")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.last_odometry = msg

    def republish_odometry(self):
        if self.last_odometry is not None:
            self.get_logger().info(
                f"Republishing Odometry message: {self.last_odometry.header.stamp}"
            )
            self.odometry_publisher.publish(self.last_odometry)
        else:
            self.get_logger().warn(
                "No new Odometry message received yet. Republishing the last one."
            )


def main(args=None):
    rclpy.init(args=args)

    # Set the frequency (in Hz) at which to republish the odometry
    frequency = 20  # Change to your desired frequency (Hz)

    # Create the node and start spinning
    node = OdometryRepublisher(frequency)
    rclpy.spin(node)

    # Shutdown ROS 2 when exiting
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
