import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.qos import QoSProfile, DurabilityPolicy
import array  # Efficiently store large arrays of numbers


class BlankMapPublisher(Node):
    """
    A ROS 2 node that publishes a blank OccupancyGrid message.

    The map is 1000x1000 meters with a 0.1 meter resolution.
    The origin (0,0) of the map is set to its center.
    All cells are initialized to 0 (free space).

    Parameters:
        - publish_once (bool): If true, the map is published once at startup.
                               If false (default), the map is published periodically.
    """

    def __init__(self):
        # Initialize the ROS 2 node with the name 'blank_map_publisher'
        super().__init__("blank_map_publisher")

        # --- Declare and get parameters ---
        # Declare a boolean parameter 'publish_once' with a default value of False
        self.declare_parameter("publish_once", True)
        # Get the value of the 'publish_once' parameter
        self.publish_once = (
            self.get_parameter("publish_once").get_parameter_value().bool_value
        )
        self.get_logger().info(f'Parameter "publish_once" set to: {self.publish_once}')

        # Create a publisher for OccupancyGrid messages on the '/blank_map' topic
        # Use a QoS profile with transient local durability for latched behavior
        qos_profile = QoSProfile(depth=10)
        qos_profile.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.publisher_ = self.create_publisher(
            OccupancyGrid, "/blank_map", qos_profile
        )

        # --- Conditional publishing logic ---
        if self.publish_once:
            # If publish_once is true, publish the map immediately
            self.publish_map()
            self.get_logger().info(
                "Map published once. Node will remain active but not publish again."
            )
            # No timer needed if publishing only once
        else:
            # If publish_once is false, create a timer to publish periodically
            self.timer = self.create_timer(10.0, self.publish_map)
            self.get_logger().info(
                "Blank Map Publisher Node has been started and will publish periodically."
            )

    def publish_map(self):
        """
        Generates and publishes a blank OccupancyGrid message.
        """
        map_msg = OccupancyGrid()

        # --- Set the Header ---
        header = Header()
        header.stamp = self.get_clock().now().to_msg()  # Current time
        header.frame_id = "map"  # Standard frame_id for maps in ROS
        map_msg.header = header

        # --- Set Map Information (info field) ---
        resolution = 0.2  # meters per cell
        map_width_meters = 1000.0  # Total width of the map in meters
        map_height_meters = 1000.0  # Total height of the map in meters

        # Calculate map dimensions in cells
        map_width_cells = int(map_width_meters / resolution)
        map_height_cells = int(map_height_meters / resolution)

        map_msg.info.resolution = resolution
        map_msg.info.width = map_width_cells
        map_msg.info.height = map_height_cells

        # Set the origin of the map
        # The OccupancyGrid's origin is its bottom-left corner.
        # If the center of a 1000x1000m map is (0,0),
        # then the bottom-left corner is at (-500, -500) meters.
        origin_pose = Pose()
        origin_pose.position.x = -map_width_meters / 2.0
        origin_pose.position.y = -map_height_meters / 2.0
        origin_pose.position.z = 0.0  # Maps are typically 2D, so z is 0

        # Set orientation (no rotation, identity quaternion)
        origin_pose.orientation.x = 0.0
        origin_pose.orientation.y = 0.0
        origin_pose.orientation.z = 0.0
        origin_pose.orientation.w = 1.0  # W component is 1 for no rotation
        map_msg.info.origin = origin_pose

        # --- Fill Map Data (data field) ---
        # The 'data' field is a 1D array representing the grid cells.
        # Values: -1 (unknown), 0 (free), 100 (occupied).
        # For a "blank" map, we'll initialize all cells as free (0).
        map_data_size = map_width_cells * map_height_cells
        # Use 'array.array' with type code 'b' for signed char (int8), which is efficient
        # and matches the ROS message type.
        map_msg.data = array.array("b", [0] * map_data_size)

        # Publish the generated map message
        self.publisher_.publish(map_msg)
        self.get_logger().info(
            f"Published a {map_width_meters}x{map_height_meters} meter blank OccupancyGrid map "
            f"with {map_width_cells}x{map_height_cells} cells."
        )


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)  # Initialize ROS 2 communication
    blank_map_publisher = BlankMapPublisher()  # Create an instance of the node
    rclpy.spin(blank_map_publisher)  # Keep the node alive and processing callbacks
    blank_map_publisher.destroy_node()  # Destroy the node when it's no longer needed
    rclpy.shutdown()  # Shutdown ROS 2 communication


if __name__ == "__main__":
    main()
