import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion
import array
import yaml  # For parsing YAML metadata
from PIL import Image  # For loading PNG images
import os  # For path manipulation
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)  # Import QoS policies


class SingleMapPublisher(Node):
    """
    A ROS 2 node that loads an OccupancyGrid from a PNG image and a YAML metadata file.
    It then publishes this map at a specified 'publish_resolution' and can constrain
    its maximum physical dimensions using 'max_width' and 'max_height'.

    The map's origin is set such that its center aligns with (0,0) in the 'map' frame.

    Parameters:
        - map_yaml_path (str): Full path to the map's YAML metadata file.
        - publish_resolution (float): The desired resolution (in meters/pixel) for the
                                      published OccupancyGrid. Defaults to 0.1 meters.
        - publish_once (bool): If true (default), the map is published once at startup.
                               If false, the map is published periodically.
        - max_width (float): Maximum physical width (in meters) of the published map.
                             If 0 or negative, no width constraint is applied.
        - max_height (float): Maximum physical height (in meters) of the published map.
                              If 0 or negative, no height constraint is applied.
    """

    def __init__(self):
        # Initialize the ROS 2 node with the name 'single_map_publisher'
        super().__init__("single_map_publisher")

        # --- Declare and get parameters ---
        self.declare_parameter(
            "map_yaml_path",
            "/usr/local/cprt/CanElevation/Ottawa_Gatineau/costmaps/dtm_1m_utm18_w_10_103.yaml",
        )
        self.declare_parameter("publish_resolution", 0.2)
        self.declare_parameter("publish_once", True)
        self.declare_parameter(
            "max_width", 700.0
        )  # New parameter, 0.0 means no constraint
        self.declare_parameter(
            "max_height", 700.0
        )  # New parameter, 0.0 means no constraint

        self.map_yaml_path = (
            self.get_parameter("map_yaml_path").get_parameter_value().string_value
        )
        self.publish_resolution = (
            self.get_parameter("publish_resolution").get_parameter_value().double_value
        )
        self.publish_once = (
            self.get_parameter("publish_once").get_parameter_value().bool_value
        )
        self.max_width = (
            self.get_parameter("max_width").get_parameter_value().double_value
        )
        self.max_height = (
            self.get_parameter("max_height").get_parameter_value().double_value
        )

        if not self.map_yaml_path:
            self.get_logger().error(
                'Parameter "map_yaml_path" is not set. Please provide the path to the YAML file.'
            )
            raise ValueError("map_yaml_path parameter is required.")

        self.get_logger().info(f"Loading map from YAML: {self.map_yaml_path}")
        self.get_logger().info(
            f"Publishing map at resolution: {self.publish_resolution} m/pixel"
        )
        self.get_logger().info(f'Parameter "publish_once" set to: {self.publish_once}')
        if self.max_width > 0:
            self.get_logger().info(f"Max published map width: {self.max_width} meters")
        if self.max_height > 0:
            self.get_logger().info(
                f"Max published map height: {self.max_height} meters"
            )

        # --- Define QoS Profile for the map publisher ---
        map_qos_profile = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.publisher_ = self.create_publisher(
            OccupancyGrid, "/blank_map", qos_profile=map_qos_profile
        )

        # Load and process the map data immediately
        self.map_msg = self._load_and_process_map_data()

        # --- Conditional publishing logic ---
        if self.publish_once:
            if self.map_msg:
                self.publisher_.publish(self.map_msg)
                self.get_logger().info(
                    "Map published once. Node will remain active but not publish again."
                )
            else:
                self.get_logger().error(
                    "Failed to load or process map, cannot publish."
                )
        else:
            if self.map_msg:
                self.timer = self.create_timer(1.0, self._periodic_publish)
                self.get_logger().info(
                    "Single Map Publisher Node has been started and will publish periodically."
                )
            else:
                self.get_logger().error(
                    "Failed to load or process map, periodic publishing will not start."
                )

    def _load_and_process_map_data(self):
        """
        Loads map metadata from YAML and pixel data from PNG, then processes it
        to the desired 'publish_resolution' and applies 'max_width'/'max_height'
        constraints to create an OccupancyGrid message.
        """
        map_msg = OccupancyGrid()

        try:
            # Load YAML metadata
            with open(self.map_yaml_path, "r") as f:
                yaml_data = yaml.safe_load(f)

            source_resolution = float(yaml_data["resolution_m"])
            filename = yaml_data["filename"]
            value_scale = yaml_data["value_scale"]

            map_image_dir = os.path.dirname(self.map_yaml_path)
            map_image_path = os.path.join(map_image_dir, filename)

            self.get_logger().info(f"Loading image from: {map_image_path}")

            img = Image.open(map_image_path).convert("L")
            source_img_width, source_img_height = img.size

            # Calculate total physical dimensions of the SOURCE map
            source_map_width_meters = source_img_width * source_resolution
            source_map_height_meters = source_img_height * source_resolution

            # Determine the physical dimensions of the PUBLISHED map, applying constraints
            constrained_map_width_meters = source_map_width_meters
            if self.max_width > 0 and self.max_width < source_map_width_meters:
                constrained_map_width_meters = self.max_width

            constrained_map_height_meters = source_map_height_meters
            if self.max_height > 0 and self.max_height < source_map_height_meters:
                constrained_map_height_meters = self.max_height

            # New dimensions in cells for the PUBLISHED map
            published_map_width_cells = int(
                constrained_map_width_meters / self.publish_resolution
            )
            published_map_height_cells = int(
                constrained_map_height_meters / self.publish_resolution
            )

            # --- Set Map Information (info field) for the PUBLISHED map ---
            map_msg.info.resolution = self.publish_resolution
            map_msg.info.width = published_map_width_cells
            map_msg.info.height = published_map_height_cells

            # Set the origin of the PUBLISHED map
            # The origin is its bottom-left corner. To center the map at (0,0),
            # the bottom-left corner is at (-constrained_map_width/2, -constrained_map_height/2).
            origin_pose = Pose()
            origin_pose.position.x = -constrained_map_width_meters / 2.0
            origin_pose.position.y = -constrained_map_height_meters / 2.0
            origin_pose.position.z = 0.0

            origin_pose.orientation.x = 0.0
            origin_pose.orientation.y = 0.0
            origin_pose.orientation.z = 0.0
            origin_pose.orientation.w = 1.0
            map_msg.info.origin = origin_pose

            # --- Fill Map Data (data field) ---
            map_data = array.array(
                "b", [-1] * (published_map_width_cells * published_map_height_cells)
            )  # Initialize with unknown
            pixels = img.load()

            min_val = value_scale["min"]
            max_val = value_scale["max"]
            unknown_val = value_scale["unknown"]

            # Calculate the global coordinates of the source map's bottom-left corner
            source_map_global_origin_x = -source_map_width_meters / 2.0
            source_map_global_origin_y = -source_map_height_meters / 2.0

            # Iterate through the cells of the *source* (low-resolution) map
            for source_grid_y in range(source_img_height):
                if source_grid_y % 1000 == 0:
                    self.get_logger().info(
                        f"Processing row {source_grid_y + 1}/{source_img_height} of the source map..."
                    )
                for source_grid_x in range(source_img_width):
                    # Get pixel value from the original image (PIL uses top-left origin)
                    y_img = (
                        source_img_height - 1 - source_grid_y
                    )  # Convert ROS-style y (bottom-left) to PIL y (top-left)
                    pixel_value = pixels[source_grid_x, y_img]

                    # Convert pixel value to OccupancyGrid value
                    occupancy_value = -1  # Default to unknown
                    if pixel_value == unknown_val:
                        occupancy_value = -1
                    elif pixel_value == min_val:
                        occupancy_value = 0
                    elif pixel_value == max_val:
                        occupancy_value = 100
                    else:
                        if max_val != min_val:
                            scaled_value = int(
                                ((pixel_value - min_val) / (max_val - min_val)) * 100
                            )
                            occupancy_value = max(0, min(100, scaled_value))
                        else:
                            occupancy_value = 0  # Default to free if range is zero

                    # Calculate the global coordinates of the bottom-left corner of this source cell
                    current_source_cell_global_x = (
                        source_map_global_origin_x + source_grid_x * source_resolution
                    )
                    current_source_cell_global_y = (
                        source_map_global_origin_y + source_grid_y * source_resolution
                    )

                    # Determine the range of published cells that this source cell covers
                    # These are the start/end indices in the *published* map's grid
                    start_published_x_cell = int(
                        (current_source_cell_global_x - map_msg.info.origin.position.x)
                        / self.publish_resolution
                    )
                    start_published_y_cell = int(
                        (current_source_cell_global_y - map_msg.info.origin.position.y)
                        / self.publish_resolution
                    )

                    end_published_x_cell = int(
                        (
                            current_source_cell_global_x
                            + source_resolution
                            - map_msg.info.origin.position.x
                        )
                        / self.publish_resolution
                    )
                    end_published_y_cell = int(
                        (
                            current_source_cell_global_y
                            + source_resolution
                            - map_msg.info.origin.position.y
                        )
                        / self.publish_resolution
                    )

                    # Fill all cells within this block in the published map
                    for y_fill in range(start_published_y_cell, end_published_y_cell):
                        for x_fill in range(
                            start_published_x_cell, end_published_x_cell
                        ):
                            # Ensure we don't write outside the bounds of the published map
                            if (
                                0 <= x_fill < published_map_width_cells
                                and 0 <= y_fill < published_map_height_cells
                            ):
                                map_data[
                                    y_fill * published_map_width_cells + x_fill
                                ] = occupancy_value

            map_msg.data = map_data

            # --- Set the Header ---
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "map"
            map_msg.header = header

            self.get_logger().info(
                f"Successfully loaded source map: {filename} ({source_resolution}m resolution, "
                f"{source_img_width}x{source_img_height} cells). "
                f"Published map dimensions: {published_map_width_cells}x{published_map_height_cells} cells "
                f"at {self.publish_resolution}m resolution "
                f"({constrained_map_width_meters:.2f}x{constrained_map_height_meters:.2f} meters)."
            )
            return map_msg

        except FileNotFoundError as e:
            self.get_logger().error(f"Map file not found: {e}")
            return None
        except yaml.YAMLError as e:
            self.get_logger().error(f"Error parsing YAML file: {e}")
            return None
        except Exception as e:
            self.get_logger().error(
                f"An unexpected error occurred while loading or processing map: {e}"
            )
            return None

    def _periodic_publish(self):
        """
        Publishes the loaded map message periodically.
        """
        if self.map_msg:
            self.map_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.map_msg)
            self.get_logger().info("Published map periodically.")
        else:
            self.get_logger().warn(
                "Map message is not loaded, skipping periodic publish."
            )


def main(args=None):
    """
    Main function to initialize and run the ROS 2 node.
    """
    rclpy.init(args=args)
    single_map_publisher = SingleMapPublisher()
    rclpy.spin(single_map_publisher)
    single_map_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
