import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, Quaternion, Point
from std_srvs.srv import Trigger
from robot_localization.srv import FromLL, ToLL  # Import ToLL service

import os
import yaml
import cv2
import numpy as np
import math
from threading import Event


class MapLoader(Node):
    """
    A ROS2 node that loads and publishes an OccupancyGrid map based on
    current GPS location and local map files. It can combine multiple
    overlapping maps and fills uncovered areas with unknown values.
    """

    def __init__(self):
        super().__init__("map_loader")

        # Declare parameters for map directory, the size of the square region of interest,
        # and the GPS covariance threshold.
        self.declare_parameter(
            "map_directory", "/usr/local/cprt/elevation_datasets/CanElevation/"
        )
        self.declare_parameter("map_square_size_m", 500.0)  # Default to 500m square
        self.declare_parameter(
            "gps_covariance_threshold", 0.5
        )  # New parameter for covariance threshold

        self.map_directory = (
            self.get_parameter("map_directory").get_parameter_value().string_value
        )
        self.map_square_size_m = (
            self.get_parameter("map_square_size_m").get_parameter_value().double_value
        )
        self.gps_covariance_threshold = (
            self.get_parameter("gps_covariance_threshold")
            .get_parameter_value()
            .double_value
        )

        self.get_logger().info(f"Initialized MapLoader Node.")
        self.get_logger().info(f"Map directory set to: {self.map_directory}")
        self.get_logger().info(
            f"Map square size for region of interest: {self.map_square_size_m} meters"
        )
        self.get_logger().info(
            f"GPS covariance threshold set to: {self.gps_covariance_threshold}"
        )

        # Callback groups to prevent deadlocks with multiple subscriptions/services
        self.gps_callback_group = MutuallyExclusiveCallbackGroup()
        self.service_callback_group = MutuallyExclusiveCallbackGroup()

        # GPS Subscriber: Subscribes to /gps/fix to get current location data.
        # QoS profile ensures reliable delivery of the latest GPS fix.
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.latest_gps_fix = None  # Stores the most recent good GPS fix
        self.gps_fix_subscriber = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_fix_callback,
            qos_profile,
            callback_group=self.gps_callback_group,
        )
        self.get_logger().info("GPS Fix subscriber created on /gps/fix.")

        # Map Publisher: Publishes the generated OccupancyGrid map.
        self.map_publisher = self.create_publisher(
            OccupancyGrid, "/map_can_elevation", qos_profile
        )
        self.get_logger().info("OccupancyGrid publisher created on /map_can_elevation.")

        # FromLL Service Client: Used to convert Lat/Lon/Alt to map coordinates.
        self.fromLL_client = self.create_client(
            FromLL, "fromLL", callback_group=self.service_callback_group
        )
        # Wait for the fromLL service to be available before proceeding
        while not self.fromLL_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("fromLL service not available, waiting again...")
        self.get_logger().info("fromLL service client successfully connected.")

        # ToLL Service Client: Used to convert map coordinates to Lat/Lon/Alt.
        self.toLL_client = self.create_client(
            ToLL, "toLL", callback_group=self.service_callback_group
        )
        # Wait for the toLL service to be available before proceeding
        while not self.toLL_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("toLL service not available, waiting again...")
        self.get_logger().info("toLL service client successfully connected.")

        # Map Generation Service: Allows external triggers for map generation.
        self.generate_map_service = self.create_service(
            Trigger,
            "generate_map_service",
            self.load_maps_service_callback,
            callback_group=self.service_callback_group,
        )
        self.get_logger().info("Generate Map service created on /generate_map_service.")

        # Call load_maps once at startup as required
        self.get_logger().info("Triggering initial map loading on startup...")
        # Use a timer to call load_maps to ensure ROS2 is fully spun up,
        # especially for service calls.
        self.timer = self.create_timer(1.0, self._initial_map_loading_timer_callback)

    def _initial_map_loading_timer_callback(self):
        """Callback for the one-shot timer to trigger initial map loading."""
        self.destroy_timer(self.timer)  # Destroy the timer after its first call
        self.get_logger().info("Initial map loading timer triggered.")
        self.load_maps()

    def gps_fix_callback(self, msg: NavSatFix):
        """
        Callback for the /gps/fix topic. Stores the latest GPS fix if its
        position covariance (X and Y components) is below the specified threshold.
        Altitude covariance is ignored as per user request.
        """
        # Check if covariance type is known or approximated and variances are low
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        if (
            msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_KNOWN
            or msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        ):
            # NavSatFix.position_covariance is a float64[9] array.
            # Indices 0 and 4 correspond to X and Y variances respectively.
            # Index 8 corresponds to Z (altitude) variance.
            # Only checking X and Y variances as per user request.
            if (
                msg.position_covariance[0] < self.gps_covariance_threshold
                and msg.position_covariance[4] < self.gps_covariance_threshold
            ):
                self.latest_gps_fix = msg
                self.get_logger().debug(
                    f"Good GPS fix received: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}"
                )
            else:
                self.get_logger().debug(
                    f"GPS fix covariance too high (threshold: {self.gps_covariance_threshold:.2f}): x={msg.position_covariance[0]:.2f}, "
                    f"y={msg.position_covariance[4]:.2f}"
                )
        else:
            self.get_logger().debug(
                "GPS fix covariance type not known or approximated, skipping."
            )

    def load_maps_service_callback(
        self, request: Trigger.Request, response: Trigger.Response
    ):
        """
        Callback for the generate_map_service. Triggers map loading.
        """
        self.get_logger().info("Generate Map service called. Triggering map loading...")
        self.load_maps()
        response.success = True
        response.message = "Map loading triggered."
        return response

    def convert_lat_lon_to_pose(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
        orientation: Quaternion,
    ) -> PoseStamped:
        """
        Converts a latitude, longitude, altitude, and orientation to a PoseStamped
        in the map frame using the 'fromLL' service.
        This function uses threading.Event to wait for the service response,
        with a timeout for graceful shutdown.
        """
        req = FromLL.Request()
        req.ll_point.longitude = longitude
        req.ll_point.latitude = latitude
        req.ll_point.altitude = altitude

        self.get_logger().info(
            f"Requesting map pose for: Lat={req.ll_point.latitude:.8f}, Long={req.ll_point.longitude:.8f}, Alt={req.ll_point.altitude:.8f}"
        )

        try:
            event = Event()  # Local event for this specific service call

            def done_callback(future):
                nonlocal event
                event.set()
                self.get_logger().info("fromLL service call completed, setting event.")

            future = self.fromLL_client.call_async(req)
            future.add_done_callback(done_callback)

            # Wait for the service call to complete with a timeout.
            # This prevents infinite blocking if the service never responds.
            # A timeout of 5 seconds is chosen as a reasonable wait time.
            if not event.wait(timeout=60.0):
                self.get_logger().warn("Timeout waiting for fromLL service response.")
                if not rclpy.ok():
                    self.get_logger().info(
                        "ROS2 context is shutting down, aborting lat/lon conversion."
                    )
                return None  # Return None if timeout occurs or ROS2 is shutting down

            # Check if the future completed successfully
            if future.done() and future.exception() is None:
                result = future.result()
                if result:
                    target_pose = PoseStamped()
                    target_pose.header.frame_id = "map"
                    target_pose.header.stamp = self.get_clock().now().to_msg()
                    target_pose.pose.position = result.map_point
                    target_pose.pose.orientation = orientation
                    self.get_logger().info(
                        f"Converted to map frame: x={target_pose.pose.position.x:.2f}, y={target_pose.pose.position.y:.2f}"
                    )

                    # hack for indoor testing (commented out as in original)
                    # target_pose.pose.position.x = 50.0
                    # target_pose.pose.position.y = 0.0
                    # target_pose.pose.position.z = 0.0

                    return target_pose
                else:
                    self.get_logger().error(
                        "Failed to convert lat/lon to map pose: Service returned no result."
                    )
                    return None
            else:
                self.get_logger().error(
                    f"fromLL service call failed: {future.exception()}"
                )
                return None
        except Exception as e:
            self.get_logger().error(f"Error during lat/lon conversion: {e}")
            return None

    def convert_pose_to_lat_lon(self, map_point: Point) -> dict:
        """
        Converts a Point in the map frame to latitude, longitude, and altitude
        using the 'toLL' service.
        This function uses threading.Event to wait for the service response,
        with a timeout for graceful shutdown.
        """
        req = ToLL.Request()
        req.map_point = map_point

        self.get_logger().info(
            f"Requesting Lat/Lon for map point: x={req.map_point.x:.2f}, y={req.map_point.y:.2f}, z={req.map_point.z:.2f}"
        )

        try:
            event = Event()

            def done_callback(future):
                nonlocal event
                event.set()

            future = self.toLL_client.call_async(req)
            future.add_done_callback(done_callback)

            if not event.wait(timeout=5.0):
                self.get_logger().warn("Timeout waiting for toLL service response.")
                if not rclpy.ok():
                    self.get_logger().info(
                        "ROS2 context is shutting down, aborting map to lat/lon conversion."
                    )
                return None

            if future.done() and future.exception() is None:
                result = future.result()
                if result:
                    self.get_logger().info(
                        f"Converted to Lat/Lon: Lat={result.ll_point.latitude:.8f}, Lon={result.ll_point.longitude:.8f}, Alt={result.ll_point.altitude:.8f}"
                    )
                    return {
                        "latitude": result.ll_point.latitude,
                        "longitude": result.ll_point.longitude,
                        "altitude": result.ll_point.altitude,
                    }
                else:
                    self.get_logger().error(
                        "Failed to convert map pose to lat/lon: Service returned no result."
                    )
                    return None
            else:
                self.get_logger().error(
                    f"toLL service call failed: {future.exception()}"
                )
                return None
        except Exception as e:
            self.get_logger().error(f"Error during map to lat/lon conversion: {e}")
            return None

    def load_maps(self):
        """
        The main function to load and publish the OccupancyGrid map.
        It waits for a good GPS fix, finds suitable map files, processes them,
        combines them, and publishes the OccupancyGrid.
        """
        self.get_logger().info("Starting map loading process...")

        # Reset latest_gps_fix to ensure we wait for a new, good reading
        self.latest_gps_fix = None
        self.get_logger().info(
            f"Resetting GPS fix and waiting for a new good GPS fix (covariance < {self.gps_covariance_threshold:.2f})..."
        )

        # 1. Wait for a good GPS fix
        # Loop until a good GPS fix is received. rclpy.spin_once allows callbacks to run.
        while self.latest_gps_fix is None:
            self.get_logger().info(
                "No good GPS fix yet, spinning once to allow callbacks..."
            )
            rclpy.spin_once(self, timeout_sec=0.5)  # Process pending callbacks
            if not rclpy.ok():  # Check if ROS2 context is still valid
                self.get_logger().info(
                    "ROS2 shutdown detected while waiting for GPS. Aborting map loading."
                )
                return

        current_gps = self.latest_gps_fix
        self.get_logger().info(
            f"Using current good GPS fix: Lat={current_gps.latitude:.6f}, Lon={current_gps.longitude:.6f}, Alt={current_gps.altitude:.2f}"
        )

        # A dummy identity quaternion is used as orientation is not relevant for position conversion
        dummy_orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        # 2. Define the query square region centered on the current GPS location in Lat/Lon
        # First, convert the current GPS location to map coordinates
        current_map_pose = self.convert_lat_lon_to_pose(
            current_gps.latitude,
            current_gps.longitude,
            current_gps.altitude,
            dummy_orientation,
        )

        if current_map_pose is None:
            self.get_logger().error(
                "Could not convert current GPS to map coordinates. Aborting map loading."
            )
            return

        current_map_x = current_map_pose.pose.position.x
        current_map_y = current_map_pose.pose.position.y

        half_square_size_map = self.map_square_size_m / 2.0

        # Calculate the four corners of the query square in map coordinates
        corners_map_frame = [
            Point(
                x=current_map_x - half_square_size_map,
                y=current_map_y - half_square_size_map,
                z=0.0,
            ),  # Bottom-Left
            Point(
                x=current_map_x + half_square_size_map,
                y=current_map_y - half_square_size_map,
                z=0.0,
            ),  # Bottom-Right
            Point(
                x=current_map_x - half_square_size_map,
                y=current_map_y + half_square_size_map,
                z=0.0,
            ),  # Top-Left
            Point(
                x=current_map_x + half_square_size_map,
                y=current_map_y + half_square_size_map,
                z=0.0,
            ),  # Top-Right
        ]

        # Convert these map coordinates back to Lat/Lon using the toLL service
        converted_corners_ll = []
        for corner_map_point in corners_map_frame:
            ll_point = self.convert_pose_to_lat_lon(corner_map_point)
            if ll_point:
                converted_corners_ll.append(ll_point)
            else:
                self.get_logger().error(
                    f"Failed to convert map corner {corner_map_point} to Lat/Lon. Aborting map loading."
                )
                return

        # Determine the min/max Lat/Lon for the query region from the converted corners
        query_min_lat = min(c["latitude"] for c in converted_corners_ll)
        query_max_lat = max(c["latitude"] for c in converted_corners_ll)
        query_min_lon = min(c["longitude"] for c in converted_corners_ll)
        query_max_lon = max(c["longitude"] for c in converted_corners_ll)

        self.get_logger().info(
            f"Query region in Lat/Lon (via toLL service): Lat[{query_min_lat:.6f}, {query_max_lat:.6f}], Lon[{query_min_lon:.6f}, {query_max_lon:.6f}]"
        )

        # 3. Find all suitable map files that intersect with the query region (Lat/Lon check)
        intersecting_maps_data = []

        # Walk through the specified map directory to find costmap subdirectories
        for root, dirs, files in os.walk(self.map_directory):
            if "costmaps" in dirs:
                costmaps_dir = os.path.join(root, "costmaps")
                self.get_logger().info(
                    f"Searching for maps in subdirectory: {costmaps_dir}"
                )

                yaml_files = [
                    f for f in os.listdir(costmaps_dir) if f.endswith(".yaml")
                ]
                self.get_logger().info(
                    f"Found {len(yaml_files)} YAML files in {costmaps_dir}."
                )

                for yaml_file in yaml_files:
                    yaml_path = os.path.join(costmaps_dir, yaml_file)
                    try:
                        with open(yaml_path, "r") as f:
                            map_metadata = yaml.safe_load(f)

                        # Extract required metadata fields from the YAML file
                        map_filename = map_metadata.get("filename")
                        resolution_m = map_metadata.get("resolution_m")
                        corners_lat_lon = map_metadata.get("corners_lat_lon")
                        value_scale = map_metadata.get("value_scale")
                        center_lat_lon = map_metadata.get("center_lat_lon")

                        if not all(
                            [
                                map_filename,
                                resolution_m is not None,
                                corners_lat_lon,
                                value_scale,
                                center_lat_lon,
                            ]
                        ):
                            self.get_logger().warn(
                                f"Skipping {yaml_file}: Missing one or more required metadata fields."
                            )
                            continue

                        png_path = os.path.join(
                            costmaps_dir, map_filename
                        )  # PNG is in the same costmaps directory
                        if not os.path.exists(png_path):
                            self.get_logger().warn(
                                f"Skipping {yaml_file}: Corresponding PNG file '{map_filename}' not found at '{png_path}'."
                            )
                            continue

                        # Extract map corners directly in Lat/Lon
                        bl_lat = corners_lat_lon["bottom_left"]["lat"]
                        bl_lon = corners_lat_lon["bottom_left"]["lon"]
                        tr_lat = corners_lat_lon["top_right"]["lat"]
                        tr_lon = corners_lat_lon["top_right"]["lon"]

                        # Determine the bounding box of the map in Lat/Lon
                        map_min_lat = min(bl_lat, tr_lat)
                        map_max_lat = max(bl_lat, tr_lat)
                        map_min_lon = min(bl_lon, tr_lon)
                        map_max_lon = max(bl_lon, tr_lon)

                        # Check for intersection between map bounding box (Lat/Lon) and query region (Lat/Lon)
                        intersects_lat = not (
                            map_max_lat < query_min_lat or map_min_lat > query_max_lat
                        )
                        intersects_lon = not (
                            map_max_lon < query_min_lon or map_min_lon > query_max_lon
                        )

                        if intersects_lat and intersects_lon:
                            self.get_logger().info(
                                f"Map '{map_filename}' intersects with the current region of interest (Lat/Lon check)."
                            )
                            # If it intersects, convert its corners to map coordinates for later combination
                            bl_map_pose = self.convert_lat_lon_to_pose(
                                bl_lat, bl_lon, 0.0, dummy_orientation
                            )
                            tr_map_pose = self.convert_lat_lon_to_pose(
                                tr_lat, tr_lon, 0.0, dummy_orientation
                            )

                            if bl_map_pose is None or tr_map_pose is None:
                                self.get_logger().warn(
                                    f"Skipping {yaml_file}: Could not convert map corners to map coordinates for combination."
                                )
                                continue

                            map_min_x = min(
                                bl_map_pose.pose.position.x, tr_map_pose.pose.position.x
                            )
                            map_max_x = max(
                                bl_map_pose.pose.position.x, tr_map_pose.pose.position.x
                            )
                            map_min_y = min(
                                bl_map_pose.pose.position.y, tr_map_pose.pose.position.y
                            )
                            map_max_y = max(
                                bl_map_pose.pose.position.y, tr_map_pose.pose.position.y
                            )

                            intersecting_maps_data.append(
                                {
                                    "png_path": png_path,
                                    "metadata": map_metadata,
                                    "map_min_x": map_min_x,
                                    "map_max_x": map_max_x,
                                    "map_min_y": map_min_y,
                                    "map_max_y": map_max_y,
                                }
                            )
                        else:
                            self.get_logger().debug(
                                f"Map '{map_filename}' in {costmaps_dir} does not intersect (Lat/Lon check). Map bounds: Lat[{map_min_lat:.6f}, {map_max_lat:.6f}], Lon[{map_min_lon:.6f}, {map_max_lon:.6f}]. Query bounds: Lat[{query_min_lat:.6f}, {query_max_lat:.6f}], Lon[{query_min_lon:.6f}, {query_max_lon:.6f}]."
                            )

                    except yaml.YAMLError as e:
                        self.get_logger().error(
                            f"Error parsing YAML file '{yaml_file}' in {costmaps_dir}: {e}"
                        )
                    except Exception as e:
                        self.get_logger().error(
                            f"An unexpected error occurred while processing '{yaml_file}' in {costmaps_dir}: {e}"
                        )

        if not intersecting_maps_data:
            self.get_logger().warn(
                "No maps found that intersect with the current GPS location's region of interest. Cannot generate map."
            )
            return

        # 4. Determine the overall bounding box for the combined map
        # Initialize with the bounds of the first intersecting map
        global_min_x = float("inf")
        global_max_x = float("-inf")
        global_min_y = float("inf")
        global_max_y = float("-inf")

        # The current_map_pose is already in map coordinates from step 2.
        # Define the query square region centered on the current GPS location in map coordinates
        # This is needed to ensure the final published map covers the requested square size
        half_square_size_map = self.map_square_size_m / 2.0
        query_min_x_map = current_map_x - half_square_size_map
        query_max_x_map = current_map_x + half_square_size_map
        query_min_y_map = current_map_y - half_square_size_map
        query_max_y_map = current_map_y + half_square_size_map

        global_min_x = min(global_min_x, query_min_x_map)
        global_max_x = max(global_max_x, query_max_x_map)
        global_min_y = min(global_min_y, query_min_y_map)
        global_max_y = max(global_max_y, query_max_y_map)

        # Find the overall min/max from all intersecting maps (which are now in map frame)
        for map_data in intersecting_maps_data:
            global_min_x = min(global_min_x, map_data["map_min_x"])
            global_max_x = max(global_max_x, map_data["map_max_x"])
            global_min_y = min(global_min_y, map_data["map_min_y"])
            global_max_y = max(global_max_y, map_data["map_max_y"])

        # Assume a consistent resolution across all maps for simplicity in combination
        # For more robustness, one might need to resample maps to a common resolution.
        # For now, we take the resolution from the first map, assuming consistency.
        resolution = intersecting_maps_data[0]["metadata"]["resolution_m"]

        # Calculate dimensions of the combined OccupancyGrid
        combined_width_m = global_max_x - global_min_x
        combined_height_m = global_max_y - global_min_y

        combined_width_pixels = int(math.ceil(combined_width_m / resolution))
        combined_height_pixels = int(math.ceil(combined_height_m / resolution))

        self.get_logger().info(
            f"Combined map dimensions: {combined_width_pixels}x{combined_height_pixels} pixels at {resolution}m/pixel."
        )
        self.get_logger().info(
            f"Combined map origin (map frame): X={global_min_x:.2f}, Y={global_min_y:.2f}"
        )

        # Initialize the combined OccupancyGrid data with -1 (unknown)
        combined_occupancy_data = np.full(
            (combined_height_pixels, combined_width_pixels), -1, dtype=np.int8
        )

        # 5. Load and combine PNG images into the global OccupancyGrid
        for map_data in intersecting_maps_data:
            png_path = map_data["png_path"]
            map_metadata = map_data["metadata"]

            image = cv2.imread(png_path, cv2.IMREAD_GRAYSCALE)
            if image is None:
                self.get_logger().error(
                    f"Failed to load image from '{png_path}'. Skipping this map for combination."
                )
                continue

            height, width = image.shape
            value_min = map_metadata["value_scale"]["min"]
            value_max = map_metadata["value_scale"]["max"]
            value_unknown = map_metadata["value_scale"]["unknown"]

            # Calculate the bottom-left corner of the current map in the 'map' frame
            # We already have map_min_x, map_min_y for this specific map from previous steps
            current_map_bl_x = map_data["map_min_x"]
            current_map_bl_y = map_data["map_min_y"]

            # Calculate the pixel offset of this map's bottom-left corner relative
            # to the combined map's bottom-left origin.
            offset_x_pixels = int(round((current_map_bl_x - global_min_x) / resolution))
            offset_y_pixels = int(round((current_map_bl_y - global_min_y) / resolution))

            self.get_logger().debug(
                f"Processing map '{map_metadata['filename']}'. Pixel offset: ({offset_x_pixels}, {offset_y_pixels})"
            )

            for r_cv in range(height):
                for c_cv in range(width):
                    pixel_value = image[r_cv, c_cv]

                    # Convert image pixel value to OccupancyGrid format (0-100 or -1)
                    if pixel_value == value_unknown:
                        new_occupancy_value = -1  # -1 for unknown
                    else:
                        new_occupancy_value = int(
                            round(
                                (pixel_value - value_min)
                                / (value_max - value_min)
                                * 100.0
                            )
                        )

                    # Calculate corresponding pixel in the combined grid (ROS convention: bottom-left origin)
                    # OpenCV image (r_cv, c_cv) is top-left origin.
                    # To get ROS map row (bottom-up), we invert r_cv: (height - 1 - r_cv)
                    # Then add the global offset.
                    global_row_ros = (height - 1 - r_cv) + offset_y_pixels
                    global_col_ros = c_cv + offset_x_pixels

                    # Ensure the calculated global pixel is within the bounds of the combined grid
                    if (
                        0 <= global_row_ros < combined_height_pixels
                        and 0 <= global_col_ros < combined_width_pixels
                    ):

                        current_combined_value = combined_occupancy_data[
                            global_row_ros, global_col_ros
                        ]

                        # Apply max cost logic for overlapping areas
                        if new_occupancy_value == -1:
                            # If new value is unknown, keep existing value (could be known or unknown)
                            pass
                        elif current_combined_value == -1:
                            # If current is unknown, and new is known, take new known value
                            combined_occupancy_data[global_row_ros, global_col_ros] = (
                                new_occupancy_value
                            )
                        else:
                            # Both are known, take the maximum cost
                            combined_occupancy_data[global_row_ros, global_col_ros] = (
                                max(current_combined_value, new_occupancy_value)
                            )
                    # else:
                    #     self.get_logger().debug(f"Calculated global pixel ({global_col_ros}, {global_row_ros}) out of bounds for combined map.")

        # Flatten the NumPy array to a list for OccupancyGrid message
        final_occupancy_data_list = combined_occupancy_data.flatten().tolist()

        # 6. Create and populate the OccupancyGrid message
        occupancy_grid_msg = OccupancyGrid()
        occupancy_grid_msg.header.frame_id = "map"  # Map frame
        occupancy_grid_msg.header.stamp = self.get_clock().now().to_msg()

        occupancy_grid_msg.info.resolution = float(resolution)
        occupancy_grid_msg.info.width = combined_width_pixels
        occupancy_grid_msg.info.height = combined_height_pixels

        # Set the origin of the combined OccupancyGrid
        occupancy_grid_msg.info.origin.position.x = global_min_x
        occupancy_grid_msg.info.origin.position.y = global_min_y
        occupancy_grid_msg.info.origin.position.z = 0.0  # Assuming a 2D map
        # Assuming the map is axis-aligned with the 'map' frame, so orientation is identity
        dummy_orientation = Quaternion(
            x=0.0, y=0.0, z=0.0, w=1.0
        )  # Re-define if not in scope
        occupancy_grid_msg.info.origin.orientation = dummy_orientation

        occupancy_grid_msg.data = final_occupancy_data_list

        # 7. Publish the OccupancyGrid map
        self.map_publisher.publish(occupancy_grid_msg)
        self.get_logger().info(f"Successfully published combined OccupancyGrid map.")


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    Uses a MultiThreadedExecutor to handle multiple callbacks concurrently.
    """
    rclpy.init(args=args)
    # MultiThreadedExecutor is crucial when you have multiple subscriptions or
    # service clients/servers that might block each other if using a SingleThreadedExecutor.
    executor = MultiThreadedExecutor()
    map_loader_node = MapLoader()
    executor.add_node(map_loader_node)

    try:
        executor.spin()  # Spin the executor to process callbacks
    except KeyboardInterrupt:
        map_loader_node.get_logger().info(
            "Keyboard Interrupt received. Shutting down..."
        )
    finally:
        # Cleanup: shutdown executor and destroy node
        executor.shutdown()
        map_loader_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
