import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
import cv2
from cv_bridge import CvBridge
import numpy as np
import struct
from message_filters import Subscriber, ApproximateTimeSynchronizer
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import os  # Import os module for path manipulation
from ament_index_python.packages import (
    get_package_share_directory,
)  # Import for getting package share directory


class PointCloudMasker(Node):
    """
    A ROS2 node that subscribes to an Ouster PointCloud2 topic and a range image topic,
    allows the user to paint a mask in an OpenCV window, and then republishes
    the point cloud with points colored based on the mask.

    - Points corresponding to black pixels in the mask will be colored blue.
    - Points corresponding to white pixels in the mask will be colored red.
    """

    def __init__(self):
        super().__init__("pointcloud_masker")

        # Declare a parameter for the mask file path prefix, allowing it to be configured externally.
        # This will be used to generate names like 'ouster_mask_combined.png', 'ouster_mask_1.png', etc.
        self.declare_parameter("mask_file_prefix", "ouster_mask")
        self.mask_file_prefix = (
            self.get_parameter("mask_file_prefix").get_parameter_value().string_value
        )

        self.bridge = CvBridge()
        self.masks = (
            {}
        )  # Dictionary to store individual masks: {index: numpy_array_mask}
        self.original_mask_shape = None  # Stores (height, width) of the original mask
        self.mask_initialized = False  # Flag to ensure mask dimensions are set once
        self.current_mask_index = 0  # 0 for combined mask, 1+ for individual masks

        # Define QoS profile for sensor data, typically best effort for high-frequency data.
        qos_profile_sensor_data = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,  # Keep a small buffer
        )

        # Create subscribers for the point cloud and range image topics with best effort reliability.
        # Using message_filters.Subscriber for time synchronization.
        self.points_sub = Subscriber(
            self, PointCloud2, "/ouster/points", qos_profile=qos_profile_sensor_data
        )
        self.image_sub = Subscriber(
            self, Image, "/ouster/range_image", qos_profile=qos_profile_sensor_data
        )

        # Use ApproximateTimeSynchronizer to synchronize the point cloud and image messages.
        # queue_size=10: Buffer up to 10 messages.
        # slop=0.1: Allow messages to be up to 0.1 seconds apart. Adjust if needed.
        # allow_headerless=True: Allow messages without headers (though generally not recommended for sensor data).
        self.ts = ApproximateTimeSynchronizer(
            [self.points_sub, self.image_sub], 10, 0.1, allow_headerless=True
        )
        self.ts.registerCallback(self.synchronized_callback)

        # Create a publisher for the new, masked point cloud topic.
        self.masked_pointcloud_pub = self.create_publisher(
            PointCloud2, "/ouster/masked_points", 10
        )

        # Initialize the OpenCV window for mask editing and set the mouse callback.
        cv2.namedWindow("Mask Editor", cv2.WINDOW_NORMAL)

        # Set a fixed size for the window as requested (1800x300).
        # The image will be stretched to fit these dimensions.
        self.display_width = 1800
        self.display_height = 300
        cv2.resizeWindow("Mask Editor", self.display_width, self.display_height)

        cv2.setMouseCallback("Mask Editor", self.mouse_callback)

        # --- New: Define and create the mask saving directory ---
        try:
            # Get the share directory of the 'localization' package
            localization_share_directory = get_package_share_directory("localization")
            # Go up 4 directories from the share directory
            project_root = os.path.abspath(
                os.path.join(localization_share_directory, "..", "..", "..", "..")
            )
            # Define the subdirectory for masks in src/navigation/lidar_masks
            self.mask_save_directory = os.path.join(
                project_root, "src", "navigation", "lidar_masks"
            )
            # Create the directory if it doesn't exist
            os.makedirs(self.mask_save_directory, exist_ok=True)
            self.get_logger().info(
                f"Masks will be saved to: {self.mask_save_directory}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Could not set up mask save directory: {e}. Masks will be saved in current working directory."
            )
            self.mask_save_directory = "."  # Fallback to current directory

        self.get_logger().info(
            "PointCloudMasker node started. Open 'Mask Editor' window to draw."
        )
        self.get_logger().info(
            f"Mask files will be saved/loaded with prefix: {self.mask_file_prefix}"
        )
        self.get_logger().info(
            "Press 'a' to go to previous mask, 'd' to go to next mask."
        )
        self.get_logger().info(
            "Mask 0 is the combined mask (read-only). Masks 1+ are individual masks (editable)."
        )

    def _get_mask_filename(self, index):
        """Generates the full path filename for a mask based on its index and the save directory."""
        if index == 0:
            filename = f"{self.mask_file_prefix}_combined.png"
        else:
            filename = f"{self.mask_file_prefix}_{index}.png"
        return os.path.join(
            self.mask_save_directory, filename
        )  # Join with the save directory

    def _load_masks(self):
        """Loads existing individual mask files from disk."""
        self.masks = {}
        max_index = 0
        self.get_logger().info("Attempting to load existing masks...")
        for i in range(1, 100):  # Check for up to 99 individual masks
            filename = self._get_mask_filename(i)
            if os.path.exists(filename):
                try:
                    loaded_mask = cv2.imread(filename, cv2.IMREAD_GRAYSCALE)
                    if (
                        loaded_mask is not None
                        and loaded_mask.shape == self.original_mask_shape
                    ):
                        self.masks[i] = loaded_mask
                        max_index = i
                        self.get_logger().info(f"Loaded mask from {filename}")
                    else:
                        self.get_logger().warn(
                            f"Skipping {filename}: Invalid or shape mismatch. Expected {self.original_mask_shape}, got {loaded_mask.shape if loaded_mask is not None else 'None'}."
                        )
                except Exception as e:
                    self.get_logger().warn(f"Error loading {filename}: {e}")
            else:
                # Stop looking if a file is not found, assuming sequential numbering
                break

        if not self.masks:
            self.get_logger().info(
                "No existing individual masks found. Creating a new blank mask at index 1."
            )
            # New blank masks now default to black (0)
            self.masks[1] = np.zeros(self.original_mask_shape, dtype=np.uint8)
            max_index = 1

        self.current_mask_index = max_index  # Start at the last loaded mask or 1

    def _save_all_masks(self):
        """Saves all currently held masks (combined and individual) to disk."""
        # Save combined mask (index 0)
        if 0 in self.masks and self.masks[0] is not None:
            try:
                cv2.imwrite(self._get_mask_filename(0), self.masks[0])
                self.get_logger().debug(
                    f"Saved combined mask to {self._get_mask_filename(0)}"
                )
                # Shift the combined mask to the right by a fraction of the width based on shift_degrees
                shift_degrees = -10.0
                mask = self.masks[0]
                shift = int(mask.shape[1] * (shift_degrees / 360.0))
                shifted_mask = np.roll(mask, shift, axis=1)
                shifted_filename = self._get_mask_filename(0).replace(
                    ".png", f"_shifted.png"
                )
                cv2.imwrite(shifted_filename, shifted_mask)
                self.get_logger().info(
                    f"Saved shifted combined mask to {shifted_filename}"
                )
            except Exception as e:
                self.get_logger().error(f"Error saving combined mask: {e}")

        # Save an extra index for only the front
        index_save = 3
        if index_save in self.masks and self.masks is not None:
            try:
                cv2.imwrite(self._get_mask_filename(index_save), self.masks[index_save])
                self.get_logger().debug(
                    f"Saved front mask to {self._get_mask_filename(index_save)}"
                )

                # Shift the front mask to the right by a fraction of the width based on shift_degrees
                shift_degrees = -10.0
                mask = self.masks[index_save]
                shift = int(mask.shape[1] * (shift_degrees / 360.0))
                shifted_mask = np.roll(mask, shift, axis=1)
                shifted_filename = self._get_mask_filename(index_save).replace(
                    ".png", f"_shifted.png"
                )
                cv2.imwrite(shifted_filename, shifted_mask)
                self.get_logger().info(
                    f"Saved shifted front mask to {shifted_filename}"
                )
            except Exception as e:
                self.get_logger().error(f"Error saving front mask: {e}")

        # Save individual masks (indices > 0)
        for index, mask in self.masks.items():
            if index > 0 and mask is not None:
                try:
                    cv2.imwrite(self._get_mask_filename(index), mask)
                    self.get_logger().debug(
                        f"Saved individual mask {index} to {self._get_mask_filename(index)}"
                    )
                except Exception as e:
                    self.get_logger().error(f"Error saving mask {index}: {e}")

    def _generate_combined_mask(self):
        """Generates the combined mask (logical OR of all individual masks) and stores it at index 0."""
        if not self.original_mask_shape:
            self.get_logger().warn(
                "Cannot generate combined mask: original_mask_shape is not set."
            )
            return

        # Combined mask starts as all black (0)
        combined_mask = np.zeros(self.original_mask_shape, dtype=np.uint8)

        # Iterate through individual masks and apply logical OR to combine them.
        # If any individual mask has a white pixel (255) at a location,
        # the combined mask should also be white (255) at that location.
        for index in sorted(self.masks.keys()):
            if index > 0 and self.masks[index] is not None:
                combined_mask = cv2.bitwise_or(combined_mask, self.masks[index])

        self.masks[0] = combined_mask  # Store the combined mask at index 0
        self.get_logger().debug("Combined mask generated.")

    def _display_mask(self):
        """Helper function to resize and display the current mask image scaled to the fixed display size."""
        if (
            self.current_mask_index not in self.masks
            or self.masks[self.current_mask_index] is None
        ):
            self.get_logger().warn(
                f"Mask at index {self.current_mask_index} is not available for display."
            )
            return

        current_mask_to_display = self.masks[self.current_mask_index]

        # Resize the mask image to the predefined display dimensions.
        # Use INTER_NEAREST for masks to preserve sharp edges (binary values).
        display_mask = cv2.resize(
            current_mask_to_display,
            (self.display_width, self.display_height),
            interpolation=cv2.INTER_NEAREST,
        )

        # Add text overlay for current mask index
        text = f"Mask: {self.current_mask_index}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 1
        font_thickness = 2
        text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
        text_x = 10
        text_y = text_size[1] + 10

        # Draw background rectangle for text for better visibility
        cv2.rectangle(
            display_mask,
            (text_x - 5, text_y - text_size[1] - 5),
            (text_x + text_size[0] + 5, text_y + 5),
            (100),
            -1,
        )  # Gray background
        cv2.putText(
            display_mask,
            text,
            (text_x, text_y),
            font,
            font_scale,
            (255),
            font_thickness,
            cv2.LINE_AA,
        )  # White text

        cv2.imshow("Mask Editor", display_mask)

    def mouse_callback(self, event, x, y, flags, param):
        """
        Callback function for mouse events in the OpenCV mask editor window.
        Allows drawing black or white circles on the mask image.
        """
        # Check if masks are initialized and if we are on an editable mask (index > 0)
        if not self.mask_initialized or self.current_mask_index == 0:
            if self.current_mask_index == 0:
                self.get_logger().warn(
                    "Cannot draw on the combined mask (Mask 0). Please switch to an individual mask (Mask 1+)."
                )
            return

        # Get the current individual mask being edited
        current_editable_mask = self.masks[self.current_mask_index]

        # Scale mouse coordinates back to original mask image dimensions for drawing accuracy.
        original_mask_width = self.original_mask_shape[1]
        original_mask_height = self.original_mask_shape[0]

        # Calculate scaling factors from display size to original mask size
        scale_x = original_mask_width / self.display_width
        scale_y = original_mask_height / self.display_height

        draw_x = int(x * scale_x)
        draw_y = int(y * scale_y)

        # Ensure drawing coordinates are within bounds of the original mask image
        draw_x = max(0, min(draw_x, original_mask_width - 1))
        draw_y = max(0, min(draw_y, original_mask_height - 1))

        # Check for left mouse button press or drag (for drawing black)
        if event == cv2.EVENT_LBUTTONDOWN or (
            event == cv2.EVENT_MOUSEMOVE and flags == cv2.EVENT_FLAG_LBUTTON
        ):
            # Draw a black circle (0) at the scaled mouse position on the original mask image
            cv2.circle(
                current_editable_mask, (draw_x, draw_y), 5, (0), -1
            )  # -1 fills the circle
            self._generate_combined_mask()  # Regenerate combined mask after individual mask changes
            self._display_mask()  # Update the displayed mask
        # Check for right mouse button press or drag (for drawing white)
        elif (
            event == cv2.EVENT_RBUTTONDOWN
            or (event == cv2.EVENT_MOUSEMOVE and flags == cv2.EVENT_FLAG_RBUTTON)
            or event == cv2.EVENT_MBUTTONDOWN
            or (event == cv2.EVENT_MOUSEMOVE and flags == cv2.EVENT_FLAG_MBUTTON)
        ):
            # Draw a white circle (255) at the scaled mouse position on the original mask image
            cv2.circle(current_editable_mask, (draw_x, draw_y), 5, (255), -1)
            self._generate_combined_mask()  # Regenerate combined mask after individual mask changes
            self._display_mask()  # Update the displayed mask

    def handle_key_press(self, key):
        """Handles key presses for mask navigation."""
        if not self.mask_initialized:
            print(f"Cannot handle key press: masks not initialized yet.")
            return

        if key == ord("a"):  # 'a' key for previous mask
            if self.current_mask_index > 0:
                self.current_mask_index -= 1
                self.get_logger().info(f"Switched to Mask {self.current_mask_index}")
                self._display_mask()
            else:
                self.get_logger().info(
                    "Already at combined mask (Mask 0). Cannot go lower."
                )
        elif key == ord("d"):  # 'd' key for next mask
            next_index = self.current_mask_index + 1
            if next_index not in self.masks:
                # If the next mask doesn't exist, create a new blank one.
                if self.original_mask_shape:
                    # New masks should start as all-black (0)
                    self.masks[next_index] = np.zeros(
                        self.original_mask_shape, dtype=np.uint8
                    )
                    self.get_logger().info(
                        f"Created new blank mask at index {next_index}"
                    )
                else:
                    self.get_logger().warn(
                        "Cannot create new mask: original_mask_shape is not set."
                    )
                    return
            self.current_mask_index = next_index
            self.get_logger().info(f"Switched to Mask {self.current_mask_index}")
            self._display_mask()
        # You can add more key bindings here (e.g., 's' for save, 'c' for clear current mask)

    def synchronized_callback(self, points_msg, image_msg):
        """
        Callback function for synchronized PointCloud2 and Image messages.
        Initializes the mask, processes the point cloud, and publishes the result.
        """
        self.get_logger().debug("Received synchronized messages.")

        # Convert ROS Image message to an OpenCV image (numpy array).
        try:
            # Ouster range images are typically 16UC1 (16-bit unsigned, 1 channel).
            # We convert it to a standard OpenCV image for dimension reference.
            current_range_image = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="passthrough"
            )

            # Initialize the mask image dimensions and load/create masks once.
            if not self.mask_initialized:
                self.original_mask_shape = (
                    current_range_image.shape[0],
                    current_range_image.shape[1],
                )
                self._load_masks()  # Load existing masks or create initial ones
                self._generate_combined_mask()  # Generate the initial combined mask
                self._display_mask()  # Display the initial mask
                self.mask_initialized = True

        except Exception as e:
            self.get_logger().error(
                f"Error converting image message to OpenCV image: {e}"
            )
            return

        # Ensure masks are initialized before proceeding with point cloud processing.
        if not self.mask_initialized:
            self.get_logger().warn(
                "Masks not initialized yet. Skipping point cloud processing."
            )
            return

        # Use the combined mask (index 0) for processing the point cloud
        if 0 not in self.masks or self.masks[0] is None:
            self.get_logger().warn(
                "Combined mask (Mask 0) is not available. Skipping point cloud processing."
            )
            return

        self.process_point_cloud(
            points_msg,
            self.masks[self.current_mask_index],
            image_msg.width,
            image_msg.height,
        )

        # Save all masks periodically.
        self._save_all_masks()

    def process_point_cloud(self, points_msg, mask_image, image_width, image_height):
        """
        Processes the input PointCloud2 message, applies the mask, and
        creates a new PointCloud2 message with colored points.
        """
        # Create a new PointCloud2 message for publishing, copying header and basic info.
        output_points_msg = PointCloud2()
        output_points_msg.header = points_msg.header
        output_points_msg.height = points_msg.height
        output_points_msg.width = points_msg.width
        output_points_msg.is_dense = points_msg.is_dense
        output_points_msg.is_bigendian = points_msg.is_bigendian

        # Copy existing fields and add an 'rgb' field if it doesn't exist.
        output_points_msg.fields = points_msg.fields[:]
        from sensor_msgs.msg import PointField

        rgb_field_found = False
        rgb_field_offset = -1
        for field in output_points_msg.fields:
            if field.name == "rgb":
                rgb_field_found = True
                rgb_field_offset = field.offset
                break

        if not rgb_field_found:
            # If 'rgb' field is not present, add it.
            # The offset for 'rgb' will be at the end of the existing point data.
            # PointField.UINT32 is used for packed RGB (0x00RRGGBB).
            rgb_field_offset = (
                points_msg.point_step
            )  # Offset will be after existing fields
            output_points_msg.fields.append(
                PointField(
                    name="rgb",
                    offset=rgb_field_offset,
                    datatype=PointField.UINT32,
                    count=1,
                )
            )
            output_points_msg.point_step = (
                points_msg.point_step + 4
            )  # Add 4 bytes for RGB
        else:
            # If 'rgb' field already exists, use the original point_step.
            output_points_msg.point_step = points_msg.point_step

        # Calculate the row step for the output message.
        output_points_msg.row_step = (
            output_points_msg.point_step * output_points_msg.width
        )
        # Initialize the data byte array for the output point cloud.
        output_points_msg.data = bytearray(
            output_points_msg.row_step * output_points_msg.height
        )

        # Find offsets for x, y, z fields in the input message.
        x_offset, y_offset, z_offset = -1, -1, -1
        for field in points_msg.fields:
            if field.name == "x":
                x_offset = field.offset
            if field.name == "y":
                y_offset = field.offset
            if field.name == "z":
                z_offset = field.offset

        if x_offset == -1 or y_offset == -1 or z_offset == -1:
            self.get_logger().error(
                "Could not find 'x', 'y', or 'z' fields in input PointCloud2 message. Cannot process."
            )
            return

        point_count = points_msg.width * points_msg.height

        # Iterate through each point in the input point cloud.
        for i in range(points_msg.height):
            for j in range(points_msg.width):
                # Calculate the byte offset for the current point in the input data.
                input_point_offset = (i * points_msg.row_step) + (
                    j * points_msg.point_step
                )

                # Extract x, y, z coordinates from the input point cloud.
                # '<f' specifies little-endian float.
                x = struct.unpack_from(
                    "<f", points_msg.data, input_point_offset + x_offset
                )[0]
                y = struct.unpack_from(
                    "<f", points_msg.data, input_point_offset + y_offset
                )[0]
                z = struct.unpack_from(
                    "<f", points_msg.data, input_point_offset + z_offset
                )[0]

                # Determine the color based on the mask image.
                # IMPORTANT ASSUMPTION: The point cloud points are ordered such that
                # (row i, column j) in the range image directly corresponds to the
                # (i * width + j)th point in the flattened point cloud data.
                # This is a common convention for organized point clouds from sensors like Ouster.
                if i < mask_image.shape[0] and j < mask_image.shape[1]:
                    mask_pixel_value = mask_image[i, j]
                else:
                    # Default to black (blue color) if no direct mask mapping.
                    mask_pixel_value = 0

                # Assign blue (0,0,255) or red (255,0,0) based on mask pixel value.
                # OpenCV uses BGR by default, but for RGB packed into UINT32, it's RRGGBB.
                if mask_pixel_value == 0:  # Black pixel in mask (0) -> Blue color
                    r, g, b = 0, 0, 255
                else:  # White pixel in mask (255) -> Red color
                    r, g, b = 255, 0, 0

                # Pack RGB components into a single UINT32 for the 'rgb' field.
                # Format: 0x00RRGGBB (alpha is 00, then Red, Green, Blue bytes)
                rgb_packed = (r << 16) | (g << 8) | b

                # Calculate the byte offset for the current point in the output data.
                output_point_offset = (i * output_points_msg.row_step) + (
                    j * output_points_msg.point_step
                )

                # Copy original XYZ data to the output point cloud.
                struct.pack_into(
                    "<fff",
                    output_points_msg.data,
                    output_point_offset + x_offset,
                    x,
                    y,
                    z,
                )

                # Pack the calculated RGB data into the 'rgb' field of the output point.
                if rgb_field_offset != -1:
                    struct.pack_into(
                        "<I",
                        output_points_msg.data,
                        output_point_offset + rgb_field_offset,
                        rgb_packed,
                    )
                else:
                    self.get_logger().error(
                        "RGB field offset not found in output PointCloud2 fields after creation. This should not happen."
                    )
                    return

        # Publish the newly created masked point cloud message.
        self.masked_pointcloud_pub.publish(output_points_msg)
        self.get_logger().debug(
            f"Published masked point cloud with {point_count} points."
        )

    def destroy_node(self):
        """
        Clean up function called when the node is destroyed.
        Ensures all mask images are saved and OpenCV windows are closed.
        """
        self._save_all_masks()  # Save all masks before shutdown
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init(args=args)  # Initialize ROS2 client library
    node = PointCloudMasker()  # Create an instance of our node

    try:
        # Spin the node in a loop. rclpy.spin_once processes pending callbacks.
        # cv2.waitKey(1) allows OpenCV to process GUI events and keeps the window responsive.
        while rclpy.ok():
            rclpy.spin_once(
                node, timeout_sec=0.01
            )  # Process ROS callbacks with a small timeout
            key = cv2.waitKey(1) & 0xFF  # Capture key press
            if key != 255:  # 255 means no key was pressed
                node.handle_key_press(key)  # Handle key press
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully.
        node.get_logger().info("Node stopped by user (KeyboardInterrupt).")
    finally:
        # Ensure the node is properly destroyed and resources are released.
        node.destroy_node()
        rclpy.shutdown()  # Shut down ROS2 client library


if __name__ == "__main__":
    main()
