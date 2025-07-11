import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import time
import numpy as np # Import numpy

class OusterRangeImageSaverSingle(Node):
    def __init__(self):
        super().__init__('ouster_range_image_saver_single')

        # Declare the parameter for the save directory
        self.declare_parameter('save_directory', '~/Pictures')
        self.save_directory = self.get_parameter('save_directory').get_parameter_value().string_value
        
        # Expand the user home directory if '~' is used
        self.save_directory = os.path.expanduser(self.save_directory)

        # Create the directory if it doesn't exist
        if not os.path.exists(self.save_directory):
            try:
                os.makedirs(self.save_directory)
                self.get_logger().info(f"Created save directory: {self.save_directory}")
            except OSError as e:
                self.get_logger().error(f"Error creating directory {self.save_directory}: {e}")
                # Fallback to current working directory if creation fails
                self.save_directory = os.getcwd()
                self.get_logger().warn(f"Saving to current working directory: {self.save_directory}")

        self.subscription = self.create_subscription(
            Image,
            '/ouster/range_image',
            self.image_callback,
            rclpy.qos.qos_profile_sensor_data # Use sensor data QoS profile
        )
        self.bridge = CvBridge()
        self.image_saved = False # Flag to ensure only one image is saved
        self.get_logger().info(f'Ouster Range Image Saver (Single) node started. Will save one image to: {self.save_directory}')
        self.get_logger().info('Waiting for the first image...')

    def image_callback(self, msg):
        if self.image_saved:
            # If an image has already been saved, do nothing
            return

        self.get_logger().info('Received first image. Saving...')

        try:
            # First, convert ROS Image message to OpenCV image using 'passthrough'
            # to preserve the original 16-bit encoding.
            cv_image_16bit = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # Check the actual encoding of the received image
            if msg.encoding == "mono16":
                self.get_logger().info("Image encoding is mono16. Scaling to 8-bit for visualization.")
                
                # Normalize the 16-bit image (0-65535) to 8-bit (0-255)
                # This is the crucial step to make the image visible.
                # cv2.normalize scales pixel values from the min/max of the input array
                # to the specified output range (0-255).
                cv_image_8bit = cv2.normalize(cv_image_16bit, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
                
                # For images where 0 means no return, and we want it to stay black
                # Make sure these values are explicitly zeroed out after normalization,
                # as normalize might map them to something else if they are part of the
                # min/max range.
                # Find the minimum possible value for range data (e.g., if there's a 0 in the 16bit data)
                # It's common for 0 in range data to mean no valid return.
                # If cv_image_16bit is directly range (e.g. in mm), 0 usually means no detection.
                # We can check if any pixel was originally 0 in the 16-bit image and set it to 0 in the 8-bit image.
                # This depends on how the ouster driver encodes "no return" in mono16.
                # Often, a 0 value in the range image already means no return.
                # If your 0s are getting scaled up, you might need:
                # cv_image_8bit[cv_image_16bit == 0] = 0 
                # However, cv2.normalize with NORM_MINMAX should ideally map the true min to 0.
                # If min_range is also 0, then 0s will stay 0. If min_range is >0, then 0s will become dark, but not necessarily black.
                # Let's rely on NORM_MINMAX which usually handles this well by mapping the lowest *actual* value to 0.

            elif msg.encoding == "mono8":
                self.get_logger().info("Image encoding is mono8. No scaling needed.")
                cv_image_8bit = cv_image_16bit # Already 8-bit, just reassign
            else:
                self.get_logger().warn(f"Unsupported image encoding: {msg.encoding}. Attempting to convert to mono8 directly.")
                # Fallback for other encodings, might not work well
                cv_image_8bit = self.bridge.imgmsg_to_cv2(msg, desired_encoding="mono8")

            # Construct the filename with a timestamp
            timestamp_sec = msg.header.stamp.sec
            timestamp_nanosec = msg.header.stamp.nanosec
            filename = os.path.join(
                self.save_directory,
                f"ouster_range_image_{timestamp_sec}_{timestamp_nanosec}.png"
            )

            # Save the image as a PNG file
            cv2.imwrite(filename, cv_image_8bit) # Save the 8-bit image
            self.get_logger().info(f'Successfully saved image to: {filename}')
            self.image_saved = True # Set the flag

        except Exception as e:
            self.get_logger().error(f"Error processing or saving image: {e}")
        finally:
            self.get_logger().info("Image processing complete. Shutting down node.")
            rclpy.shutdown() # This will stop the rclpy.spin() in main

def main(args=None):
    rclpy.init(args=args)
    node = OusterRangeImageSaverSingle()
    try:
        rclpy.spin(node) # This will block until rclpy.shutdown() is called
    except KeyboardInterrupt:
        node.get_logger().info('Node interrupted by user before image was saved.')
    finally:
        node.destroy_node()
        if rclpy.ok(): # Only call if not already shutdown
            rclpy.shutdown()

if __name__ == '__main__':
    main()