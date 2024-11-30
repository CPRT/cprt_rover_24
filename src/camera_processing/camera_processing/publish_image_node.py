import cv2, os, re, argparse
from time import sleep
from enum import Enum

import rclpy, cv2
from rclpy.node import Node 
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

from .zed_helper_files.video_capture import VideoCapture

from .zed_helper_files.map_targets_between_cams import PitchYaw, Point, CameraUtil, LinearUndistortion

class CameraUtil(Enum):
    ARUCO_CAMERA = CameraUtil("ArucoCamera", 1920.0*2, 1080.0, 110.0, 70.0, \
                                # "usb-Sonix_Technology_Co.__Ltd._USB_2.0_Camera_SN5100-video-index0") # Top cam
                                "usb-Technologies__Inc._ZED_2i_OV0001-video-index0") # ZED 2i

class PublishImage(Node):
    def __init__(self):
        super().__init__('publish_image')

        self.cv_bridge = CvBridge()

        try:
            self.camera_capture = VideoCapture(CameraUtil.ARUCO_CAMERA)
        except Exception as e:
            self.get_logger().error(f"Could not create the camera capture object. Error: {e}")
            raise ValueError(f"Could not create the camera capture object. Error: {e}")

        self.count = 1
        self.image_pub = self.create_publisher(CompressedImage, '/published_image', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def _find_video_index(v4l_byid_name: str):
        if v4l_byid_name is None or len(v4l_byid_name) == 0:
            raise ValueError("Must provide a v4l_byid_name name to find the video index")
        
        v4l_byid_name = "/dev/v4l/by-id/" + v4l_byid_name

        if not os.path.exists(v4l_byid_name):
            raise ValueError("Path doesn't exist when following the v4l_byid_name")
        
        device_path = os.path.realpath(v4l_byid_name)
        device_re = re.compile("\/dev\/video(\d+)")
        info = device_re.match(device_path)
        if not info:
            raise ValueError("Could not find the video index in the file pointed to by v4l_byid_name")
        
        return int(info.group(1))

    def timer_callback(self):
        frame, delta = self.camera_capture.read()
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        frame = cv2.resize(frame, None, fx=0.1, fy=0.1, interpolation = cv2.INTER_LINEAR)
        
        self.get_logger().info("Publishing image " + str(self.count) + ", shape: " + str(frame.shape))
        self.image_pub.publish(self.cv_bridge.cv2_to_compressed_imgmsg(frame))

        self.count += 1

def main(args=None):
    rclpy.init(args=args)

    node = PublishImage()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()
