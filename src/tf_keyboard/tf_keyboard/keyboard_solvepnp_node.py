import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from tf_keyboard.keyboard_detector import KeyboardDetector
from tf_keyboard.image_splitter import ImageSplitter
from tf_keyboard.solve_pnp import SolvePnPURCKeyboard

import cv2
import glob
import numpy as np

class KeyboardSolvePnPNode(Node):
    def __init__(self):
        super().__init__('keyboard_solvepnp_node')

        self.get_logger().info("Loading Model")
        self.keyboard_detector = KeyboardDetector()
        self.get_logger().info("Model Loaded")

        self.image_splitter = ImageSplitter()
        self.solve_pnp = SolvePnPURCKeyboard(score_threshold=0.6)

        self.keyboard_pose_publisher = self.create_publisher(PoseStamped, 'keyboard_pose', 10)

        self.create_timer(0.1, self.timer_callback)

        # self.video_capture = cv2.VideoCapture(3)
        # if not self.video_capture.isOpened():
        #     self.get_logger().error("Could not open video device")
        #     raise RuntimeError("Could not open video device")
        # self.video_capture.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        # self.video_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

        img_dir = "/workspaces/CPRT/solvepnp_keyboard_will_model/ArmCamImagesTwo-200mmAway"
        self.image_filenames = glob.glob(f"{img_dir}/*.png")
        self.image_index = 0

    def timer_callback(self):
        # Read one image from the self.image_filenames list
        if self.image_index >= len(self.image_filenames):
            self.image_index = 0
            self.get_logger().info("Resetting image index to 0")

        image_path = self.image_filenames[self.image_index]
        self.image_index += 1
        frame = cv2.imread(image_path)
        if frame is None:
            self.get_logger().error(f"Failed to read image from {image_path}")
            return
        
        self.get_logger().info(f"Processing image: {image_path}")
        self.simple_process_image(frame)


        # ret, frame = self.video_capture.read()
        # if not ret:
        #     self.get_logger().error("Failed to capture image")
        #     return


    def simple_process_image(self, frame):
        self.image_splitter.reset_base_image(frame)
        process_frame = self.image_splitter.get_image_subsection_centered_on_point(1920//2, 1080//2, 1280, 720)

        # Process the image
        detections = self.keyboard_detector.detect(process_frame)
        if detections is None:
            self.get_logger().error("No detections found")
            return

        # Convert the detection boxes to the base image coordinates
        for idx, box in enumerate(detections['detection_boxes']):
            x1, y1 = box[0], box[1]
            x2, y2 = box[2], box[3]
            x1, y1 = self.image_splitter.convert_split_image_pixels_to_base_image(x1, y1)
            x2, y2 = self.image_splitter.convert_split_image_pixels_to_base_image(x2, y2)
            detections['detection_boxes'][idx] = np.array([x1, y1, x2, y2], dtype=np.float32)

        # Solve PnP
        pose = self.solve_pnp.solvepnp(detections, frame)
        if pose is None:
            self.get_logger().error("Failed to solve PnP")
        else:
            # Publish the pose
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'arm_camera_frame'
            pose_msg.pose.position.x = pose[0]
            pose_msg.pose.position.y = pose[1]
            pose_msg.pose.position.z = pose[2]
            self.keyboard_pose_publisher.publish(pose_msg)

        self.keyboard_detector.draw_detections_onto_image(frame, detections)
        cv2.imshow("Keyboard Detection", frame)
        key = cv2.waitKey(0)
        if key == ord('q'):
            self.get_logger().info("Quitting...")
            cv2.destroyAllWindows()
            return



        

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardSolvePnPNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard SolvePnP Node is shutting down.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()