import rclpy, cv2
from rclpy.node import Node 
from rclpy import Parameter
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Int32

class DisplayImageLocally(Node):
    def __init__(self):
        super().__init__('display_image_locally')

        self.declare_parameters(
            namespace="",
            parameters=[
                ('window_name', 'DisplayImage'),
                ('image_topic', '/zed/cv_zed_image'),
                ('control_svo_index_topic', '/zed/control_svo_index'),
                ('current_svo_index_topic', '/zed/current_svo_index'),
                ('max_svo_index', '/zed/max_svo_index'),
                ('is_image_compressed', True),
                ('depth_history', 10),
                ('encoding', 'passthrough'),
                ('is_depth_image', False)
            ]
        )

        self.is_image_compressed = bool(self.get_parameter('is_image_compressed').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.image_encoding = str(self.get_parameter('encoding').value)
        self.is_depth_image = bool(self.get_parameter('is_depth_image').value)
        self.control_svo_index_topic = str(self.get_parameter('control_svo_index_topic').value)
        self.current_svo_index_topic = str(self.get_parameter('current_svo_index_topic').value)
        self.max_svo_index = str(self.get_parameter('max_svo_index').value)

        self.image_subscriber = self.create_subscription(
            CompressedImage if self.is_image_compressed else Image,
            str(self.get_parameter('image_topic').value),
            self.image_callback, 
            int(self.get_parameter('depth_history').value)
        )

        self.publish_control_svo_index_topic = self.create_publisher(
            Int32,
            self.control_svo_index_topic,
            10
        )

        self.sub_current_svo_index_topic = self.create_subscription(
            Int32,
            self.current_svo_index_topic,
            self.current_svo_index_callback,
            10
        )

        self.sub_max_svo_index = self.create_subscription(
            Int32,
            self.max_svo_index,
            self.set_max_svo_index,
            10
        )

        self.cv_bridge = CvBridge()
        self.image_count = 0
        self.svo_index = 0
        self.control_svo_index = 0
        self.prev_control_svo_index = 0
        self.max_svo_index = 30000
        self.time = self.get_clock().now()
        self.create_display()

        self.get_logger().info(f"Subscribed to {self.get_parameter('image_topic').value}")
        self.get_logger().info(f"Expecting {'CompressedImage' if self.is_image_compressed else 'Image'} message")
        self.get_logger().info(f"Window name: {self.window_name}")
        self.get_logger().info(f"Encoding: {self.image_encoding}")
        self.get_logger().info(f"Depth image: {self.is_depth_image}")

    def create_display(self):
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.createTrackbar('Control SVO Index', self.window_name, self.control_svo_index, self.max_svo_index, self.nothing)
        cv2.createTrackbar('Current SVO Index', self.window_name, self.svo_index, self.max_svo_index, self.nothing)

    def set_max_svo_index(self, msg):
        if self.max_svo_index != msg.data:
            self.max_svo_index = int(msg.data)
            cv2.setTrackbarMax('Control SVO Index', self.window_name, self.max_svo_index)
            cv2.setTrackbarMax('Current SVO Index', self.window_name, self.max_svo_index)

    def current_svo_index_callback(self, msg):
        self.svo_index = int(msg.data)
        cv2.setTrackbarPos('Current SVO Index', self.window_name, self.svo_index)


    def image_callback(self, image):
        # Check the control SVO index trackbar
        control_svo_index = cv2.getTrackbarPos('Control SVO Index', self.window_name)
        if control_svo_index != self.control_svo_index:
            if control_svo_index != self.prev_control_svo_index:
                self.prev_control_svo_index = control_svo_index
                self.time = self.get_clock().now()
            elif ((self.get_clock().now() - self.time).nanoseconds / 1000000000) > 2:
                self.control_svo_index = control_svo_index
                msg = Int32()
                msg.data = self.control_svo_index
                self.publish_control_svo_index_topic.publish(msg)
                self.get_logger().info(f"Control SVO Index: {self.control_svo_index}")
                self.time = self.get_clock().now()
        else:
            self.time = self.get_clock().now()

        # Display the image
        if self.is_image_compressed:
            img = self.cv_bridge.compressed_imgmsg_to_cv2(image, self.image_encoding)
        else:
            img = self.cv_bridge.imgmsg_to_cv2(image, self.image_encoding)

        if self.is_depth_image:
            img = cv2.equalizeHist(img)

        self.get_logger().info(f"Shape before: {img.shape}")
        img = cv2.resize(img, (1920*2, 1080))
        self.get_logger().info(f"Shape after: {img.shape}")
        
        cv2.imshow(self.window_name, img)
        

        self.image_count += 1
        self.get_logger().info(f"Image count: {self.image_count}")
        cv2.waitKey(500)

    def nothing(self, x):
        pass

def main(args=None):
    rclpy.init(args=args)

    node = DisplayImageLocally()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()