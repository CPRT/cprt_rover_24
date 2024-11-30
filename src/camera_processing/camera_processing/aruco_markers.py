import rclpy, cv2, numpy
from rclpy.node import Node 
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
from std_msgs.msg import Int32MultiArray, Float32, Float32MultiArray, Int32
from math import sqrt
    
class DetectArucoMarkers(Node):
    def __init__(self):
        super().__init__('detect_aruco_makers')

        self.is_source_image_compressed = True
        self.should_publish_compressed = True
        self._image_scaling = 0.3

        # source_image_topic = '/zed/camera1/image_raw'
        # source_image_topic = '/zed/camera1/image_compressed'
        # source_image_topic = '/zed/source_image'
        source_image_topic = '/published_image'

        processed_image_topic = '/acruco_processed'

        aruco_ids_array = 'aruco_tag_ids_array'
        aruco_id_largest = 'acruco_tag_id_largest'

        aruco_percent_filled_array= 'aruco_percent_filled_array'
        aruco_percent_filled_largest = 'aruco_percent_filled_largest'

        depth_history = 10

        image_width = 640
        image_height = 480
        


        # Subscribe to the get images
        self.source_image_subscriber = self.create_subscription(
            CompressedImage if self.is_source_image_compressed else Image,
            source_image_topic,
            self.process_image, 
            depth_history
        )

        # Publish processed images which outline the tags
        self.processed_image_publisher = self.create_publisher(
            CompressedImage if self.should_publish_compressed else Image,
            processed_image_topic,
            depth_history
        )

        # Publish the acruco marker ids as an array and the largest one
        self.aruco_ids_array_publisher = self.create_publisher(
            Int32MultiArray, 
            aruco_ids_array,
            depth_history
        )
        self.aruco_ids_largest_publisher = self.create_publisher(
            Int32,
            aruco_id_largest,
            depth_history
        )
        
        # Publish the percent each aruco marker takes up in the frame
        self.aruco_percent_filled_array_publisher = self.create_publisher(
            Float32MultiArray,
            aruco_percent_filled_array,
            depth_history
        )
        self.aruco_percent_filled_largest_publisher = self.create_publisher(
            Float32,
            aruco_percent_filled_largest,
            depth_history
        )
        
        self.br = CvBridge()
        self.aruco_dict_4x4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
        self.aruco_params_4x4 =  cv2.aruco.DetectorParameters()
        self.aruco_detector_4x4 = cv2.aruco.ArucoDetector(self.aruco_dict_4x4, self.aruco_params_4x4)

        self.frame_area = image_width * image_height

    def process_image(self, image: Image):
        self.get_logger().info('Got Image to process')
        if self.is_source_image_compressed:
            cv_image = self.br.compressed_imgmsg_to_cv2(image)
        else:
            cv_image = self.br.imgmsg_to_cv2(image)




        self.detect_aruco(cv_image)

        cv_image = cv2.resize(cv_image, None, fx=self._image_scaling, fy=self._image_scaling, interpolation = cv2.INTER_LINEAR)
    
        if self.should_publish_compressed:
            self.processed_image_publisher.publish(self.br.cv2_to_compressed_imgmsg(cv_image)) 
        else:
            self.processed_image_publisher.publish(self.br.cv2_to_imgmsg(cv_image))

    def detect_aruco(self, cv_image):
        corners, ids, rejected = self.aruco_detector_4x4.detectMarkers(cv_image)

        # cv2 code from https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/

        if len(corners) == 0:
            # self.get_logger().info('No aruco markers found')
            return
        
        ids = ids.flatten()
        self.aruco_ids_array_publisher.publish(Int32MultiArray(data=ids))

        largest_percent_filled = 0
        largest_marker_id = -1

        percent_filled_array = []

        # loop over the detected ArUCo corners
        for (markerCorner, markerID) in zip(corners, ids):
            if markerID in (85, 37, 17, 13):
                continue

            # Publish helpful data for navigation
            percent_filled = self.calculate_percent_filled(markerCorner, markerID)
            percent_filled_array += [percent_filled]

            if percent_filled > largest_percent_filled:
                largest_marker_id = markerID
                largest_percent_filled = percent_filled


            # extract the marker corners (which are always returned in
            # top-left, top-right, bottom-right, and bottom-left order)
            corners = markerCorner.reshape((4, 2))
            (topLeft, topRight, bottomRight, bottomLeft) = corners

            # convert each of the (x, y)-coordinate pairs to integers
            topRight = (int(topRight[0]), int(topRight[1]))
            bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
            bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
            topLeft = (int(topLeft[0]), int(topLeft[1]))

            # draw the bounding box of the ArUCo detection
            cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 4)
            cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 4)
            cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 4)
            cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 4)

            # compute and draw the center (x, y)-coordinates of the ArUco
            # marker
            cX = int((topLeft[0] + bottomRight[0]) / 2.0)
            cY = int((topLeft[1] + bottomRight[1]) / 2.0)
            cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)

            # draw the ArUco marker ID on the image
            cv2.putText(cv_image, str(markerID),
                (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
                4, (0, 255, 0), 4)

        # Publish calculated data helpful for navigation
        if largest_marker_id == -1:
            self.getLogger().error("Unknown issue in calculating data from aruco markers")
        else:
            self.aruco_percent_filled_array_publisher.publish(Float32MultiArray(data=percent_filled_array))
            self.aruco_percent_filled_largest_publisher.publish(Float32(data=largest_percent_filled))
            self.aruco_ids_largest_publisher.publish(Int32(data=int(largest_marker_id)))

            self.get_logger().info(f"Largest Aruco marker in frame is {largest_marker_id}, fills {100*largest_percent_filled:.2f}% of the frame.")


    def calculate_percent_filled(self, corners, markerID) -> float:
        try:
            aruco_area = cv2.contourArea(corners)
        except Exception as e:
            self.get_logger().error(f"Error in calculating the area of aruco marker id {markerID}.")
            return 0

        return aruco_area / self.frame_area


def main(args=None):
    rclpy.init(args=args)

    node = DetectArucoMarkers()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()
