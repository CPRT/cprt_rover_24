import rclpy, cv2
from rclpy.node import Node 

import tf2_geometry_msgs

from tf2_ros import TransformException, LookupException, ConnectivityException, ExtrapolationException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from std_msgs.msg import Header
from geometry_msgs.msg import Point, PointStamped
from interfaces.msg import PointArray, ArucoMarkers
from visualization_msgs.msg import MarkerArray, Marker

from typing import List, Union

class TransformZedPointsToMap(Node):
    def __init__(self):
        super().__init__('transform_zed_points_to_map')

        self.declare_parameters(
            namespace="",
            parameters=[
                ('expected_input_frame', 'zed_left_frame_link'),
                ('desired_frame', 'map'),
            ]
        )

        self.expected_input_frame = str(self.get_parameter('expected_input_frame').value)
        self.desired_frame = str(self.get_parameter('desired_frame').value)

        self.subscribe_zed_aruco_points = self.create_subscription(ArucoMarkers, '/zed/zed_aruco_points', self.zed_aruco_callback, 10)
        self.subscribe_blue_led_points = self.create_subscription(PointArray, '/zed/blue_led_points', self.blue_led_callback, 10)
        self.subscribe_red_led_points = self.create_subscription(PointArray, '/zed/red_led_points', self.red_led_callback, 10)
        self.subscribe_ir_led_points = self.create_subscription(PointArray, '/zed/ir_led_points', self.ir_led_callback, 10)

        self.publish_zed_aruco_points = self.create_publisher(ArucoMarkers, '/zed/zed_aruco_points_map', 10)
        self.publish_blue_led_points = self.create_publisher(PointArray, '/zed/blue_led_points_map', 10)
        self.publish_red_led_points = self.create_publisher(PointArray, '/zed/red_led_points_map', 10)
        self.publish_ir_led_points = self.create_publisher(PointArray, '/zed/ir_led_points_map', 10)

        self.publish_aruco_rviz_marker = self.create_publisher(MarkerArray, '/zed/rviz_aruco_markers', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.transform = None

    def update_transform(self) -> bool:
        try:
            self.transform = self.tf_buffer.lookup_transform(
                self.desired_frame,
                self.expected_input_frame,
                rclpy.time.Time())
            return True
        except (
            TransformException,
            LookupException,
            ConnectivityException,
            ExtrapolationException,
        ) as e:
            self.get_logger().warn(
                f"Could not retrieve transformation from {self.expected_input_frame} to {self.desired_frame}. Exception: {e}.")
            return False

    def transform_point_array(self, header: Header, points: List[Point]) -> List[Point]:
        mapped_points: List[Point] = []
        for point in points:
            point_stamped = PointStamped()
            point_stamped.point = point
            point_stamped.header = header
            mapped_point = tf2_geometry_msgs.do_transform_point(point_stamped, self.transform)
            mapped_points.append(mapped_point.point)

        return mapped_points

    def handle_any_callback(self, msg: Union[ArucoMarkers, PointArray]) -> Union[ArucoMarkers, PointArray, None]:
        if msg.header.frame_id != self.expected_input_frame:
            self.get_logger().error(f"Recieved a message with the wrong input frame. " + 
                    f"Expected {self.expected_input_frame} but got {msg.header.frame_id}")
            return None
            
        if not self.update_transform():
            return None # Failed to get transform

        initial_length = len(msg.points)

        if isinstance(msg, ArucoMarkers):
            new_msg = ArucoMarkers()
            new_msg.marker_ids = msg.marker_ids
            new_msg.is_moving = msg.is_moving
        elif isinstance(msg, PointArray):
            new_msg = PointArray()
            new_msg.is_moving = msg.is_moving
        else:
            self.get_logger().error(f"Recieved a message that is not either ArucoMarkers or PointArray")
            return None

        new_msg.points = self.transform_point_array(msg.header, msg.points)
        new_msg.header = msg.header
        new_msg.header.frame_id = self.desired_frame

        if initial_length != len(new_msg.points):
            self.get_logger().error(f"Lost points from array after transforming it")
            return None

        return new_msg

    def zed_aruco_callback(self, msg: ArucoMarkers):
        new_msg = self.handle_any_callback(msg)
        if new_msg is None:
            return
        # elif not isinstance(new_msg, ArucoMarkers):
        #     self.get_logger().error(f"Function handle_any_callback in transform_zed_points_to_map.py returned incorrect type for zed_aruco_callback")
        else:
            self.publish_zed_aruco_points.publish(new_msg)
            self.publish_aruco_rviz_marker.publish(self.get_rviz_markers(new_msg.points, new_msg.header.stamp))

    def blue_led_callback(self, msg: PointArray):
        new_msg = self.handle_any_callback(msg)
        if new_msg is None:
            return
        
        # elif not isinstance(new_msg, PointArray):
        #     self.get_logger().error(f"Function handle_any_callback in transform_zed_points_to_map.py returned incorrect type for blue_led_callback")
        else:
            self.publish_blue_led_points.publish(new_msg)

    def red_led_callback(self, msg: PointArray):
        new_msg = self.handle_any_callback(msg)
        if new_msg is None:
            return
        # elif not isinstance(new_msg, PointArray):
        #     self.get_logger().error(f"Function handle_any_callback in transform_zed_points_to_map.py returned incorrect type for red_led_callback")
        else:
            self.publish_red_led_points.publish(new_msg)

    def ir_led_callback(self, msg: PointArray):
        new_msg = self.handle_any_callback(msg)
        if new_msg is None:
            return
        # elif not isinstance(new_msg, PointArray):
        #     self.get_logger().error(f"Function handle_any_callback in transform_zed_points_to_map.py returned incorrect type for ir_led_callback")
        else:
            self.publish_ir_led_points.publish(new_msg)

    def get_rviz_markers(self, point_list: List[Point], header_timestamp) -> MarkerArray:
        markerArray = MarkerArray()
        for point_index, point in enumerate(point_list):
            marker = Marker()
            marker.id = point_index
            marker.header.stamp = header_timestamp
            marker.header.frame_id = "map"
            marker.type = marker.CUBE
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = point.x
            marker.pose.position.y = point.y
            marker.pose.position.z = point.y
            marker.lifetime = rclpy.time.Duration(seconds=0).to_msg()
            marker.frame_locked = False
            markerArray.markers.append(marker)

        for i in range(len(point_list), 12):
            marker = Marker()
            marker.id = i
            marker.header.stamp = header_timestamp
            marker.header.frame_id = "map"
            marker.action = 2
            marker.lifetime = rclpy.time.Duration(seconds=0).to_msg()
            marker.frame_locked = False
            markerArray.markers.append(marker)

        return markerArray
def main(args=None):
    rclpy.init(args=args)

    node = TransformZedPointsToMap()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()