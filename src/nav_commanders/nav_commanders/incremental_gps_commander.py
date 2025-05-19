import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
from robot_localization.srv import FromLL
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPose
from interfaces.srv import NavToGPSGeopose
from rclpy.qos import qos_profile_sensor_data
import math


class IncrementalGpsCommander(Node):
    """
    Class to incrementally navigate to a GPS goal.
    """

    def __init__(self):
        super().__init__("incremental_gps_commander")
        self.navigator = BasicNavigator("incremental_gps_navigator")

        self.declare_parameter(
            "incremental_distance", 50.0
        )  # Distance in meters to the intermediate goal
        self.incremental_distance = (
            self.get_parameter("incremental_distance")
            .get_parameter_value()
            .double_value
        )
        self.get_logger().info(
            f"Incremental distance set to: {self.incremental_distance} meters"
        )

        self.declare_parameter("frequency", 0.1)  # Frequency in Hz
        self.frequency = (
            self.get_parameter("frequency").get_parameter_value().double_value
        )
        self.get_logger().info(
            f"Frequency set to: {self.frequency} Hz or {1.0 / self.frequency} seconds"
        )

        self.declare_parameter("robot_pose_topic", "odometry/filtered/global")
        self.robot_pose_topic = (
            self.get_parameter("robot_pose_topic").get_parameter_value().string_value
        )
        self.get_logger().info(f"Robot pose topic set to: {self.robot_pose_topic}")

        self.nav_fix_topic = "fromLL"
        self.geopose_service_name = "commander/nav_to_gps_incremental"
        self.intermediate_goal_topic = "intermediate_goal"

        self.localizer_callback_group = MutuallyExclusiveCallbackGroup()
        self.pose_callback_group = ReentrantCallbackGroup()
        self.goal_callback_group = MutuallyExclusiveCallbackGroup()

        self.localizer_client = self.create_client(
            FromLL, "fromLL", callback_group=self.localizer_callback_group
        )
        self.geopose_service = self.create_service(
            NavToGPSGeopose,
            self.geopose_service_name,
            self.geopose_server,
            callback_group=self.goal_callback_group,
        )
        self.robot_pose_subscription = self.create_subscription(
            PoseStamped,
            self.robot_pose_topic,
            self.robot_pose_callback,
            qos_profile_sensor_data,
            callback_group=self.pose_callback_group,
        )
        self.intermediate_goal_publisher = self.create_publisher(
            PoseStamped, self.intermediate_goal_topic, 10
        )

        self.final_lat_lon = None
        self.current_robot_pose = None
        self.goal_handle = None  # To store the navigation goal handle

        self.timer = self.create_timer(1.0 / self.frequency, self.timer_callback)

        self.get_logger().info("Nav fix service name: " + str("fromLL"))
        self.get_logger().info("Waiting for nav_sat to be active")
        while not self.localizer_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(
                f"Service on {self.nav_fix_topic} is not available, waiting again..."
            )
        self.get_logger().info("nav_sat is active")

        self.get_logger().info("Waiting for Nav2 to be active")
        self.navigator.waitUntilNav2Active(localizer="controller_server")
        self.get_logger().info("Nav2 is active")

    def robot_pose_callback(self, msg: PoseStamped):
        """Callback to update the current robot pose."""
        self.current_robot_pose = msg

    def geopose_server(
        self, msg: NavToGPSGeopose, response: NavToGPSGeopose
    ) -> NavToGPSGeopose:
        """Service server to receive the initial GPS goal."""
        self.get_logger().info("Received a new GPS goal:")
        self.final_lat_lon = (
            msg.goal.position.latitude,
            msg.goal.position.longitude,
            msg.goal.position.altitude,
            msg.goal.orientation,
        )
        self.goal_handle = None  # Reset goal handle for a new goal
        response.success = True  # Acknowledge receipt of the goal
        return response

    def convert_lat_lon_to_pose(self, latitude, longitude, altitude, orientation):
        """Converts a latitude, longitude, altitude, and orientation to a PoseStamped in the map frame."""
        req = FromLL.Request()
        req.ll_point.longitude = longitude
        req.ll_point.latitude = latitude
        req.ll_point.altitude = altitude

        self.get_logger().info(
            f"Requesting map pose for: Long={req.ll_point.longitude:.8f}, Lat={req.ll_point.latitude:.8f}, Alt={req.ll_point.altitude:.8f}"
        )

        try:
            future = self.localizer_client.call_async(req)
            rclpy.spin_until_future_complete(self, future)
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
                return target_pose
            else:
                self.get_logger().error("Failed to convert lat/lon to map pose")
                return None
        except Exception as e:
            self.get_logger().error(f"Error during lat/lon conversion: {e}")
            return None

    def euclidean_distance(self, pose1: PoseStamped, pose2: PoseStamped):
        """Calculates the Euclidean distance between two poses (ignoring z)."""
        if pose1 is None or pose2 is None:
            return float("inf")
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    def calculate_intermediate_goal(
        self, current_pose: PoseStamped, final_pose: PoseStamped, distance: float
    ):
        """Calculates an intermediate goal point a certain distance along the path to the final goal."""
        if current_pose is None or final_pose is None:
            return None

        dx = final_pose.pose.position.x - current_pose.pose.position.x
        dy = final_pose.pose.position.y - current_pose.pose.position.y
        total_distance = math.sqrt(dx * dx + dy * dy)

        if total_distance <= distance:
            return final_pose  # Already within the desired distance

        ratio = distance / total_distance
        intermediate_x = current_pose.pose.position.x + dx * ratio
        intermediate_y = current_pose.pose.position.y + dy * ratio

        intermediate_goal = PoseStamped()
        intermediate_goal.header.frame_id = "map"
        intermediate_goal.header.stamp = self.get_clock().now().to_msg()
        intermediate_goal.pose.position.x = intermediate_x
        intermediate_goal.pose.position.y = intermediate_y
        intermediate_goal.pose.orientation = (
            final_pose.pose.orientation
        )  # Keep the final orientation
        return intermediate_goal

    def timer_callback(self):
        """Timer routine to calculate and publish intermediate goals and handle final goal completion."""
        if self.final_lat_lon is not None and self.current_robot_pose is not None:
            final_pose = self.convert_lat_lon_to_pose(*self.final_lat_lon)
            if final_pose:
                distance_to_final = self.euclidean_distance(
                    self.current_robot_pose, final_pose
                )
                if distance_to_final <= self.incremental_distance:
                    self.get_logger().info(
                        f"Within {self.incremental_distance:.2f} meters of the final goal. Sending final goal."
                    )
                    self.intermediate_goal_publisher.publish(final_pose)
                    if self.goal_handle is None or not self.navigator.isTaskComplete(
                        self.goal_handle
                    ):
                        self.goal_handle = self.navigator.goToPose(final_pose)
                        self.get_logger().info("Navigating to final goal...")
                    elif self.goal_handle is not None and self.navigator.isTaskComplete(
                        self.goal_handle
                    ):
                        result = self.navigator.getResult(self.goal_handle)
                        if result == TaskResult.SUCCEEDED:
                            self.get_logger().info("Final goal succeeded!")
                        elif result == TaskResult.CANCELED:
                            self.get_logger().info("Final goal was canceled!")
                        elif result == TaskResult.FAILED:
                            self.get_logger().error("Final goal failed!")
                        self.final_lat_lon = None  # Stop sending goals
                        self.goal_handle = None
                else:
                    intermediate_goal = self.calculate_intermediate_goal(
                        self.current_robot_pose, final_pose, self.incremental_distance
                    )
                    if intermediate_goal:
                        self.get_logger().info(
                            f"Publishing intermediate goal: x={intermediate_goal.pose.position.x:.2f}, y={intermediate_goal.pose.position.y:.2f}"
                        )
                        self.intermediate_goal_publisher.publish(intermediate_goal)
                    else:
                        self.get_logger().warn("Could not calculate intermediate goal.")
            else:
                self.get_logger().warn("Could not convert final lat/lon to pose.")
        elif self.final_lat_lon is not None and self.current_robot_pose is None:
            self.get_logger().warn("Waiting for the robot's current pose...")
        elif (
            self.final_lat_lon is None
            and self.goal_handle is not None
            and self.navigator.isTaskComplete(self.goal_handle)
        ):
            # Ensure we log the final result even if the timer ticks after completion
            result = self.navigator.getResult(self.goal_handle)
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("Final goal succeeded!")
            elif result == TaskResult.CANCELED:
                self.get_logger().info("Final goal was canceled!")
            elif result == TaskResult.FAILED:
                self.get_logger().error("Final goal failed!")
            self.goal_handle = None


def main(args=None):
    rclpy.init(args=args)
    node = IncrementalGpsCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
