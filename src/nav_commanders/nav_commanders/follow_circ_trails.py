import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from nav2_simple_commander.robot_navigator import BasicNavigator
import yaml
from ament_index_python.packages import get_package_share_directory
import os
import sys
import time
import rclpy.logging
from robot_localization.srv import FromLL
from rclpy.node import Node
from nav2_msgs.action import FollowWaypoints
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geographic_msgs.msg import GeoPose
from std_msgs.msg import Bool, Float32 
from interfaces.srv import NavToGPSGeopose
from interfaces.msg import ArucoMarkers, PointArray
from time import sleep
import math

from .single_trail import SingleCIRCTrail, TrailType, TrailState, TrailGoal, ReturnGoalAndStateAndPose, RecoverySteps, TrailHelperFunctions

class CIRCTrailsCommander(Node):
    """
    Class to do stuff yay. Please forget you read this.
    """

    def __init__(self):
        super().__init__('CIRCTrailsCommander')
        self.navigator = BasicNavigator("CIRCTrailsNavigator_Navigator")

        self.declare_parameters(
            namespace="",
            parameters=[ 
                ('trail', 'error1'),
            ]
        )

        self.sub_rover_pose = self.create_subscription(Odometry, '/odometry/filtered/global', self.rover_pose_callback, 10)
        self.last_rover_pose = PoseStamped()

        self.sub_aruco_markers = self.create_subscription(ArucoMarkers, '/zed/zed_aruco_points_map', self.aruco_marker_callback, 10)
        self.last_aruco_markers = ArucoMarkers()
        self.last_arucos_used = True

        self.sub_blue_led = self.create_subscription(PointArray, '/zed/blue_led_points_map', self.blue_led_callback, 10)
        self.last_blue_led = PointArray()
        self.last_blue_used = True

        self.sub_red_led = self.create_subscription(PointArray, '/zed/red_led_points_map', self.red_led_callback, 10)
        self.last_red_led = PointArray()
        self.last_red_used = True

        self.sub_ir_led = self.create_subscription(PointArray, '/zed/ir_led_points_map', self.ir_led_callback, 10)
        self.last_ir_led = PointArray()
        self.last_ir_used = True

        self.pub_goal = self.create_publisher(PoseStamped, '/circ_trails_goal', 10)
        self.pub_goal_yaw = self.create_publisher(Float32, '/circ_trails_goal_yaw', 10)

        self.pub_lights_on = self.create_publisher(Bool, '/lights', 10)

        self.aruco_tag_found = -1
        self.trail_name = str(self.get_parameter('trail').value)
        if self.trail_name == '':
            self.get_logger().error("No trail name provided")
            raise Exception("No trail name provided")
        
        elif self.trail_name == 'blue':
            self.trail = SingleCIRCTrail(TrailType.BLUE_TRAIL, False, self.get_logger())
        elif self.trail_name == 'red':
            self.trail = SingleCIRCTrail(TrailType.RED_TRAIL, False, self.get_logger())
        elif self.trail_name == 'ir':
            self.trail = SingleCIRCTrail(TrailType.IR_TRAIL, True, self.get_logger())
        else:
            self.get_logger().error("Invalid trail name provided (" + self.trail_name + "). Must be blue, red or ir")
            raise Exception("Invalid trail name provided. Must be blue, red or ir")

        self.get_logger().info(f'Created {self.trail_name} trail')

        self.get_logger().info('Waiting for Nav2 to be active')
        self.navigator.waitUntilNav2Active()
        self.get_logger().info('Nav2 is active')

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.main_timer_callback)
        self.get_logger().info("Starting the trail named " + self.trail_name)

    def control_lights(self, is_lights_on: bool):
        msg = Bool()
        msg.data = is_lights_on
        self.pub_lights_on.publish(msg)

    def rover_pose_callback(self, msg: Odometry):
        self.last_rover_pose = PoseStamped()
        self.last_rover_pose.pose = msg.pose.pose
        self.last_rover_pose.header = msg.header

    def aruco_marker_callback(self, msg: ArucoMarkers):
        self.last_aruco_markers = msg
        self.last_arucos_used = False

    def blue_led_callback(self, msg: PointArray):
        self.last_blue_led = msg
        self.last_blue_used = False

    def red_led_callback(self, msg: PointArray):
        self.last_red_led = msg
        self.last_red_used = False

    def ir_led_callback(self, msg: PointArray):
        self.last_ir_led = msg
        self.last_ir_used = False

    def main_timer_callback(self):
        if self.trail._trail == TrailType.BLUE_TRAIL:
            if not self.last_blue_used:
                self.last_blue_used = True
                led_points = self.last_blue_led
            else:
                return

        elif self.trail._trail == TrailType.RED_TRAIL:
            if not self.last_red_used:
                self.last_red_used = True
                led_points = self.last_red_led
            else:
                return

        elif self.trail._trail == TrailType.IR_TRAIL:
            if not self.last_ir_used:
                self.last_ir_used = True
                led_points = self.last_ir_led
            else:
                return

        else:
            return

        trail_success = self.run_trail(self.trail, led_points)
        if trail_success:
            raise SystemExit("Successfully completed the " + self.trail_name + " trail")

    def run_trail(self, trail: SingleCIRCTrail, led_points: PointArray) -> bool:
        data = trail.run_led_trail(led_points, self.last_rover_pose, self.get_clock().now())
        if data.recovery_step == RecoverySteps.FAILED_RECOVERY:
            raise Exception("Failed to recover from lack of trail data")
            
        # Run recovery
        if data.trail_goal_info in (TrailGoal.RECOVERY_GOAL, TrailGoal.NO_GOAL):
            data = trail.run_recovery(self.last_rover_pose, self.get_clock().now())

        self.control_lights(data.lights_on)
        self.pub_goal.publish(data.goal_pose)
        self.pub_goal_yaw.publish(Float32(data=math.degrees(TrailHelperFunctions.yaw_from_quaternion(data.goal_pose.pose.orientation))))
        self.get_logger().info(f"goal_state: {data.trail_goal_info.value}, trail_state: {data.trail_state_info.value}, recovery_step: {data.recovery_step.value} , lights_on: {data.lights_on}")

        # Scan for aruco markers
        if data.trail_state_info == TrailState.REQUEST_ARUCO_SCAN:
            found_aruco = trail.run_aruco_finder(self.last_aruco_markers, [], self.get_clock().now())
            if found_aruco != -1:
                self.aruco_tag_found = found_aruco
                self.get_logger().info(f"~~~~\n~~~~~~~~~~~~~              Found aruco marker!! Number is {found_aruco}!!! \n")
                return True
            
        # Stand still if trying to bake a target
        if data.trail_state_info == TrailState.BAKING_TARGET:
            self.get_logger().info("Baking target!!!!!!!!!!")
            # self.navigator.cancelTask()
            return False
        
        # Set the desired goal pose
        self.navigator.goToPose(data.goal_pose)
        return False

def main():
    rclpy.init()

    node = CIRCTrailsCommander()

    try:
        rclpy.spin(node)
    except SystemExit as e:
        rclpy.logging.get_logger("Quitting").info(str(e))

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
