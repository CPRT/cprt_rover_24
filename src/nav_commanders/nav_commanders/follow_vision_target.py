from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from interfaces.msg import ArucoMarkers

"""
Follow a vision target published by to a topic in the map frame
"""

class FollowVisionTarget(BasicNavigator):
    def __init__(self):
        super().__init__()
        self.init_basic_navigator()

        # Follow aruco tag
        self.specific_tag_index = 15
        self.aruco_tag_topic = "/zed/zed_aruco_points_map"
        self.aruco_tag_subscriber = self.create_subscription(ArucoMarkers, self.aruco_tag_topic, self.aruco_tag_callback, 10)

        # self.dynamic_nav_pose_subscriber = self.create_subscription(PoseStamped, 'dynamic_nav_pose', self.vision_target_callback, 10)
        self.process_timer = self.create_timer(0.1, self.process)

        self.task_complete = False

    def init_basic_navigator(self):
        self.basic_nav = BasicNavigator()

        # NOT SURE HOW setInitialPose WILL REACT IN CPRT SYSTEM
        # Set our demo's initial pose
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.basic_nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = 0.0
        initial_pose.pose.position.y = 0.0
        initial_pose.pose.orientation.w = 1.0
        self.basic_nav.setInitialPose(initial_pose)

        # Activate navigation, if not autostarted. This should be called after setInitialPose()
        # or this will initialize at the origin of the map and update the costmap with bogus readings.
        # If autostart, you should `waitUntilNav2Active()` instead.
        # self.basic_nav.lifecycleStartup()

        # Wait for navigation to fully activate, since autostarting nav2
        self.basic_nav.waitUntilNav2Active()

        # You may use the navigator to clear the global and local costmaps
        # navigator.clearAllCostmaps()

    def cleanup(self):
        pass
        # self.basic_nav.lifecycleShutdown()

    # def vision_target_callback(self, msg):
    #     self.goToPose(msg)  

    def aruco_tag_callback(self, msg):
        for index, point in msg.points:
            if marker_ids[index] == self.specific_tag_index:
                pose = PoseStamped()
                pose.header = msg.header
                pose.position = point
                self.goToPose(pose)
                return

    def process(self):
        feedback = self.basic_nav.getFeedback()
        if feedback:
            self.get_logger().info('Estimated time of arrival: ' + '{0:.0f}'.format(
                  Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9)
                  + ' seconds.')
        
        if not self.basic_nav.isTaskComplete():
            self.task_complete = False

        elif not self.task_complete:
            self.task_complete = True

            result = self.basic_nav.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Goal succeeded!')
            elif result == TaskResult.CANCELED:
                self.get_logger().info('Goal was canceled!')
            elif result == TaskResult.FAILED:
                self.get_logger().error('Goal failed!')
            else:
                self.get_logger().error('Goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)

    node = FollowVisionTarget()

    try:
        rclpy.spin(node) # spin will block until the node is shutdown
    finally:
        node.cleanup()

    node.destroy_node()
    rclpy.shutdown()

  
if __name__ == '__main__':
  main()