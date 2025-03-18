from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String

import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
from interfaces.msg import SixFloats
from interfaces.srv import ArmPos
from interfaces.srv import ArmCmd
from trajectory_msgs.msg import JointTrajectoryPoint
from math import pi

from moveit_msgs.action import ExecuteTrajectory

# import moveit_msgs.action


class typingController(Node):
    def __init__(self):
        super().__init__("trajectoryInt")

        self.get_logger().info("Running node")

        self.traj_sub = self.create_subscription(
            ExecuteTrajectory.FeedbackMessage(),
            "/execute_trajectory/_action/feedback",
            self.traj_callback,
            10,
        )

    def traj_callback(self, msg):
        self.get_logger().info(f"Message: {msg}")


def main(args=None):
    rclpy.init(args=args)
    node = typingController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
