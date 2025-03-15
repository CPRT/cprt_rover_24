import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from interfaces.srv import GetNodeStatus
from interfaces.srv import LaunchNode
from interfaces.srv import ListNodes
from interfaces.srv import ShutdownAllNodes
from interfaces.srv import StopNode

import random

from typing import Callable

import subprocess

import rclpy.timer


def get_stdout(process: list[str]) -> str:
    subprocess.Popen(process).stdout.read()


class NodeManagerTestClient(Node):
    def __init__(self):
        super().__init__("node_manager_test_client")

        self.launcher_client = self.create_client(
            LaunchNode, "node_manager/launch_node"
        )
        while not self.launcher_client.wait_for_service(1.0):
            self.get_logger().info("launch service not available, reattempting")
        self.stopper_client = self.create_client(StopNode, "node_manager/stop_node")
        self.node_status_client = self.create_client(
            GetNodeStatus, "node_manager/get_node_status"
        )
        self.node_listing_client = self.create_client(
            ListNodes, "node_manager/list_nodes"
        )
        self.shutdown_client = self.create_client(
            ShutdownAllNodes, "node_manager/shutdown_all"
        )
        self.get_logger().info("clients created")

        self.attempted_tests = 0
        self.successful_tests = 0

        self.timer = self.create_timer(1.0, self.test_manager)
        self.launch_request = LaunchNode.Request()

    def test_manager(self):
        self.timer.destroy()

        self.launch_request = LaunchNode.Request()
        self.launch_request.package = "node_manager"
        self.launch_request.executable = "test_node"
        self.launch_request.name = "test"
        self.get_logger().info("testing launch")
        # TODO: this future always pends and never actually completes
        self.future = self.launcher_client.call_async(self.launch_request)
        rclpy.spin_until_future_complete(self, self.future)
        self.get_logger().info("asdfqwerty")

    def assert_okay(self, function: Callable[[], None], test_name: str):
        self.get_logger().info(f"running test `{test_name}`")
        try:
            self.attempted_tests += 1
            function()
        except:
            self.get_logger().error(f"test `{test_name}` failed")
        else:
            self.successful_tests += 1
            self.get_logger().info(f"test `{test_name}` passed")

    def assert_throws(self, function: Callable[[], None], test_name: str):
        self.get_logger().info(f"running throwing test `{test_name}`")
        try:
            self.attempted_tests += 1
            function()
        except:
            self.successful_tests += 1
            self.get_logger().info(f"throwing test `{test_name}` passed")
        else:
            self.get_logger().error(f"throwing test `{test_name}` failed")


def main(args=None):
    rclpy.init(args=args)
    client = NodeManagerTestClient()
    executor = MultiThreadedExecutor(8)
    executor.add_node(client)
    executor.spin()
    client.destroy_node()
    rclpy.shutdown()
