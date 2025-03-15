import rclpy
from rclpy.node import Node

from interfaces.srv import GetNodeStatus
from interfaces.srv import LaunchNode
from interfaces.srv import ListNodes
from interfaces.srv import ShutdownAllNodes
from interfaces.srv import StopNode

import random

from typing import Callable

import subprocess


def get_stdout(process: list[str]) -> str:
    subprocess.Popen(process).stdout.read()


class NodeManagerTestClient(Node):
    def __init__(self):
        super().__init__("node_manager_test_client")

        self.launcher_client = self.create_client(LaunchNode, "node_manager/launch_node")
        self.stopper_client = self.create_client(StopNode, "node_manager/stop_node")
        self.node_status_client = self.create_client(GetNodeStatus, "node_manager/get_node_status")
        self.node_listing_client = self.create_client(ListNodes, "node_manager/list_nodes")
        self.shutdown_client = self.create_client(ShutdownAllNodes, "node_manager/shutdown_all")
        self.get_logger().info("clients created")

        self.attempted_tests = 0
        self.successful_tests = 0


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
    launch_request = LaunchNode.Request()
    launch_request.package = "node_manager"
    launch_request.executable = "test_node"
    launch_request.name = "test"
    client.get_logger().info("testing launch")
    future = client.launcher_client.call_async(launch_request)
    client.get_logger().info("asdfqwerty")

    while True: pass
    # rclpy.spin(client)
    
    client.destroy_node()
    rclpy.shutdown()
