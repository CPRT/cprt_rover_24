import rclpy
from time import sleep
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

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
    output = subprocess.Popen(process, stdout=subprocess.PIPE).stdout
    if output is None:
        return ""
    return output.read().decode("utf-8").replace('\n', '')

def is_in_list(name: str, timeout: float = 0) -> bool:
    start = rclpy.clock.Clock().now()
    while True:
        ret = get_stdout(["ros2", "node", "list"])
        if name in ret:
            return True
        if rclpy.clock.Clock().now() - start > rclpy.time.Duration(seconds=timeout):
            return False
        sleep(0.1)


class NodeManagerTestClient(Node):
    def __init__(self):
        super().__init__("node_manager_test_client")
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()

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

        self.timer = self.create_timer(1.0, self.test_manager, callback_group=self.timer_callback_group)

    def test_manager(self):
        self.get_logger().info("running tests")
        self.assert_okay(self.test_launch, "launch")
        self.assert_okay(self.test_list, "list")
        self.assert_okay(self.test_status, "status")
        self.assert_okay(self.test_stop, "stop")
        self.assert_okay(self.test_launch, "launch")
        self.assert_okay(self.test_shutdown, "shutdown")
        self.get_logger().info(f"tests complete: {self.successful_tests}/{self.attempted_tests}")

    def test_launch(self):
        self.timer.destroy()
        launch_request = LaunchNode.Request()
        launch_request.package = "node_manager"
        launch_request.executable = "test_node"
        launch_request.name = "test"
        assert not is_in_list("node_manager_test_node", timeout=0.5)
        future = self.launcher_client.call_async(launch_request)
        rclpy.spin_until_future_complete(self, future)
        assert is_in_list("node_manager_test_node", timeout=0.5)
    
    def test_stop(self):
        stop_request = StopNode.Request()
        stop_request.name = "test"
        assert is_in_list("node_manager_test_node")
        future = self.stopper_client.call_async(stop_request)
        rclpy.spin_until_future_complete(self, future)
        assert not is_in_list("node_manager_test_node", timeout=0.5)
    
    def test_list(self):
        list_request = ListNodes.Request()
        future = self.node_listing_client.call_async(list_request)
        rclpy.spin_until_future_complete(self, future)
        assert future.result() is not None
    
    def test_status(self):
        status_request = GetNodeStatus.Request()
        status_request.name = "test"
        future = self.node_status_client.call_async(status_request)
        rclpy.spin_until_future_complete(self, future)
        assert future.result() is not None

    def test_shutdown(self):
        shutdown_request = ShutdownAllNodes.Request()
        future = self.shutdown_client.call_async(shutdown_request)
        rclpy.spin_until_future_complete(self, future)
        assert is_in_list("node_manager", timeout=0.5)
        assert not is_in_list("node_manager_test_node", timeout=0.5)

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
    executor = MultiThreadedExecutor()
    executor.add_node(client)
    executor.spin()
    client.destroy_node()
    rclpy.shutdown()
