import rclpy
from rclpy.node import Node

from interfaces.srv import GetNodeStatus
from interfaces.srv import LaunchNode
from interfaces.srv import ListNodes
from interfaces.srv import ShutdownAllNodes
from interfaces.srv import StopNode

import random

from typing import Callable


class NodeManagerTestNode(Node):
    def __init__(self):
        super().__init__("node_manager_test_node")


def main(args=None):
    rclpy.init(args=args)
    client = NodeManagerTestNode()
    rclpy.spin(client)
    client.destroy_node()
    rclpy.shutdown()
