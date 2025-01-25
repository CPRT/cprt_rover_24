#!/usr/bin/env python
import threading
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy.time
from ros_phoenix.msg import MotorControl, MotorStatus

# Defines:
LEFT = 0
RIGHT = 1


class WheelControl:
    """
    Class to control individual wheels of the robot.
    """

    def __init__(self, wheel_name, node):
        """
        Initialize the WheelControl object.

        :param wheel_name: Name of the wheel (must include "left" or "right").
        :param node: The ROS2 node instance.
        """
        self.name = wheel_name
        if "Left" in wheel_name or "left" in wheel_name:
            self.side = LEFT
        elif "Right" in wheel_name or "right" in wheel_name:
            self.side = RIGHT
        else:
            raise RuntimeError('Wheel names must include "left" or "right"')

        self.pub = node.create_publisher(MotorControl, f"/{wheel_name}/set", 10)
        self.sub = node.create_subscription(
            MotorStatus, f"/{wheel_name}/status", self.wheel_status_callback, 10
        )
        self.status = MotorStatus()
        self._control = MotorControl()
        # TODO (after PID tuning): Switch to Vel control
        # self._control.mode = MotorControl.VELOCITY
        self._control.mode = MotorControl.PERCENT_OUTPUT
        self.lock = threading.Lock()

    def wheel_status_callback(self, msg: MotorStatus):
        """
        Callback function to update the wheel status.

        :param msg: MotorStatus message.
        """
        self.status = msg

    def set_vel(self, value):
        """
        Set the velocity of the wheel.

        :param value: Velocity value to set.
        """
        with self.lock:
            self._control.value = value

    def send(self):
        """
        Publish the control command to the wheel.
        """
        with self.lock:
            self.pub.publish(self._control)


class TalonDriveController(Node):
    """
    ROS2 Node to control the Talon drive system.
    """

    def __init__(self):
        """
        Initialize the TalonDriveController node.
        """
        super().__init__("talonDrive")
        self.last_timestamp = 0

        self.declare_parameter("max_speed", 2.0)
        self.MAX_SPEED = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        self.declare_parameter("base_width", 0.9144)
        self.BASE_WIDTH = (
            self.get_parameter("base_width").get_parameter_value().double_value
        )
        self.declare_parameter("pub_odom", True)
        PUB_ODOM = self.get_parameter("pub_odom").get_parameter_value().bool_value
        self.declare_parameter("pub_elec", True)
        PUB_ELEC = self.get_parameter("pub_elec").get_parameter_value().bool_value
        self.declare_parameter("frequency", 10)
        FREQ = self.get_parameter("frequency").get_parameter_value().integer_value
        FREQ = max(1, FREQ)

        self.declare_parameter("robot_frame", "base_link")
        self.FRAME = (
            self.get_parameter("robot_frame").get_parameter_value().string_value
        )
        self.declare_parameter("angular_covariance", 0.3)
        angular_cov = (
            self.get_parameter("angular_covariance").get_parameter_value().double_value
        )
        self.declare_parameter("linear_covariance", 0.3)
        linear_cov = (
            self.get_parameter("linear_covariance").get_parameter_value().double_value
        )
        self.covariance = [0.0] * 36
        self.covariance[0] = linear_cov
        self.covariance[35] = angular_cov

        self.declare_parameter("timeout", 2.0)
        self.TIMEOUT = (
            self.get_parameter("timeout").get_parameter_value().double_value * 1e9
        )

        self.declare_parameter(
            "wheels", ["frontRight", "frontLeft", "backRight", "backLeft"]
        )
        wheel_names = (
            self.get_parameter("wheels").get_parameter_value().string_array_value
        )

        self.wheels = []
        for wheel in wheel_names:
            self.wheels.append(WheelControl(wheel, self))
        self.get_logger().info(f"There are {str(len(self.wheels))} wheels")

        self.twist_sub = self.create_subscription(
            Twist, "/cmd_vel", self.twist_callback, 10
        )
        self.timer = self.create_timer(1 / FREQ, self.control_timer_callback)
        if PUB_ODOM:
            self.odom_pub = self.create_publisher(Odometry, "/drive/odom", 10)
            self.odom_timer = self.create_timer(1 / FREQ, self.odom_pub_callback)

    def odom_pub_callback(self):
        """
        Callback function to publish odometry information.
        """
        vl = 0
        vr = 0
        for wheel in self.wheels:
            if wheel.side == LEFT:
                vl += wheel.status.velocity
            else:
                vr += wheel.status.velocity
        vl /= len(self.wheels) / 2
        vr /= len(self.wheels) / 2

        angular_vel = (vl - vr) / self.BASE_WIDTH
        linear_vel = (vr + vl) / 2
        odom = self.create_odom(linear_vel, angular_vel)
        self.odom_pub.publish(odom)

    def create_odom(self, linear_vel, angular_vel):
        """
        Create an Odometry message.

        :param linear_vel: Linear velocity.
        :param angular_vel: Angular velocity.
        :return: Odometry message.
        """
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.FRAME
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.angular.z = angular_vel
        odom.twist.covariance = self.covariance
        return odom

    def control_timer_callback(self):
        """
        Timer callback function to control the wheels.
        """
        if self.get_clock().now().nanoseconds - self.last_timestamp > self.TIMEOUT:
            for wheel in self.wheels:
                wheel.set_vel(0.0)
        for wheel in self.wheels:
            wheel.send()

    def twist_callback(self, msg: Twist):
        """
        Callback function to handle Twist messages.

        :param msg: Twist message.
        """
        self.last_timestamp = self.get_clock().now().nanoseconds
        linear_x = msg.linear.x
        linear_x = min(linear_x, self.MAX_SPEED)
        linear_x = max(linear_x, -self.MAX_SPEED)

        vr = linear_x + msg.angular.z * self.BASE_WIDTH / 2  # m/s
        vl = linear_x - msg.angular.z * self.BASE_WIDTH / 2

        for wheel in self.wheels:
            if wheel.side == LEFT:
                wheel.set_vel(float(vl))
            else:
                wheel.set_vel(float(-vr))


def main(args=None):
    """
    Main function to initialize and spin the ROS2 node.
    """
    rclpy.init(args=args)
    node = TalonDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
