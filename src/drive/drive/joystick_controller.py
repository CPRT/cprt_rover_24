import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist


class Curve:
    """
    Class to handle different types of curves for joystick input.
    """

    def __init__(self, curve_type):
        """
        Initialize the Curve object.

        :param curve_type: Type of curve to use (e.g., "linear").
        """
        self.curve = lambda x: x
        self.set_curve(curve_type)

    def set_curve(self, curve_type):
        """
        Set the type of curve to use.

        :param curve_type: Type of curve to use (e.g., "linear").
        """
        if curve_type == "linear":
            self.curve = lambda x: x
        else:
            rclpy.logging.get_logger("joystick_controller").warn(
                f"Failed to change curve type. Unknown curve {curve_type}."
            )
            rclpy.logging.get_logger("joystick_controller").info(
                "Curve options are: linear"
            )

    def __call__(self, x):
        """
        Apply the curve to the input value.

        :param x: Input value.
        :return: Curved value.
        """
        return self.curve(x)


class ButtonLayout:
    """
    Class to handle the button layout of the joystick.
    """

    def __init__(self, deadman_button, turbo_button, linear_axis, angular_axis):
        """
        Initialize the ButtonLayout object.

        :param deadman_button: Index of the deadman button.
        :param turbo_button: Index of the turbo button.
        :param linear_axis: Index of the linear axis.
        :param angular_axis: Index of the angular axis.
        """
        self.deadman_button = deadman_button
        self.turbo_button = turbo_button
        self.linear_axis = linear_axis
        self.angular_axis = angular_axis

    def deadman(self, msg):
        """
        Check if the deadman button is pressed.

        :param msg: Joy message.
        :return: True if the deadman button is pressed, False otherwise.
        """
        return self.deadman_button < 0 or msg.buttons[self.deadman_button] == 1

    def turbo(self, msg):
        """
        Check if the turbo button is pressed.

        :param msg: Joy message.
        :return: True if the turbo button is pressed, False otherwise.
        """
        return self.turbo_button >= 0 and msg.buttons[self.turbo_button] == 1

    def linear(self, msg):
        """
        Get the linear axis value from the joystick message.

        :param msg: Joy message.
        :return: Linear axis value.
        """
        return msg.axes[self.linear_axis]

    def angular(self, msg):
        """
        Get the angular axis value from the joystick message.

        :param msg: Joy message.
        :return: Angular axis value.
        """
        return msg.axes[self.angular_axis]


class JoystickController(Node):
    """
    ROS2 Node to control the robot using a joystick.
    """

    def __init__(self):
        """
        Initialize the JoystickController node.
        """
        super().__init__("joystick_controller")
        self.declare_parameter("linear_axis_index", 0)
        linear_axis = (
            self.get_parameter("linear_axis_index").get_parameter_value().integer_value
        )
        self.declare_parameter("turn_axis_index", 1)
        turn_axis = (
            self.get_parameter("turn_axis_index").get_parameter_value().integer_value
        )
        self.declare_parameter("turbo_button", -1)
        turbo_button = (
            self.get_parameter("turbo_button").get_parameter_value().integer_value
        )
        self.declare_parameter("deadman_button", -1)
        deadman_button = (
            self.get_parameter("deadman_button").get_parameter_value().integer_value
        )
        self.declare_parameter("max_linear_speed", 1.0)
        self.max_linear_speed = (
            self.get_parameter("max_linear_speed").get_parameter_value().double_value
        )
        self.declare_parameter("max_angular_speed", 1.0)
        self.max_angular_speed = (
            self.get_parameter("max_angular_speed").get_parameter_value().double_value
        )
        self.declare_parameter("turbo_multiplier", 2.0)
        self.turbo_multiplier = (
            self.get_parameter("turbo_multiplier").get_parameter_value().double_value
        )
        self.declare_parameter("curve_type", "linear")
        curve_type = self.get_parameter("curve_type").get_parameter_value().string_value

        self.layout = ButtonLayout(deadman_button, turbo_button, linear_axis, turn_axis)
        self.curve = Curve(curve_type)

        self.twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)

    def joy_callback(self, msg: Joy):
        """
        Callback function to handle joystick messages.

        :param msg: Joy message.
        """
        if not self.layout.deadman(msg):
            return

        twist = Twist()
        twist.linear.x = self.curve(self.layout.linear(msg)) * self.max_linear_speed
        twist.angular.z = (
            -1 * self.curve(self.layout.angular(msg)) * self.max_angular_speed
        )

        if self.layout.turbo(msg):
            twist.linear.x *= self.turbo_multiplier
            twist.angular.z *= self.turbo_multiplier

        self.twist_pub.publish(twist)


def main(args=None):
    """
    Main function to initialize and spin the ROS2 node.
    """
    rclpy.init(args=args)
    node = JoystickController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
