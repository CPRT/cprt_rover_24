import rclpy
import math
from rclpy.node import Node
import tf
# from ublox_msgs.msg import NavRELPOSNED9  
from sensor_msgs.msg import Imu
from pyubx2.ubxtypes_configdb import SET_LAYER_RAM, SET_LAYER_BBR, SET_LAYER_FLASH
from pyubx2 import (
    UBX_PROTOCOL,
)
from .ubx_io_manager import UbxIoManager

# Defines
# TMODE_SVIN = 1
# TMODE_FIXED = 2


class HeadingNode(Node):
    """
    ROS 2 node for ending the via serial communication.

    Attributes:
        rtcm_pub (Publisher): Publishes Heading messages to the /rtcm topic.
        serial_conn (IoManager): Manages serial I/O operations.
        layers (int): Configuration layers for the UBXMessage.
        timer (Timer): Timer for periodically reading and publishing RTCM data.
    """

    def __init__(self):
        """
        Initializes the RTCM node, loads parameters, sets up serial communication,
        configures RTCM output, and starts a periodic timer callback.
        """
        super().__init__("heading_node")
        self.load_params()
        queue_depth = (
            self.get_parameter("QueueDepth").get_parameter_value().integer_value
        )
        self.heading_pub = self.create_publisher(Imu, "/heading", queue_depth)
        self.serial_conn = UbxIoManager(port=self.dev, baud=self.baudrate, msg_filter=UBX_PROTOCOL)
        self.layers = SET_LAYER_RAM | SET_LAYER_BBR
        if self.persistent:
            self.layers |= SET_LAYER_FLASH
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)

    def load_params(self):
        """
        Loads parameters from the ROS 2 parameter server with default values.
        Parameters include timing mode, survey-in settings, persistence, frequency,
        baud rate, and serial device.
        """
        self.declare_parameter("Persistent", False)
        self.persistent = (
            self.get_parameter("Persistent").get_parameter_value().bool_value
        )
        self.declare_parameter("Freq", 2.0)
        self.freq = self.get_parameter("Freq").get_parameter_value().double_value
        if self.freq <= 0:
            self.get_logger().warn("Frequency must be positive. Defaulting to 2.0 Hz.")
            self.freq = 2.0

        self.declare_parameter("Baudrate", 115200)
        self.baudrate = (
            self.get_parameter("Baudrate").get_parameter_value().integer_value
        )
        self.declare_parameter("Device", "/dev/ttyUSB0")
        self.dev = self.get_parameter("Device").get_parameter_value().string_value
        self.declare_parameter("QueueDepth", 1)


    def timer_callback(self):
        """
        Periodic callback to read RTCM data from the receiver and publish it to the /rtcm topic.
        If data is available, it publishes the raw data and logs the parsed message.
        """
        while self.serial_conn.data_available():
            raw, parsed_data = self.serial_conn.read()   

            if not raw:
                self.get_logger().warn("No data read from serial port.")
                return
            
            self.imu.linear_acceleration_covariance[0] = -1
            self.imu.angular_velocity_covariance[0] = -1
            heading = math.pi / 2 - (m.relPosHeading * 1e-5 / 180.0 * math.pi)
            orientation = tf.transformations.quaternion_from_euler(0, 0, heading)
            self.imu.orientation.x = orientation[0]
            self.imu.orientation.y = orientation[1]
            self.imu.orientation.z = orientation[2]
            self.imu.orientation.w = orientation[3]
            self.imu.orientation_covariance[0] = 1000.0
            self.imu.orientation_covariance[4] = 1000.0
            self.imu.orientation_covariance[8] = 1000.0

            # When heading is reported to be valid, use accuracy reported in 1e-5 deg units
            #if heading_is_valid set to 1:
                # self.imu.orientation_covariance[8] = (m.accHeading * 1e-5 / 180.0 * math.pi) ** 2

            self.heading_pub.publish(self.imu)
            self.get_logger().info(
                f"Published UBX message of length {len(raw)}. Parsed message: {parsed_data}"
            )
#Find is N or E is 0
#Convert heading to geometry_msgs/Quaternion orientation from cpp(https://github.com/KumarRobotics/ublox/blob/master/ublox_gps/src/node.cpp). Is heading an eular angle?
#And the covarience(accurcy from messages)
#Last touches(Publishers)

def main(args=None):
    """
    Main entry point for the Heading node.

    Initializes the ROS 2 system, creates the Heading node, and starts spinning the event loop.
    """ 
    rclpy.init(args=args)
    heading_node = HeadingNode()
    rclpy.spin(heading_node)
    heading_node.destroy_node()
    rclpy.shutdown()

