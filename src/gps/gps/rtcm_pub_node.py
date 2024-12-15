import threading
import rclpy
import serial
from rclpy.node import Node
from rtcm_msgs.msg import Message as Rtcm
from pyubx2.ubxtypes_configdb import SET_LAYER_RAM, SET_LAYER_BBR, SET_LAYER_FLASH
from pyubx2 import (
    RTCM3_PROTOCOL,
    UBXMessage,
    UBXMessageError,
    UBXParseError,
    UBXReader,
    VALCKSUM,
)

# Defines
TMODE_SVIN = 1
TMODE_FIXED = 2


class IoManager:
    """
    Manages serial I/O operations, ensuring thread safety for reading and writing data.

    Attributes:
        lock (threading.Lock): Ensures thread-safe access to the serial port.
        worker (serial.Serial): Serial connection instance.
        ubr (UBXReader): UBXReader instance for parsing RTCM/UBX data.
    """

    def __init__(self, port="/dev/ttyACM0", baud=38400):
        """
        Initializes the serial connection and UBXReader.

        Args:
            port (str): Serial port to connect to.
            baud (int): Baud rate for the serial connection.

        Raises:
            RuntimeError: If the serial port cannot be opened.
        """
        self.lock = threading.Lock()
        with self.lock:
            try:
                self.worker = serial.Serial(port, baud, timeout=1)
            except serial.SerialException as e:
                raise RuntimeError(f"Failed to open serial port {port}: {e}") from e
            self.ubr = UBXReader(
                self.worker,
                protfilter=RTCM3_PROTOCOL,
                validate=VALCKSUM,
            )

    def read(self) -> tuple:
        """
        Reads data from the serial port using UBXReader.

        Returns:
            tuple: Raw data and parsed data from UBXReader.

        Raises:
            RuntimeError: If reading from the serial port fails.
        """
        with self.lock:
            try:
                return self.ubr.read()
            except Exception as e:
                raise RuntimeError(f"Error reading from serial port: {e}") from e

    def write(self, data: bytes):
        """
        Writes data to the serial port.

        Args:
            data (bytes): Data to write to the serial port.

        Raises:
            RuntimeError: If writing to the serial port fails.
        """
        with self.lock:
            try:
                self.worker.write(data)
            except Exception as e:
                raise RuntimeError(f"Error writing to serial port: {e}") from e

    def data_available(self) -> bool:
        """
        Checks if there is data available to read from the serial port.

        Returns:
            bool: True if data is available, False otherwise.
        """
        with self.lock:
            return self.worker.in_waiting > 0


class RtcmNode(Node):
    """
    ROS 2 node for managing RTCM data via serial communication.

    Attributes:
        rtcm_pub (Publisher): Publishes RTCM messages to the /rtcm topic.
        serial_conn (IoManager): Manages serial I/O operations.
        layers (int): Configuration layers for the UBXMessage.
        timer (Timer): Timer for periodically reading and publishing RTCM data.
    """

    def __init__(self):
        """
        Initializes the RTCM node, loads parameters, sets up serial communication,
        configures RTCM output, and starts a periodic timer callback.
        """
        super().__init__("rtcm_node")
        self.load_params()
        self.rtcm_pub = self.create_publisher(Rtcm, "/rtcm", 1)
        self.serial_conn = IoManager(port=self.dev, baud=self.baudrate)
        self.layers = SET_LAYER_RAM | SET_LAYER_BBR
        if self.persistent:
            self.layers |= SET_LAYER_FLASH
        self.config_rtcm()
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)

    def load_params(self):
        """
        Loads parameters from the ROS 2 parameter server with default values.
        Parameters include timing mode, survey-in settings, persistence, frequency,
        baud rate, and serial device.
        """
        self.declare_parameter("TimingMode", TMODE_SVIN)
        self.time_mode = (
            self.get_parameter("TimingMode").get_parameter_value().integer_value
        )
        if self.time_mode not in (TMODE_SVIN, TMODE_FIXED):
            self.get_logger().warn(
                f"Unknown timing mode {self.time_mode}. Defaulting ..."
            )
            self.time_mode = TMODE_SVIN
        self.declare_parameter("MinTime", 600)
        self.min_time = (
            self.get_parameter("MinTime").get_parameter_value().integer_value
        )
        self.declare_parameter("MinAcc", 0.5)
        self.min_acc = self.get_parameter("MinAcc").get_parameter_value().double_value
        self.declare_parameter("Persistent", False)
        self.persistent = (
            self.get_parameter("Persistent").get_parameter_value().bool_value
        )
        self.declare_parameter("Freq", 2.0)
        self.freq = self.get_parameter("Freq").get_parameter_value().double_value
        if self.freq <= 0:
            self.get_logger().warn("Frequency must be positive. Defaulting to 2.0 Hz.")
            self.freq = 2.0

        self.declare_parameter("Baudrate", 38400)
        self.baudrate = (
            self.get_parameter("Baudrate").get_parameter_value().integer_value
        )
        self.declare_parameter("Device", "/dev/ttyACM0")
        self.dev = self.get_parameter("Device").get_parameter_value().string_value

    def config_rtcm(self, port_type: str = "USB") -> UBXMessage:
        """
        Configures the output of RTCM messages on the receiver.

        Args:
            port_type (str): Port type for RTCM messages (e.g., "USB").

        Returns:
            UBXMessage: The configuration message sent to the receiver.

        Raises:
            UBXMessageError, UBXParseError, RuntimeError: If configuration fails.
        """
        transaction = 0
        cfg_data = []
        rtcm_types = ("1005", "1077", "1087", "1097", "1127", "1230")

        for rtcm_type in rtcm_types:
            cfg = f"CFG_MSGOUT_RTCM_3X_TYPE{rtcm_type}_{port_type}"
            cfg_data.append((cfg, 1))

        try:
            ubx = UBXMessage.config_set(self.layers, transaction, cfg_data)
            self.get_logger().info(f"Sending Config message: {ubx}")
            self.serial_conn.write(ubx.serialize())
        except (UBXMessageError, UBXParseError, RuntimeError) as e:
            self.get_logger().error(f"Error configuring RTCM messages: {e}")
            return

        if self.time_mode == TMODE_SVIN:
            acc_limit = int(round(self.min_acc * 10000, 0))
            cfg_data = [
                ("CFG_TMODE_MODE", TMODE_SVIN),
                ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),
                ("CFG_TMODE_SVIN_MIN_DUR", self.min_time),
                (f"CFG_MSGOUT_UBX_NAV_SVIN_{port_type}", 1),
            ]
            try:
                ubx = UBXMessage.config_set(self.layers, transaction, cfg_data)
                self.serial_conn.write(ubx.serialize())
            except (UBXMessageError, UBXParseError, RuntimeError) as e:
                self.get_logger().error(f"Error setting survey-in mode: {e}")
        else:
            self.get_logger().warn("Fixed mode is not implemented yet.")

    def timer_callback(self):
        """
        Periodic callback to read RTCM data from the receiver and publish it to the /rtcm topic.
        If data is available, it publishes the raw data and logs the parsed message.
        """
        while self.serial_conn.data_available():
            try:
                raw, parsed_data = self.serial_conn.read()
            except Exception as e:
                self.get_logger().error(f"Error reading or publishing RTCM data: {e}")
                return

            if not raw:
                self.get_logger().warn("No data read from serial port.")
                return
            msg = Rtcm()
            msg.message = list(raw)
            self.rtcm_pub.publish(msg)
            self.get_logger().info(
                f"Published RTCM message of length {len(raw)}. Parsed message: {parsed_data}"
            )


def main(args=None):
    """
    Main entry point for the RTCM node.

    Initializes the ROS 2 system, creates the RTCM node, and starts spinning the event loop.
    """
    rclpy.init(args=args)
    rtcm_node = RtcmNode()
    rclpy.spin(rtcm_node)
    rtcm_node.destroy_node()
    rclpy.shutdown()
