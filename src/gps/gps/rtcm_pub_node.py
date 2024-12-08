import rclpy
import threading
from rclpy.node import Node
from rtcm_msgs.msg import Message as Rtcm

# Defines
TMODE_SVIN = 1
TMODE_FIXED = 2

RAM = 1
BBR = 2
FLASH = 4

class IoManager:
    def __init__(self, port="/dev/ttyACM0", baud=38400):
        try:
            self.worker = serial.Serial(port, baud, timeout=1)
        except serial.SerialException as e:
            raise RuntimeError(f"Failed to open serial port {port}: {e}")
        self.lock = threading.Lock()

    def read(self) -> bytes:
        with self.lock:
            return self.worker.read(self.worker.in_waiting())

    def write(self, data: bytes):
        with self.lock:
            self.worker.write(data)

    def data_available(self) -> bool:
        with self.lock:
            return self.worker.in_waiting() > 0



class Rtcm_Node(Node):
    def __init__(self):
        super().__init__("rtcm_node")
        self.load_params()
        self.rtcm_pub = self.create_publisher(Rtcm, "/rtcm", 1)
        self.serial_conn = io_manager(port=self.dev, baud=self.baudrate)
        self.layers = RAM | BBR
        if(self.persistent):
            self.layers |= FLASH
        self.config_rtcm()
        self.timer = self.create_timer(1/self.freq, self.timer_callback)

    def load_params(self):
        self.declare_parameter("TimingMode", TMODE_SVIN)
        self.time_mode = (
            self.get_parameter("TimingMode").get_parameter_value().integer_value
        )
        if self.time_mode not in (TMODE_SVIN, TMODE_FIXED):
            self.get_logger().warn(f"Unknown timing mode {self.time_mode}. Defaulting ...")
            self.time_mode = TMODE_SVIN
        self.declare_parameter("MinTime", 600)
        self.min_time = (
            self.get_parameter("MinTime").get_parameter_value().integer_value
        )
        self.declare_parameter("MinAcc", 0.5)
        self.min_acc = (
            self.get_parameter("MinAcc").get_parameter_value().double_value
        )
        self.declare_parameter("Persistent", False)
        self.persistent = (
            self.get_parameter("Persistent").get_parameter_value().bool_value
        )
        self.declare_parameter("Freq", 2.0)
        self.freq = (
            self.get_parameter("Freq").get_parameter_value().double_value
        )
        if self.freq <= 0:
            self.get_logger().warn("Frequency must be positive. Defaulting to 2.0 Hz.")
            self.freq = 2.0

        self.declare_parameter("Baudrate", 38400)
        self.baudrate = (
            self.get_parameter("Baudrate").get_parameter_value().integer_value
        )
        self.dev = (
            self.get_parameter("Device").get_parameter_value().string_value
        )
        
    
    def config_rtcm(self, port_type: str = "USB") -> UBXMessage:
        transaction = 0

        cfg_data = []
        rtcm_types = ("1005", "1077", "1087", "1097", "1127", "1230")
        for rtcm_type in rtcm_types:
            cfg = f"CFG_MSGOUT_RTCM_3X_TYPE{rtcm_type}_{port_type}"
            cfg_data.append([cfg, 1])

        ubx = UBXMessage.config_set(self.layers, transaction, cfg_data)    
        self.get_logger().info(f"Sending Config message: {ubx}")
        self.serial_conn.write(ubx.serialize())
        if TMODE == TMODE_SVIN:
            layers = 1
            transaction = 0
            acc_limit = int(round(self.min_acc * 10000, 0))
            cfg_data = [
                ("CFG_TMODE_MODE", TMODE_SVIN),
                ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),
                ("CFG_TMODE_SVIN_MIN_DUR", self.min_time),
                (f"CFG_MSGOUT_UBX_NAV_SVIN_{port_type}", 1),
            ]
            ubx = UBXMessage.config_set(self.layers, transaction, cfg_data)
        else:
            self.get_logger().warn(f"Fixed mode not implemented yet")
        ubx = UBXMessage.config_set(self.layers, transaction, cfg_data)
    
    def timer_callback(self):
        if self.serial_conn.data_availible():
            data = self.serial_conn.read()
            msg = RTCM()
            msg.message = list(data)
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published RTCM message: {len(msg.data)} bytes')

def main(args=None):
    rclpy.init(args=args)
    rtcm_node = Rtcm_Node()
    rclpy.spin(rtcm_node)
    rtcm_node.destroy_node()
    rclpy.shutdown()