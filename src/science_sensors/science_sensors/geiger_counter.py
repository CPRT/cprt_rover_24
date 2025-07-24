import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class GeigerReader(Node):
    def __init__(self):
        super().__init__("gieger_reader")
        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Float32, "geiger_counts", 10)

        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(
                f"GiegerReader connected to {port} at {baudrate} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            raise SystemExit

        self.timer = self.create_timer(0.001, self.read_line)

    def read_line(self):
        try:
            if self.serial.in_waiting:
                line = self.serial.readline().decode("ascii", errors="ignore").strip()
                if line:
                    try:
                        value = float(line)
                        msg = Float32()
                        msg.data = value
                        self.publisher_.publish(msg)
                    except ValueError:
                        self.get_logger().warn(f"Invalid float: '{line}'")
        except Exception as e:
            self.get_logger().error(f"Serial read failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = GeigerReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
