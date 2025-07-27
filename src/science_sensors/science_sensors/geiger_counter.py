import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial


class GeigerReader(Node):
    def __init__(self):
        super().__init__("geiger_reader")
        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("test_mode", False)

        port = self.get_parameter("serial_port").get_parameter_value().string_value
        baudrate = self.get_parameter("baudrate").get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(Float32, "geiger_counts", 10)

        try:
            self.serial = serial.Serial(port, baudrate, timeout=1)
            self.get_logger().info(
                f"GeigerReader connected to {port} at {baudrate} baud"
            )
        except serial.SerialException as e:
            self.get_logger().error(f"Serial port error: {e}")
            raise SystemExit(1)

        self.timer = self.create_timer(0.01, self.read_line)
        if self.get_parameter("test_mode").get_parameter_value().bool_value:
            self.get_logger().warn(
                "Test mode active: Hook up the rx to the tx to test receiving data"
            )
            self.test_timer = self.create_timer(0.1, self.send_msg)

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

    def send_msg(self):
        import random

        try:
            number = random.random() * 5
            digits_to_include = random.randint(1, 10)
            rounded_num = round(number, digits_to_include)
            self.serial.write((str(rounded_num) + "\n").encode())
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")


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
