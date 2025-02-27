import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno08x

class BNO08XPublisher(Node):
    def __init__(self):
        super().__init__('bno08x_publisher')

        # Initialize I2C communication
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno08x.BNO08X_I2C(i2c)

        # IMU Publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)

        # Timer to publish data
        self.timer = self.create_timer(0.05, self.publish_imu_data)

        self.get_logger().info("BNO08X IMU Node Started!")

    def publish_imu_data(self):
        msg = Imu()

        # Get sensor readings
        quat = self.sensor.quaternion
        accel = self.sensor.acceleration
        gyro = self.sensor.gyro

        # Fill in IMU message
        msg.orientation.x = quat[1]
        msg.orientation.y = quat[2]
        msg.orientation.z = quat[3]
        msg.orientation.w = quat[0]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]

        # Publish the IMU message
        self.publisher_.publish(msg)
        self.get_logger().info("Published IMU Data")

def main(args=None):
    rclpy.init(args=args)
    node = BNO08XPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
