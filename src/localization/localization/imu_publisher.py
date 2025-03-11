import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
import adafruit_bno08x
import yaml
import os


class BNO08XPublisher(Node):
    def __init__(self, frame_id = "imu_link"):
        super().__init__("imu_publisher")

        # Initialize I2C communication
        i2c = busio.I2C(board.SCL, board.SDA)
        self.sensor = adafruit_bno08x.BNO08X_I2C(i2c)

        # IMU Publisher
        self.publisher_ = self.create_publisher(Imu, "imu/data", 10)
        self.load_covariance_config()

        # Timer to publish data
        Rate = 0.05 
        self.timer = self.create_timer(Rate, self.publish_imu_data)

        self.get_logger().info(f"BNO08X IMU Node Started! Frame ID : {self.frame_id}")

    def load_covariance_config(self):
        config_path = "/src/localization/config/imu_config.yaml"
        
        try:
            with open(config_path, "r") as file:
                config = yaml.safe_load(file)
                self.orientation_covariance = config["orientation_covariance"]
                self.angular_velocity_covariance = config["angular_velocity_covariance"]
                self.linear_acceleration_covariance = config["linear_acceleration_covariance"]
                self.get_logger().info("Loaded covariance values from config file.")
        except Exception as e:
            self.get_logger().warn(f"Failed to load covariance config: {e}. Using defaults.")
            self.orientation_covariance = [0.01] * 9
            self.angular_velocity_covariance = [0.01] * 9
            self.linear_acceleration_covariance = [0.01] * 9


    def publish_imu_data(self):
        msg = Imu()

        # Get sensor readings
        quat = self.sensor.quaternion
        accel = self.sensor.acceleration
        gyro = self.sensor.gyro

        # Fill in IMU message
        msg.orientation.x = quat[2]
        msg.orientation.y = -quat[1]
        msg.orientation.z = quat[3]
        msg.orientation.w = quat[0]

        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]

        msg.linear_acceleration.x = accel[0]
        msg.linear_acceleration.y = accel[1]
        msg.linear_acceleration.z = accel[2]


        msg.orientation_covariance = self.orientation_covariance
        msg.angular_velocity_covariance = self.angular_velocity_covariance
        msg.linear_acceleration_covariance = self.linear_acceleration_covariance

        # Publish the IMU message
        self.publisher_.publish(msg)
        #self.get_logger().info("Published IMU Data")


def main(args=None):
    rclpy.init(args=args)
    node = BNO08XPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
