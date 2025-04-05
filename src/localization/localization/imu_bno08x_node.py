import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
import adafruit_bno08x
import os
import math

def euler_from_quaternion(q):
    """
    Convert a quaternion into euler angles
    taken from: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
    """
    t0 = +2.0 * (q.w * q.x + q.y * q.z)
    t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (q.w * q.y - q.z * q.x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z

class BNO08XPublisher(Node):
    def __init__(self):
        super().__init__("imu_pub_node")
        self.load_params()
        # IMU Publisher
        queue_depth = (
            self.get_parameter("QueueDepth").get_parameter_value().integer_value
        )
        self.imu_pub = self.create_publisher(Imu, "imu/data", queue_depth)

        # Initialize I2C communication
        i2c = busio.I2C(
            board.SCL_1, board.SDA_1
        )  # On RPI it was usio.I2C(board.SCL, board.SDA)
        self.sensor = BNO08X_I2C(
            i2c, address=0x4b
        )  # on rpi it was adafruit_bno08x.BNO08X_I2C

        self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
        self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
        # self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)
        self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_MAGNETOMETER)

        # Timer to publish data
        self.timer = self.create_timer(1 / self.freq, self.timer_callback)

        self.frame_id="imu_link"
        self.get_logger().info(f"BNO08X IMU Node Started! Frame ID : {self.frame_id}")

    def load_params(self):
        self.declare_parameter("orientation_covariance", [0.01] * 9)
        self.orientation_covariance = (
            self.get_parameter("orientation_covariance")
            .get_parameter_value()
            .double_array_value
        )
        self.declare_parameter("angular_velocity_covariance", [0.01] * 9)
        self.angular_velocity_covariance = (
            self.get_parameter("angular_velocity_covariance")
            .get_parameter_value()
            .double_array_value
        )
        self.declare_parameter("linear_acceleration_covariance", [0.01] * 9)
        self.linear_acceleration_covariance = (
            self.get_parameter("linear_acceleration_covariance")
            .get_parameter_value()
            .double_array_value
        )
        self.declare_parameter("Freq", 2.0)
        self.freq = self.get_parameter("Freq").get_parameter_value().double_value
        if self.freq <= 0:
            self.get_logger().warn("Frequency must be positive. Defaulting to 2.0 Hz.")
            self.freq = 2.0
        self.declare_parameter("QueueDepth", 10)

    def timer_callback(self):
        msg = Imu()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()

        # Get sensor readings
        quat = self.sensor.geomagnetic_quaternion
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
        self.imu_pub.publish(msg)

        # roll, pitch, yaw = euler_from_quaternion(msg.orientation)
        # roll, pitch, yaw = math.degrees(roll), math.degrees(pitch), math.degrees(yaw)        
        # self.get_logger().info(f"Roll: {roll:.2f}, pitch: {pitch:.2f}, yaw: {yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = BNO08XPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
