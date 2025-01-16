import math
from math import cos, pi, sin

from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from std_msgs.msg import Float64
from tf_transformations import quaternion_from_euler


class EncoderWrapper:
    def __init__(
        self,
        ticks_per_meter,
        ticks_per_rotation,
        base_width,
        parent_node,
    ):
        """Encoder Odometry + Electrical

        Args:
            ticks_per_meter (float): Ticks per meter, according to the ROS parameters
            ticks_per_rotation (float): Ticks per wheel rotation, according to the ROS parameters
            base_width (float): Base width (baseline) of the robot
            parent_node (rclpy.node.Node): RCL Node
        """
        self.TICKS_PER_METER = ticks_per_meter
        self.TICKS_PER_ROTATION = ticks_per_rotation
        self.BASE_WIDTH = base_width
        # Node parameters
        self.parent_node = parent_node
        self.clock = self.parent_node.get_clock()
        self.logger = self.parent_node.get_logger()
        # Odometry, left and right encoders publishers
        self.odom_pub = self.parent_node.odom_pub
        self.left_encoder_pub = self.parent_node.left_encoder_pub
        self.right_encoder_pub = self.parent_node.right_encoder_pub
        # Odometry data
        self.cur_x = 0
        self.cur_y = 0
        self.cur_theta = 0.0
        self.last_enc_left = 0
        self.last_enc_right = 0
        self.last_enc_time = self.clock.now().nanoseconds
        self.vel_theta = 0
        self.left_ang_vel = 0
        self.right_ang_vel = 0

    @staticmethod
    def normalize_angle(angle):
        while angle > pi:
            angle -= 2.0 * pi
        while angle < -pi:
            angle += 2.0 * pi
        return angle

    def update(self, enc_left, enc_right):
        # Hack solution to accomidate for reversing motor 1 is make it negative
        left_ticks = enc_left - self.last_enc_left
        right_ticks = -(enc_right - self.last_enc_right)
        self.last_enc_left = enc_left
        self.last_enc_right = enc_right

        dist_left = left_ticks / self.TICKS_PER_METER
        dist_right = right_ticks / self.TICKS_PER_METER
        dist = (dist_right + dist_left) / 2.0

        current_time = self.clock.now().nanoseconds
        d_time = current_time - self.last_enc_time
        self.last_enc_time = current_time

        self.left_ang_vel = (
            2 * math.pi * left_ticks / (self.TICKS_PER_ROTATION * d_time * 1e-9)
        )
        self.right_ang_vel = (
            2 * math.pi * right_ticks / (self.TICKS_PER_ROTATION * d_time * 1e-9)
        )

        # TODO find better what to determine going straight, this means slight deviation is accounted
        if left_ticks == right_ticks:
            d_theta = 0.0
            self.cur_x += dist * cos(self.cur_theta)
            self.cur_y += dist * sin(self.cur_theta)
        else:
            # delta theta
            d_theta = -(dist_right - dist_left) / self.BASE_WIDTH
            r = dist / d_theta
            self.cur_x += r * (sin(d_theta + self.cur_theta) - sin(self.cur_theta))
            self.cur_y -= r * (cos(d_theta + self.cur_theta) - cos(self.cur_theta))
            self.cur_theta = self.normalize_angle(self.cur_theta + d_theta)

        if abs(d_time) < 1000:
            vel_x = 0.0
            vel_theta = 0.0
        else:
            vel_x = dist / (d_time * 1e-9)
            vel_theta = d_theta / (d_time * 1e-9)

        self.vel_theta = vel_theta
        return vel_x, vel_theta

    def update_n_publish(self, enc_left: float, enc_right: float, publish_time: Time):
        """Update odom and elec and publish all data accordingly

        Args:
            enc_left: Left Encoder Measurement
            enc_right: Right Encoder Measurement
            publish_time (Time): Current time
        """
        # 2106 per 0.1 seconds is max speed, error in the 16th bit is 32768
        # TODO lets find a better way to deal with this error
        if abs(enc_left - self.last_enc_left) > 20000:
            self.logger.error(
                "Ignoring left encoder jump: cur "
                + str(enc_left)
                + ", last "
                + str(self.last_enc_left)
            )
            return
        elif abs(enc_right - self.last_enc_right) > 20000:
            self.logger.error(
                "Ignoring right encoder jump: cur "
                + str(enc_right)
                + ", last "
                + str(self.last_enc_right)
            )
            return

        vel_x, vel_theta = self.update(enc_left, enc_right)
        self.publish_odom(
            self.cur_x, self.cur_y, self.cur_theta, publish_time, vel_x, vel_theta
        )

    def publish_odom(
        self,
        cur_x: float,
        cur_y: float,
        cur_theta: float,
        cur_time: Time,
        vx: float,
        vth: float,
    ):
        """Publish odometry

        Args:
            cur_x (float): Current x coordinate
            cur_y (float): Current y coordinate
            cur_theta (float): Current theta heading
            cur_time (Time): Current time
            vx (float): Linear speed - forward
            vth (float): angular speed => delta theta / time
        """
        quat = quaternion_from_euler(0, 0, cur_theta)

        t = TransformStamped()
        t.header.stamp = cur_time.to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "odom"
        t.transform.translation.x = cur_x
        t.transform.translation.y = cur_y
        t.transform.translation.z = 0.0
        q = quaternion_from_euler(0, 0, -cur_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        # br = tf2_ros.TransformBroadcaster(self.parent_node)
        # br.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = cur_time.to_msg()
        odom.header.frame_id = "odom"

        odom.pose.pose.position.x = cur_x
        odom.pose.pose.position.y = cur_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(
            x=quat[0], y=quat[1], z=quat[2], w=quat[3]
        )

        odom.pose.covariance[0] = 0.03
        odom.pose.covariance[7] = 0.03
        odom.pose.covariance[14] = 99999
        odom.pose.covariance[21] = 99999
        odom.pose.covariance[28] = 99999
        odom.pose.covariance[35] = 0.03

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = vth
        odom.twist.covariance = odom.pose.covariance

        left_enc = Float64()
        left_enc.data = self.left_ang_vel
        right_enc = Float64()
        right_enc.data = self.right_ang_vel

        self.odom_pub.publish(odom)

        self.left_encoder_pub.publish(left_enc)
        self.right_encoder_pub.publish(right_enc)
