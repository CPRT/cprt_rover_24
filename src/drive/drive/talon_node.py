#!/usr/bin/env python
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import rclpy.time
from ros_phoenix.msg import MotorControl, MotorStatus
from sensor_msgs.msg import Joy

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

class talonDriveController(Node):
    def __init__(self):
        super().__init__("talonDrive")
        self.lastTimestamp = 0
        
        self.declare_parameter("max_speed", 2.0)
        self.MAX_SPEED = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        self.declare_parameter("ticks_per_meter", 819)
        self.TICKS_PER_METER = (
            self.get_parameter("ticks_per_meter").get_parameter_value().integer_value
        )
        self.declare_parameter("ticks_per_rotation", 1024)
        self.TICKS_PER_ROTATION = (
            self.get_parameter("ticks_per_rotation").get_parameter_value().integer_value
        )
        self.declare_parameter("base_width", 0.9144)
        self.BASE_WIDTH = (
            self.get_parameter("base_width").get_parameter_value().double_value
        )
        self.declare_parameter("pub_odom", True)
        self.PUB_ODOM = self.get_parameter("pub_odom").get_parameter_value().bool_value
        self.declare_parameter("pub_elec", True)
        self.PUB_ELEC = self.get_parameter("pub_elec").get_parameter_value().bool_value
        self.declare_parameter("stop_movement", True)
        self.STOP_MOVEMENT = (
            self.get_parameter("stop_movement").get_parameter_value().bool_value
        )

        self.declare_parameter("wheels", ["frontRight", "frontLeft", "backRight", "backLeft"])
        self.wheels = self.get_parameter("wheels").get_parameter_value().string_array_value

        self.wheelPub = {}
        self.wheelControl = {}

        for wheel in self.wheels:
            self.wheelPub[wheel] = self.create_publisher(MotorControl, f"/{wheel}/set", 10)
            self.wheelControl[wheel] = MotorControl()
            self.wheelControl[wheel].mode = 0 #initialize to velocity mode
            self.create_subscription(MotorStatus, f"/{wheel}/status", lambda msg: self.wheelStatusCallback(msg, wheel), 5) #this should pass in the wheel name to the subscriber

        # self.twistSub = self.create_subscription(Twist, "/drive/cmd_vel", self.twist_callback, 10)
        self.joySub = self.create_subscription(Joy, "/joystick/drive", self.twist_callback, 10)
        self.get_logger().info(f"THERE ARE THIS MANY WHEELS IN THE DRIVE "  + str(len(self.wheels)))

        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        # self.timer = self.create_timer(period, self.controlPublisher)

    def wheelStatusCallback(self, msg: MotorStatus, wheel: str):
        # self.get_logger().info(f"wheel {wheel} status: pos {msg.position}, vel {msg.velocity}")
        return

    # def controlPublisher(self):
    #     # if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2 or self.estop.data == True):
    #     #     return
    #     for wheel in self.wheels:
    #         self.wheelPub[wheel].publish(self.wheelControl[wheel])

    def twist_callback(self, msg: Joy):
        twist = Twist()
        twist.linear.x = msg.axes[3]
        twist.angular.z = msg.axes[2]

        if(twist.linear.x == 0 and twist.angular.z == 0):
            for wheel in self.wheels:
                self.wheelControl[wheel].value = 0.0
            for wheel in self.wheels:
                self.wheelPub[wheel].publish(self.wheelControl[wheel])
            return
        linear_x = twist.linear.x
        if linear_x > self.MAX_SPEED:  # check for messages above speed limit
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        # vr and vl are how fast the velocity on the left and right side is in m/s
        vr = linear_x - twist.angular.z * self.BASE_WIDTH / 2  # m/s
        vl = linear_x + twist.angular.z * self.BASE_WIDTH / 2
        # wheelnames = ' '
        # wheelnames = ' '.join(self.wheels)

        # ticks convert the speed the wheel needs to go into encoder ticks per second
        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        for wheel in self.wheels:
            if "Left" in wheel:
                self.wheelControl[wheel].value = float(-vl) 
            else:
                self.wheelControl[wheel].value = float(vr)
        # self.get_logger().info(f"vl: {vl}, vr: {vr}" + wheelnames)
        for wheel in self.wheels:
            self.wheelPub[wheel].publish(self.wheelControl[wheel])


def main(args=None):
    rclpy.init(args=args)
    node = talonDriveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
