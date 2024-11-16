from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
from interfaces.msg import SixFloats
from interfaces.srv import ArmPos
from trajectory_msgs.msg import JointTrajectoryPoint
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value


class trajectoryInterpreter(Node):
    def __init__(self):
        super().__init__("trajectoryInt")
        self.expectedTraj = JointTrajectoryPoint()
        self.motorCommand = MotorControl()


        self.baseAngle = 0.0
        self.baseZeroPoint = 0.0
        self.diff1Cont = 0.0
        self.diff1Angle = 0.0
        self.diff1ZeroPoint = 0.0
        self.diff2Cont = 0.0
        self.diff2Angle = 0.0
        self.diff2ZeroPoint = 0.0
        self.elbowAngle = 0.0
        self.elbowZeroPoint = 0.0
        self.wristTurnAngle = 0.0
        self.wristTurnZeroPoint = 0.0
        self.wristTiltAngle = 0.0
        self.wristTiltZeroPoint = 0.0


        self.estop = Bool()
        self.lastTimestamp = 0

        self.srv = self.create_service(ArmPos, 'arm_pos', self.get_arm_pos_callback)

        self.setEstop = self.create_publisher(
            Bool, "/drive/estop", 1)
        
        self.anglePub = self.create_publisher(
            SixFloats, "/arm/Angle" , 1)
        
        self.traj_sub = self.create_subscription(
            JointTrajectoryPoint, "/arm/expectedTraj", self.cmd_traj_callback, 10) 
        
        self.base_motor = self.create_subscription(
            MotorStatus, "/base/status", self.base_callback, 5)
        self.diff_motor1 = self.create_subscription(
            MotorStatus, "/diff1/status", self.diff1_callback, 5)
        self.diff_motor2 = self.create_subscription(
            MotorStatus, "/diff2/status", self.diff2_callback, 5)
        self.elbow_motor = self.create_subscription(
            MotorStatus, "/elbow/status", self.elbow_callback, 5)
        self.wrist_turn_motor = self.create_subscription(
            MotorStatus, "/wristTurn/status", self.wrist_turn_callback, 5)
        self.wrist_tilt_motor = self.create_subscription(
            MotorStatus, "/wristTilt/status", self.wrist_tilt_callback, 5)
        
        self.setEstop.publish(self.estop) #init as not estoped
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.anglepublisher)

    def base_callback(self, msg: MotorStatus):
        self.baseAngle = 0.0
    def diff1_callback(self, msg: MotorStatus):
        self.diff1Cont = self.diff1ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.diff1Angle = (self.diff1Cont - self.diff2Cont) * 4
        print("bruh1" + str(self.diff1Angle))
    def diff2_callback(self, msg: MotorStatus):
        self.diff2Cont = self.diff2ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.diff2Angle = (self.diff1Cont + self.diff2Cont) * 4
        print("bruh2" + str(self.diff2Angle))
    def elbow_callback(self, msg: MotorStatus):
        self.elbowAngle = (self.elbowZeroPoint + msg.position * ((2*pi)/1000 * ((1/83) * (1/100) * (16/13)))) * 4
    def wrist_turn_callback(self, msg: MotorStatus):
        self.wristTurnAngle = 0.0
    def wrist_tilt_callback(self, msg: MotorStatus):
        self.wristTiltAngle = 0.0

    def anglepublisher(self):
        out = SixFloats()
        out.m0 = self.elbowAngle 
        out.m1 = self.diff1Angle 
        out.m2 = self.diff2Angle
        out.m3 = self.elbowAngle
        self.m4 = self.wristTiltAngle
        self.m5 = self.wristTurnAngle
        self.anglePub.publish(out)
    
    def get_arm_pos_callback(self, request, response):
        response.base = self.elbowAngle 
        response.diff1 = self.diff1Angle 
        response.diff2 = self.diff2Angle
        response.elbow = self.elbowAngle
        response.wristturn = self.wristTurnAngle
        response.wristtilt = self.wristTiltAngle
        return response

    def cmd_traj_callback(self, msg: JointTrajectoryPoint):
        self.expectedTraj = msg;
        self.lastTimestamp = Node.get_clock(self).now().seconds_nanoseconds()[0]
    def trajToTalon(pt: JointTrajectoryPoint):
        pt.positions

    def baseToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def diff1ToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def diff2ToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def elbowToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

    def wristToTraj(self, status: MotorStatus):
        self.expectedTraj #math to convert status to traj point

def main(args=None):
    rclpy.init(args=args)
    node = trajectoryInterpreter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
