from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import String

import rclpy.time
from std_msgs.msg import Int8, Float32, Bool
from interfaces.msg import SixFloats
from interfaces.srv import ArmPos
from interfaces.srv import ArmCmd
from trajectory_msgs.msg import JointTrajectoryPoint
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi

def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value

def elbow_rad_to_pos(rad):
    return (rad*8300*2000*13/16)/(2*3.14159)

def diff_rad_to_pos(diff1, diff2):
    diffCont2 = (diff2 - diff1)/2
    diffCont1 = diff2 - diffCont2
    diffCont2 = (diffCont2*8300*8000)/(2*3.14159)
    diffCont1 = (diffCont1*8300*8000)/(2*3.14159)
    return diffCont1, diffCont2


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
        
        self.srv2 = self.create_service(ArmCmd, 'arm_cmd', self.get_arm_cmd_callback)

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
        
        
        self.base = MotorControl()
        self.diff1 = MotorControl()
        self.diff2 = MotorControl()
        self.elbow = MotorControl()
        self.wristTilt = MotorControl()
        self.wristTurn = MotorControl()
        
        self.base.mode = 1;
        self.diff1.mode = 1;
        self.diff2.mode = 1;
        self.elbow.mode = 1;
        self.wristTilt.mode = 1;
        self.wristTurn.mode = 1;
        
        self.basePos = 0;
        self.diff1Pos = 0;
        self.diff2Pos = 0;
        self.elbowPos = 0;
        self.wristTiltPos = 0;
        self.wristTurnPos = 0;
        self.shouldPub = False
        
        self.baseCommand = self.create_publisher(
            MotorControl, "/base/set", 1)
        self.diff1Command = self.create_publisher(
            MotorControl, "/diff1/set", 1)
        self.diff2Command = self.create_publisher(
            MotorControl, "/diff2/set", 1)
        self.elbowCommand = self.create_publisher(
            MotorControl, "/elbow/set", 1)
        self.wristTiltCommand = self.create_publisher(
            MotorControl, "/wristTilt/set", 1)
        self.wristTurnCommand = self.create_publisher(
            MotorControl, "/wristTurn/set", 1)
        
        self.setEstop.publish(self.estop) #init as not estoped
        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.anglepublisher)
        self.timer2 = self.create_timer(period, self.controlPublisher)
        
        self.keyboard = self.create_subscription(
            String, "/keyboard_arm", self.keyboard_callback, 5)

    def base_callback(self, msg: MotorStatus): #14 - 58 (lower turns 58/14 times, upper turns once)
        self.baseAngle = 0.0
    def diff1_callback(self, msg: MotorStatus):
        #self.diff1Cont = self.diff1ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.diff1Cont = self.diff1ZeroPoint + msg.position * ((2*pi)/8000 * 1/83 * 1/100)
        self.diff1Angle = (self.diff1Cont - self.diff2Cont)
        self.diff1Pos = msg.position
        print("bruh1" + str(self.diff1Angle))
    def diff2_callback(self, msg: MotorStatus):
        #self.diff2Cont = self.diff2ZeroPoint + msg.position * ((2*pi)/1000 * 1/83 * 1/100)
        self.diff2Cont = self.diff2ZeroPoint + msg.position * ((2*pi)/8000 * 1/83 * 1/100) #diff drive: 83 100
        self.diff2Angle = (self.diff1Cont + self.diff2Cont)
        self.diff2Pos = msg.position
        print("bruh2" + str(self.diff2Angle))
    def elbow_callback(self, msg: MotorStatus):
        #self.elbowAngle = (self.elbowZeroPoint + msg.position * ((2*pi)/1000 * ((1/83) * (1/100) * (16/13)))) * 4
        self.elbowAngle = (self.elbowZeroPoint + msg.position * ((2*pi)/2000 * ((1/83) * (1/100) * (16/13))))
        self.elbowPos = msg.position
    def wrist_turn_callback(self, msg: MotorStatus):
        self.wristTurnAngle = 0.0
    def wrist_tilt_callback(self, msg: MotorStatus):
        self.wristTiltAngle = 0.0

    def anglepublisher(self):
        out = SixFloats()
        #out.m0 = self.elbowAngle 
        out.m0 = self.baseAngle 
        out.m1 = self.diff1Angle 
        out.m2 = self.diff2Angle
        out.m3 = self.elbowAngle
        self.m4 = self.wristTiltAngle
        self.m5 = self.wristTurnAngle
        self.anglePub.publish(out)
    
    def get_arm_pos_callback(self, request, response):
        #self.get_logger().info(f'Elbow: {self.elbowAngle}')
        #response.base = self.elbowAngle 
        response.base = self.baseAngle
        response.diff1 = self.diff1Angle 
        response.diff2 = self.diff2Angle
        response.elbow = self.elbowAngle
        response.wristturn = self.wristTurnAngle
        response.wristtilt = self.wristTiltAngle
        return response
    
    def get_arm_cmd_callback(self, request, response):
        #self.get_logger().info(f'Elbow: {self.elbowAngle}')
        #response.base = self.elbowAngle 
        #self.get_logger().info("I received something!");
        #time for elbow 90 degrees: 9.37 s
        #time for diff2 90 degrees: 31.15 s
        #time for 
        self.elbow.value = elbow_rad_to_pos(request.elbow);
        self.diff1.value, self.diff2.value = diff_rad_to_pos(request.diff1, request.diff2);
        #self.get_logger().info(f'Arm cmd: {request.base}, {request.diff1}, {request.diff2}, {request.elbow}, {request.wristtilt}, {request.wristturn}');
        if (self.shouldPub):
          self.get_logger().info(f'Elbow target: {self.elbow.value}, {self.elbowPos}, {abs(self.elbow.value - self.elbowPos)}, {self.elbowAngle}, {request.elbow}')
          if (abs(self.elbow.value - self.elbowPos) > 1000):
            self.elbowCommand.publish(self.elbow)
          if (abs(self.diff2.value - self.diff2Pos) > 1000 and abs(self.diff1.value - self.diff1Pos) > 1000):
            #pass
            self.diff2Command.publish(self.diff2)
            self.diff1Command.publish(self.diff1)
        response.success = True
        return response
        
    def controlPublisher(self):
        '''# if(Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2 or self.estop.data == True):
        #     return
        if (self.shouldPub):
          self.get_logger().info(f'Elbow target: {self.elbow.value}, {self.elbowPos}, {abs(self.elbow.value - self.elbowPos)}')
          #self.baseCommand.publish(self.base)
          if (abs(self.elbow.value - self.elbowPos) > 1000):
            self.elbowCommand.publish(self.elbow)
          if (abs(self.diff2.value - self.diff2Pos) > 1000 and abs(self.diff1.value - self.diff1Pos) > 1000):
            self.diff2Command.publish(self.diff2)
            self.diff1Command.publish(self.diff1)
          #self.wristTiltCommand.publish(self.wristTilt)
          #self.wristTurnCommand.publish(self.wristTurn)'''
    
    def keyboard_callback(self, msg):
        if msg.data == 'p': #elbow up
          self.shouldPub = not self.shouldPub

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
