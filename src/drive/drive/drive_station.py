from math import radians
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8, Float32
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist

from interfaces.msg import LiveTune
from interfaces.msg import SixFloats
from interfaces.msg import PIDmsg

class driveStation(Node):
    def __init__(self):
        super().__init__("drivestation")
        self.liveTunePub = self.create_publisher(
                LiveTune, "drive_liveTune", 10)
        self.statePub = self.create_publisher(
            Int8, "cmd_drive_state", 10)

        self.cmddrivePub = self.create_publisher(
            SixFloats, "cmd_move", 10)

        self.setVoltsPub = self.create_publisher(
            SixFloats, "cmd_voltage_drive", 10)

        self.setTwistPub = self.create_publisher(
            Twist, "/cmd_vel", 10)

        self.setPid = self.create_publisher(
            PIDmsg, "pid_update", 10)

        self.create_timer(0.2, self.loopRun)
    def loopRun(self):
        print("Enter command: ")
        u = input("Enter command: ").lower().split(' ')

        if (u[0] == "t"):
            twist = Twist()
            twist.linear.x = float(u[1])
            twist.angular.z = float(u[2])
            self.setTwistPub.publish(twist)
            return 
        elif(u[0] == 'v'):
            volts = SixFloats()
            volts.m0 = float(u[1])
            volts.m1 = float(u[2])
            volts.m2 = float(u[3])
            volts.m3 = float(u[4])
            volts.m4 = float(u[5])
            volts.m5 = float(u[6])
            self.setVoltsPub.publish(volts)
            return
        
        elif(u[0] == 'p'):
            pid = PIDmsg()
            pid.p = float(u[1])
            pid.i = float(u[2])
            pid.d = float(u[3])
            self.setPid.publish(pid)
            return

        elif (u[0] == 'y'):
            self.get_logger().info("Turning PID on")
            temp = Int8()
            temp.data = 1
            self.statePub.publish(temp)
            return

        elif (u[0] == 'n'):
            self.get_logger().info("Turning PID off")
            temp = Int8()
            temp.data = 0
            self.statePub.publish(temp)
            return

        elif(u[0] == "stop"):
            volts = SixFloats()
            volts.m0 = 0
            volts.m1 = 0
            volts.m2 = 0
            volts.m3 = 0
            volts.m4 = 0
            volts.m5 = 0
            self.setVoltsPub.publish(volts)
            return
        



def main(args=None):
    rclpy.init(args=args)
    node = driveStation()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()
