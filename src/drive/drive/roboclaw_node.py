#!/usr/bin/env python
import diagnostic_updater
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float64, Bool

from . import roboclaw_driver as roboclaw
from . import utils as u
from .electrical_wrapper import ElectricalWrapper
from .encoder_wrapper import EncoderWrapper


class Movement:
    """Movement class - responsible of running the RoboClaw"""

    def __init__(
        self,
        address,
        max_speed,
        base_width,
        ticks_per_meter,
        ticks_per_rotation,
        parent_clock,
        parent_logger,
    ):
        self.twist = None
        self.address = address
        self.MAX_SPEED = max_speed
        self.BASE_WIDTH = base_width
        self.TICKS_PER_METER = ticks_per_meter
        self.TICKS_PER_ROTATION = ticks_per_rotation
        self.clock = parent_clock
        self.logger = parent_logger
        self.last_set_speed_time = self.clock.now().nanoseconds
        self.vr_ticks = 0
        self.vl_ticks = 0
        self.stopped = True

    def run(self):  # DRIVE LOGIC
        if self.stopped is True:
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                self.logger.warn("Stopping error: " + str(e.errno))
                self.logger.debug(e)
            return
        if self.twist is None:
            return
        if self.twist.linear.x != 0 or self.twist.angular.z != 0:
            self.stopped = False
        if self.twist is None or self.stopped is True:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
            return

        linear_x = self.twist.linear.x
        if linear_x > self.MAX_SPEED:  # check for messages above speed limit
            linear_x = self.MAX_SPEED
        if linear_x < -self.MAX_SPEED:
            linear_x = -self.MAX_SPEED

        # vr and vl are how fast the velocity on the left and right side is in m/s
        vr = linear_x - self.twist.angular.z * self.BASE_WIDTH / 2  # m/s
        vl = linear_x + self.twist.angular.z * self.BASE_WIDTH / 2
        self.twist = None

        # ticks convert the speed the wheel needs to go into encoder ticks per second
        vr_ticks = int(vr * self.TICKS_PER_METER)  # ticks/s
        vl_ticks = int(vl * self.TICKS_PER_METER)

        self.logger.debug("vr_ticks: " + str(vr_ticks) + "vl_ticks: " + str(vl_ticks))

        try:
            # This is a hack way to keep a poorly tuned PID from making noise at speed 0
            if vr_ticks == 0 and vl_ticks == 0:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            ####### PID DRIVE #######
            else:
                roboclaw.SpeedM1M2(self.address, -vr_ticks, vl_ticks)
            self.logger.info(
                "tryng PID vr = " + str(-vr_ticks) + " vl = " + str(vl_ticks)
            )

            ####### VOLTAGE DRIVE #######
            # else:
            #   dutyCycle1 = int(vr / 12 * 32767) #mainBatteryVoltage * 32767
            #   dutyCycle2 = int(vl / 12 * 32767)
            #   roboclaw.DutyM1M2(self.address, -dutyCycle1, dutyCycle2)
        except OSError as e:
            self.logger.warn("SpeedM1M2 OSError: " + str(e.errno))
            self.logger.debug(e)


class RoboclawNode(Node):
    """RoboClaw Node"""

    def __init__(self):
        super().__init__("roboclaw_node")
        self.ERRORS = u.ROBOCLAW_ERRORS

        freq = 30
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.run)

        self.get_logger().info("Connecting to roboclaw")

        # initialzie default connection params
        self.declare_parameter("dev", "/dev/ttyACM0")
        dev_name = self.get_parameter("dev").get_parameter_value().string_value
        self.get_logger().info(dev_name)

        self.declare_parameter("baud", 115200)
        baud_rate = self.get_parameter("baud").get_parameter_value().integer_value

        self.declare_parameter("address", 128)
        self.address = self.get_parameter("address").get_parameter_value().integer_value
        if self.address > 0x87 or self.address < 0x80:
            self.get_logger().fatal("Address out of range")
            self.shutdown("Address out of range")

        # TODO need someway to check if address is correct
        try:
            roboclaw.Open(dev_name, baud_rate)
        except Exception as e:
            self.get_logger().fatal("Could not connect to RoboClaw")
            self.get_logger().debug(e)
            self.shutdown("Could not connect to RoboClaw")

        self.updater = diagnostic_updater.Updater(self)
        self.updater.setHardwareID("RoboClaw")
        self.updater.add(
            diagnostic_updater.FunctionDiagnosticTask("Vitals", self.check_vitals)
        )
        try:
            version = roboclaw.ReadVersion(self.address)
        except Exception as e:
            self.get_logger().warn("Problem getting RoboClaw version")
            self.get_logger().warn(e)
            pass

        if not version[0]:
            self.get_logger().warn("Could not get version from RoboClaw")
        else:
            self.get_logger().debug(repr(version[1]))

        roboclaw.SpeedM1M2(self.address, 0, 0)  # set roboclaws to default
        roboclaw.ResetEncoders(self.address)

        # Get launch parameters
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

        self.encodm = None
        self.electr = None
        if self.PUB_ELEC:
            self.left_elec_pub = self.create_publisher(
                BatteryState, dev_name + "/roboclaw/elec/left", 1
            )
            self.right_elec_pub = self.create_publisher(
                BatteryState, dev_name + "/roboclaw/elec/right", 1
            )
            self.electr = ElectricalWrapper(self)

        if self.PUB_ODOM:  # publish odometry
            self.odom_pub = self.create_publisher(
                Odometry, dev_name + "/odom_roboclaw", 1
            )
            self.left_encoder_pub = self.create_publisher(
                Float64, dev_name + "/left_encoder_angular_velocity", 1
            )
            self.right_encoder_pub = self.create_publisher(
                Float64, dev_name + "/right_encoder_angular_velocity", 1
            )
            self.encodm = EncoderWrapper(
                self.TICKS_PER_METER,
                self.TICKS_PER_ROTATION,
                self.BASE_WIDTH,
                self,
            )
        self.movement = Movement(  # create movement class with parameters
            self.address,
            self.MAX_SPEED,
            self.BASE_WIDTH,
            self.TICKS_PER_METER,
            self.TICKS_PER_ROTATION,
            self.get_clock(),
            self.get_logger(),
        )
        self.last_set_speed_time = self.get_clock().now().nanoseconds
        # subscriptions
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/drive/cmd_vel", self.cmd_vel_callback, 1
        )
        self.cmd_estop_sub = self.create_subscription(
            Bool, "/drive/estop", self.cmd_estop_callback, 1
        )
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=1.0))

    def run(self):  # Roboclaw run loop
        # stop movement if robot doesn't recieve commands for 1 sec
        if (
            self.STOP_MOVEMENT
            and not self.movement.stopped
            and self.get_clock().now().nanoseconds - self.movement.last_set_speed_time
            > 10e9
        ):
            self.get_logger().info("Did not get command for 1 second, stopping")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                self.get_logger().error("Could not stop")
                self.get_logger().debug(e)
            self.movement.stopped = True

        # TODO need find solution to the OSError11 looks like sync problem with serial
        # Read encoders
        status1, enc1, crc1 = None, None, None
        status2, enc2, crc2 = None, None, None

        # log and publish
        try:
            status1, enc1, crc1 = roboclaw.ReadEncM1(self.address)
        except ValueError:
            pass
        except OSError as e:
            self.get_logger().warn("ReadEncM1 OSError: " + str(e.errno))
            self.get_logger().debug(e)

        try:
            status2, enc2, crc2 = roboclaw.ReadEncM2(self.address)
        except ValueError:
            pass
        except OSError as e:
            self.get_logger().warn("ReadEncM2 OSError: " + str(e.errno))
            self.get_logger().debug(e)

        if self.PUB_ELEC:
            try:
                elec_data = self.poll_elec()
            except ValueError:
                pass
            except OSError as e:
                self.get_logger().warn("Electrical OSError: " + str(e.errno))
                self.get_logger().debug(e)

        has_enc1 = "enc1" in vars()
        has_enc2 = "enc2" in vars()
        has_encoders = has_enc1 and has_enc2 and enc1 and enc2

        publish_time = self.get_clock().now()

        if has_encoders:
            self.get_logger().debug(" Encoders " + str(enc1) + " " + str(enc2))
            if self.encodm:
                # Left motor encoder : M2 / Right motor encoder : M1
                self.encodm.update_n_publish(enc2, enc1, publish_time)
                # self.encodm.update_n_publish(enc1, enc2)
            self.updater.update()

        if self.electr:
            self.electr.publish_elec(publish_time, elec_data)

        # self.get_logger().info("Update done moving if cmd")

        self.movement.run()  # run movement loop

    def poll_elec(self) -> dict:
        """Read motors electrical data"""
        elec_data = {}

        status, *currents = roboclaw.ReadCurrents(self.address)
        self.log_elec(status, "currents")
        elec_data["current"] = {
            sd: curr / 100 for sd, curr in zip(("left", "right"), currents)
        }

        status, voltage = roboclaw.ReadMainBatteryVoltage(self.address)
        self.log_elec(status, "voltages")
        elec_data["voltage"] = voltage / 10

        status, logicbatt = roboclaw.ReadLogicBatteryVoltage(self.address)
        self.log_elec(status, "logic voltages")
        elec_data["logicbatt"] = logicbatt / 10

        status, *pwms = roboclaw.ReadPWMs(self.address)
        self.log_elec(status, "PWMs")
        elec_data["pwm"] = {
            sd: pwm / 327.67 for sd, pwm in zip(("left", "right"), pwms)
        }

        status, temperature = roboclaw.ReadTemp(self.address)
        self.log_elec(status, "temperature")
        elec_data["temperature"] = temperature / 10

        return elec_data

    def log_elec(self, status: int, name: str):
        if status:
            # self.get_logger().info(f"Got {name.lower()} data")
            pass
        else:
            self.get_logger().info(f"Missing {name.lower()}")

    def cmd_vel_callback(self, twist):
        self.movement.last_set_speed_time = self.get_clock().now().nanoseconds
        self.movement.twist = twist

    def cmd_estop_callback(self, stopped):
        self.movement.stopped = stopped.data

    # TODO: Need to make this work when more than one error is raised
    def check_vitals(self, stat):
        try:
            status = roboclaw.ReadError(self.address)[1]
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: " + str(e.errno))
            self.get_logger().debug(e)
            return
        state, message = self.ERRORS[status]
        stat.summary(state, message)
        try:
            stat.add(
                "Main Batt V:",
                str(float(roboclaw.ReadMainBatteryVoltage(self.address)[1] / 10)),
            )
            stat.add(
                "Logic Batt V:",
                str(float(roboclaw.ReadLogicBatteryVoltage(self.address)[1] / 10)),
            )
            stat.add("Temp1 C:", str(float(roboclaw.ReadTemp(self.address)[1] / 10)))
            stat.add("Temp2 C:", str(float(roboclaw.ReadTemp2(self.address)[1] / 10)))
        except OSError as e:
            self.get_logger().warn("Diagnostics OSError: " + str(e.errno))
            self.get_logger().debug(e)
        return stat

    # TODO: need clean shutdown so motors stop even if new msgs are arriving
    def shutdown(self, str_msg):
        self.get_logger().info("Shutting down :" + str_msg)
        try:
            roboclaw.ForwardM1(self.address, 0)
            roboclaw.ForwardM2(self.address, 0)
        except OSError:
            self.get_logger().error("Shutdown did not work trying again")
            try:
                roboclaw.ForwardM1(self.address, 0)
                roboclaw.ForwardM2(self.address, 0)
            except OSError as e:
                self.get_logger().error("Could not shutdown motors!!!!")
                self.get_logger().debug(e)


def main(args=None):
    rclpy.init(args=args)
    roboclaw_node = RoboclawNode()
    rclpy.spin(roboclaw_node)
    roboclaw_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
