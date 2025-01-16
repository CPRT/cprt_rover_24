from rclpy.time import Time
from sensor_msgs.msg import BatteryState


class ElectricalWrapper:
    def __init__(self, parent_node):
        """Roboclaw Electrical Data

        Args:
            parent_node (rclpy.node.Node): RCL Node
        """
        # Node parameters
        self.parent_node = parent_node
        self.clock = self.parent_node.get_clock()
        self.logger = self.parent_node.get_logger()
        # Electricity publishers
        self.left_elec_pub = self.parent_node.left_elec_pub
        self.right_elec_pub = self.parent_node.right_elec_pub
        # Electrical data
        self.motor_elec = {}

    def publish_elec(self, cur_time: Time, elec_data: dict):
        """Publish elec in two battery states messages

        Main Battery Voltage => voltage
        Current => current
        Logic Battery Voltage => charge
        PWM => percentage
        Temperature => temperature

        Args:
            cur_time (Time): Current time
            elec_data: Electrical data from the RoboClaw
        """

        side = "left"
        bs = BatteryState()
        bs.header.stamp = cur_time.to_msg()
        bs.header.frame_id = "base_link"
        bs.voltage = elec_data["voltage"]
        bs.current = elec_data["current"][side]
        bs.percentage = elec_data["pwm"][side]
        bs.charge = elec_data["logicbatt"]
        bs.temperature = elec_data["temperature"]
        self.left_elec_pub.publish(bs)

        side = "right"
        bs = BatteryState()
        bs.header.stamp = cur_time.to_msg()
        bs.header.frame_id = "base_link"
        bs.voltage = elec_data["voltage"]
        bs.current = elec_data["current"][side]
        bs.percentage = elec_data["pwm"][side]
        bs.charge = elec_data["logicbatt"]
        bs.temperature = elec_data["temperature"]
        self.right_elec_pub.publish(bs)
