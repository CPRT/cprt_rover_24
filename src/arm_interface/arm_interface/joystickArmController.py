from math import radians
import rclpy
import rclpy.logging
from rclpy.node import Node

import rclpy.time
from sensor_msgs.msg import Joy
from std_msgs.msg import Int8, Float32, Bool
from ros_phoenix.msg import MotorControl, MotorStatus
from math import pi
from interfaces.srv import MoveServo


def map_range(value, old_min, old_max, new_min, new_max):
    old_range = old_max - old_min
    new_range = new_max - new_min
    scaled_value = (value - old_min) / old_range
    mapped_value = new_min + scaled_value * new_range
    return mapped_value


def joystick_to_motor_control(vertical, horizontal):
    vertical = max(min(vertical, 1.0), -1.0)
    horizontal = max(min(horizontal, 1.0), -1.0)

    left_motor = vertical + horizontal
    right_motor = vertical - horizontal

    left_motor = max(min(left_motor, 1.0), -1.0)
    right_motor = max(min(right_motor, 1.0), -1.0)

    return -left_motor, -right_motor


class joystickArmController(Node):
    def __init__(self):
        super().__init__("joystickControl")

        # GPIO.setmode(GPIO.BOARD)
        # output_pins = {
        #     'JETSON_XAVIER': 18,
        #     'JETSON_NANO': 33,
        #     'JETSON_NX': 33,
        #     'CLARA_AGX_XAVIER': 18,
        #     'JETSON_TX2_NX': 32,
        #     'JETSON_ORIN': 18,
        #     'JETSON_ORIN_NX': 33,
        #     'JETSON_ORIN_NANO': 33
        # }
        # output_pin = output_pins.get(GPIO.model, None)
        # if output_pin is None:
        #     raise Exception('PWM not supported on this board')

        # GPIO.setup(output_pin, GPIO.OUT, initial=GPIO.HIGH)
        # self.gripper = GPIO.PWM(output_pin, 50)

        self.base = MotorControl()
        self.diff1 = MotorControl()
        self.diff2 = MotorControl()
        self.elbow = MotorControl()
        self.wristTilt = MotorControl()
        self.wristTurn = MotorControl()
        self.estop = Bool()
        self.estopTimestamp = 0.0
        self.lastTimestamp = 0

        self.toggleArmControl = False

        self.MAX_ACTUATION = 71
        self.MIN_ACTUATION = 8
        self.SERVO_MAX = 180
        self.SERVO_MIN = 0
        self.SERVO_PORT = 0
        self.ACTUATION_RATE = 5
        self.currPos = self.MAX_ACTUATION

        self.baseCommand = self.create_publisher(MotorControl, "/base/set", 1)
        self.diff1Command = self.create_publisher(MotorControl, "/diff1/set", 1)
        self.diff2Command = self.create_publisher(MotorControl, "/diff2/set", 1)
        self.elbowCommand = self.create_publisher(MotorControl, "/elbow/set", 1)
        self.wristTiltCommand = self.create_publisher(MotorControl, "/wristTilt/set", 1)
        self.wristTurnCommand = self.create_publisher(MotorControl, "/wristTurn/set", 1)

        self.joystick = self.create_subscription(Joy, "/joy", self.joy_callback, 5)

        self.cli = self.create_client(MoveServo, "servo_service")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

        self.servo_request(
            self.SERVO_PORT, self.MAX_ACTUATION, self.SERVO_MIN, self.SERVO_MAX
        )  # set initial position to fully open

        freq = 10
        self.rate = self.create_rate(freq)
        period = 1 / freq
        self.timer = self.create_timer(period, self.controlPublisher)

    def send_request(self, port: int, pos: int, min: int, max: int) -> MoveServo:
        req = MoveServo.Request()
        req.port = port
        req.pos = pos
        req.min = min
        req.max = max
        self.future = self.cli.call_async(req)
        return self.future.result()

    def servo_request(self, req_port, req_pos, req_min, req_max) -> None:
        self.send_request(port=req_port, pos=req_pos, min=req_min, max=req_max)

    def controlPublisher(self):
        if (
            Node.get_clock(self).now().seconds_nanoseconds()[0] - self.lastTimestamp > 2
            or self.estop.data == True
        ):
            return
        if self.toggleArmControl == True:
            self.baseCommand.publish(self.base)
            self.diff1Command.publish(self.diff1)
            self.diff2Command.publish(self.diff2)
            self.elbowCommand.publish(self.elbow)
            self.wristTiltCommand.publish(self.wristTilt)
            self.wristTurnCommand.publish(self.wristTurn)

    def joy_callback(self, msg: Joy):
        self.lastTimestamp = msg.header.stamp.sec
        self.base.mode = 0
        self.diff1.mode = 0
        self.diff2.mode = 0
        self.elbow.mode = 0
        self.wristTilt.mode = 0
        self.wristTurn.mode = 0

        if msg.buttons[13] == 1:  # toggle arm control
            self.toggleArmControl = not self.toggleArmControl

        if msg.axes[0] < -0.1:  # RIGHT BASE, TM AXIS-0 -
            self.base.value = -msg.axes[0]
        elif msg.axes[0] > 0.1:  # LEFT BASE, TM AXIS-0 +
            self.base.value = -msg.axes[0]
        else:
            self.base.value = 0.0

        if msg.axes[4] == -1:  # RIGHT WRIST TURN, TM AXIS-4 +
            self.wristTurn.value = 1.0
        elif msg.axes[4] == 1:  # LEFT WRIST TURN, TM AXIS-4 -
            self.wristTurn.value = -1.0
        else:
            self.wristTurn.value = 0.0

        if msg.buttons[2]:  # RIGHT WRIST TILT, TM BUTTON 2
            self.wristTilt.value = 1.0
        elif msg.buttons[3]:  # LEFT WRIST TILT, TM BUTTON 3
            self.wristTilt.value = -1.0
        else:
            self.wristTilt.value = 0.0

        if msg.axes[5] < -0.2:  # DIFF2 FORWARD, TM AXIS-0 -
            self.diff2.value = 1.0
        elif msg.axes[5] > 0.2:  # DIFF2 BACKWARD, TM AXIS-0 +
            self.diff2.value = -1.0
        else:
            self.diff2.value = 0.0

        # self.elbow.value = msg.axes[4]  # ELBOW ROTATION, TM AXIS-2 +/-

        if msg.axes[2] < -0.2:  # ELBOW LEFT, TM AXIS-0 +
            self.elbow.value = 0.2
        elif msg.axes[2] > 0.2:  # EBLOW RIGHT, TM AXIS-0 -
            self.elbow.value = -0.2
        else:
            self.elbow.value = 0.0

        self.diff1.value = -msg.axes[1]  # DIFF1 CONTROL, TM AXIS-1 +/-

        # old diff drive control for other arm
        #
        # diff1, diff2 = joystick_to_motor_control(msg.axes[0], msg.axes[1])
        # self.get_logger().info(f'diff1: {self.diff1.value}, diff2: {self.diff2.value}')
        # self.diff1.value = float(diff1)
        # self.diff2.value = float(diff2)

        if msg.buttons[15]:
            self.estop.data = True
            self.estopTimestamp = msg.header.stamp.sec
        if msg.buttons[14] and msg.header.stamp.sec - self.estopTimestamp > 2:
            self.estop.data = False

        if msg.buttons[0] == 1:
            if self.currPos > self.MIN_ACTUATION:
                self.currPos -= self.ACTUATION_RATE
                self.servo_request(
                    self.SERVO_PORT, self.currPos, self.SERVO_MIN, self.SERVO_MAX
                )

        if msg.buttons[1] == 1:
            if self.currPos < self.MAX_ACTUATION:
                self.currPos += self.ACTUATION_RATE
                self.servo_request(
                    self.SERVO_PORT, self.currPos, self.SERVO_MIN, self.SERVO_MAX
                )


def main(args=None):
    rclpy.init(args=args)
    node = joystickArmController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
