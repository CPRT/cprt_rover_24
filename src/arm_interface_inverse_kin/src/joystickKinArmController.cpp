#include "joystickKinArmController.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

bool isEqual(interfaces::msg::ArmCmd a, interfaces::msg::ArmCmd b) {
  return a == b;
}

JoystickReader::JoystickReader() : Node("JoystickReader") {
  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10, std::bind(&JoystickReader::topic_callback, this, _1));
  publisher_ =
      this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 2);
  timer_ = this->create_wall_timer(
      300ms, std::bind(&JoystickReader::publish_message, this));  //*/

  client_ = this->create_client<interfaces::srv::MoveServo>("servo_service");

  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(),
                "servo service not available, waiting again...");
  }
  sol_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/solenoid", 10);
  light_publisher_ = this->create_publisher<std_msgs::msg::Int8>("/light", 10);
}

void JoystickReader::topic_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  interfaces::msg::ArmCmd poseCmd = [] {
    interfaces::msg::ArmCmd msg;
    msg.pose.position.x = 0;
    msg.pose.position.y = 0;
    msg.pose.position.z = 0;
    msg.pose.orientation.x = 0;
    msg.pose.orientation.y = 0;
    msg.pose.orientation.z = 0;
    msg.pose.orientation.w = 0;
    msg.speed = 10;
    msg.estop = false;
    msg.reset = false;
    return msg;
  }();
  if (msg->axes[4] > 0.1)  // thumbstick forward: + Roll
  {
    poseCmd.pose.orientation.x = -1;
  }
  if (msg->axes[4] < -0.1)  // thumbstick backwards: - Roll
  {
    poseCmd.pose.orientation.x = 1;
  }
  if (msg->axes[5] > 0.1)  // Joystick left: + pitch
  {
    poseCmd.pose.orientation.y = 1;
  }
  if (msg->axes[5] < -0.1)  // Joystick right: - pitch
  {
    poseCmd.pose.orientation.y = -1;
  }
  if (msg->axes[2] > 0.1)  // Joystick rotate left: + yaw
  {
    poseCmd.pose.orientation.z = 1;
  }
  if (msg->axes[2] < -0.1)  // Joystick rotate right: - yaw
  {
    poseCmd.pose.orientation.z = -1;
  }
  if (msg->axes[1] > 0.1)  // Joystick forward: Translate up
  {
    poseCmd.pose.position.z = -1;
  }
  if (msg->axes[1] < -0.1)  // Joystick backwards: Translate down
  {
    poseCmd.pose.position.z = 1;
  }
  if (msg->axes[0] > 0.1)  // Joystick left: Translate left
  {
    poseCmd.pose.position.y = 1;
  }
  if (msg->axes[0] < -0.1)  // Joystick right: Translate right
  {
    poseCmd.pose.position.y = -1;
  }
  if (msg->buttons[0] == 1)  // Joystick forward: Translate forward
  {
    poseCmd.pose.position.x = 1;
  }
  if (msg->buttons[1] == 1)  // Joystick backward: Translate backwards
  {
    poseCmd.pose.position.x = -1;
  }

  if (msg->buttons[14] == 1) {
    servo_request(15, 8, 0, 180);  // end effector close
  }
  if (msg->buttons[15] == 1) {
    servo_request(15, 71, 0, 180);  // end effector open
  }
  if (msg->buttons[11])  // lights
  {
    std_msgs::msg::Int8 light_msg;
    light_msg.data = 1;
    light_publisher_->publish(light_msg);
  }
  if (msg->buttons[12])  // lights
  {
    std_msgs::msg::Int8 light_msg;
    light_msg.data = 2;
    light_publisher_->publish(light_msg);
  }
  if (msg->buttons[13])  // solenoid
  {
    std_msgs::msg::Bool sol_msg;
    sol_msg.data = false;
    sol_publisher_->publish(sol_msg);
  } else {
    std_msgs::msg::Bool sol_msg;
    sol_msg.data = true;
    sol_publisher_->publish(sol_msg);
  }

  if (!isEqual(poseCmd, oldCmd)) {
    oldCmd = poseCmd;
    shouldPub = true;
  }
}

void JoystickReader::publish_message() {
  if (shouldPub) {
    shouldPub = false;
    RCLCPP_INFO(this->get_logger(),
                "Current pose: %f %f %f %f %f %f %f %f %i %i",
                oldCmd.pose.position.x, oldCmd.pose.position.y,
                oldCmd.pose.position.z, oldCmd.pose.orientation.x,
                oldCmd.pose.orientation.y, oldCmd.pose.orientation.z,
                oldCmd.pose.orientation.w, oldCmd.speed, int(oldCmd.estop),
                int(oldCmd.reset));  //*/
    publisher_->publish(oldCmd);
  }
}

void JoystickReader::send_request(int port, int pos, int min, int max) {
  auto request = std::make_shared<interfaces::srv::MoveServo::Request>();
  request->port = port;
  request->pos = pos;
  request->min = min;
  request->max = max;

  auto future = client_->async_send_request(request);
}

void JoystickReader::servo_request(int req_port, int req_pos, int req_min,
                                   int req_max) {
  RCLCPP_INFO(this->get_logger(), "Sending Request for: %d", req_pos);
  send_request(req_port, req_pos, req_min, req_max);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JoystickReader>());
  rclcpp::shutdown();
  return 0;
}
