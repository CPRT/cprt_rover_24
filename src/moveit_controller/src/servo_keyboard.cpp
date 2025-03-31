#include "servo_keyboard.h"

KeyboardReader::KeyboardReader() : kfd(0) {
  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  struct termios raw;
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &= ~(ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);
}
void KeyboardReader::readOne(char* c) {
  int rc = read(kfd, c, 1);
  if (rc < 0) {
    throw std::runtime_error("read failed");
  }
}
void KeyboardReader::shutdown() { tcsetattr(kfd, TCSANOW, &cooked); }

KeyboardServo::KeyboardServo()
    : frame_to_publish_(BASE_FRAME_ID), joint_vel_cmd_(1.0) {
  nh_ = rclcpp::Node::make_shared("servo_keyboard_input");

  twist_pub_ = nh_->create_publisher<geometry_msgs::msg::TwistStamped>(
      TWIST_TOPIC, ROS_QUEUE_SIZE);
  joint_pub_ = nh_->create_publisher<control_msgs::msg::JointJog>(
      JOINT_TOPIC, ROS_QUEUE_SIZE);
}

void KeyboardServo::spin() {
  while (rclcpp::ok()) {
    rclcpp::spin_some(nh_);
  }
}

KeyboardServo::~KeyboardServo() { input.shutdown(); }

int KeyboardServo::keyLoop() {
  char c;
  bool publish_twist = false;
  bool publish_joint = false;

  std::thread{std::bind(&KeyboardServo::spin, this)}.detach();

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys and the '.' and ';' keys to Cartesian jog");
  puts(
      "Use 'W' to Cartesian jog in the world frame, and 'E' for the "
      "End-Effector frame");
  puts(
      "Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of "
      "jogging.");
  puts("'Q' to quit.");

  for (;;) {
    // get the next event from the keyboard
    try {
      input.readOne(&c);
    } catch (const std::runtime_error&) {
      RCLCPP_INFO(nh_->get_logger(), "Read error!");
      return -1;
    }

    RCLCPP_DEBUG(nh_->get_logger(), "value: 0x%02X\n", c);

    // // Create the messages we might publish
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Use read key-press
    switch (c) {
      case KEYCODE_LEFT:
        RCLCPP_DEBUG(nh_->get_logger(), "LEFT");
        twist_msg->twist.linear.y = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_RIGHT:
        RCLCPP_DEBUG(nh_->get_logger(), "RIGHT");
        twist_msg->twist.linear.y = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_UP:
        RCLCPP_DEBUG(nh_->get_logger(), "UP");
        twist_msg->twist.linear.x = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_DOWN:
        RCLCPP_DEBUG(nh_->get_logger(), "DOWN");
        twist_msg->twist.linear.x = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_PERIOD:
        RCLCPP_DEBUG(nh_->get_logger(), "PERIOD");
        twist_msg->twist.linear.z = -1.0;
        publish_twist = true;
        break;
      case KEYCODE_SEMICOLON:
        RCLCPP_DEBUG(nh_->get_logger(), "SEMICOLON");
        twist_msg->twist.linear.z = 1.0;
        publish_twist = true;
        break;
      case KEYCODE_E:
        RCLCPP_DEBUG(nh_->get_logger(), "E");
        frame_to_publish_ = EEF_FRAME_ID;
        break;
      case KEYCODE_W:
        RCLCPP_DEBUG(nh_->get_logger(), "W");
        frame_to_publish_ = BASE_FRAME_ID;
        break;
      case KEYCODE_1:
        RCLCPP_DEBUG(nh_->get_logger(), "1");
        joint_msg->joint_names.push_back("Joint_2");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_2:
        RCLCPP_DEBUG(nh_->get_logger(), "2");
        joint_msg->joint_names.push_back("Joint_2");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_3:
        RCLCPP_DEBUG(nh_->get_logger(), "3");
        joint_msg->joint_names.push_back("Joint_3");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_4:
        RCLCPP_DEBUG(nh_->get_logger(), "4");
        joint_msg->joint_names.push_back("Joint_4");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_5:
        RCLCPP_DEBUG(nh_->get_logger(), "5");
        joint_msg->joint_names.push_back("Joint_5");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_6:
        RCLCPP_DEBUG(nh_->get_logger(), "6");
        joint_msg->joint_names.push_back("Joint_6");
        joint_msg->velocities.push_back(joint_vel_cmd_);
        publish_joint = true;
        break;
      case KEYCODE_R:
        RCLCPP_DEBUG(nh_->get_logger(), "R");
        joint_vel_cmd_ *= -1;
        break;
      case KEYCODE_Q:
        RCLCPP_DEBUG(nh_->get_logger(), "quit");
        return 0;
    }

    // If a key requiring a publish was pressed, publish the message now
    if (publish_twist) {
      twist_msg->header.stamp = nh_->now();
      twist_msg->header.frame_id = frame_to_publish_;
      twist_pub_->publish(std::move(twist_msg));
      publish_twist = false;
    } else if (publish_joint) {
      joint_msg->header.stamp = nh_->now();
      joint_msg->header.frame_id = BASE_FRAME_ID;
      joint_pub_->publish(std::move(joint_msg));
      publish_joint = false;
    }
  }

  return 0;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  KeyboardServo keyboard_servo;

  int rc = keyboard_servo.keyLoop();
  rclcpp::shutdown();

  return rc;
}
