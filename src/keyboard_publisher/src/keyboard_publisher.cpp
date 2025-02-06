#include "keyboard_publisher.h"

using namespace std::chrono_literals;

KeyboardPublisher::KeyboardPublisher() : Node("minimal_publisher"), count_(0) {
  publisher_ =
      this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 10);
  timer_ = this->create_wall_timer(
      500ms, std::bind(&KeyboardPublisher::timer_callback, this));
  std::cout << "Type w, a, s, d to move. Use zxrtfgcv to change orientation. "
               "Type 'h' to change step size (default is 10 rviz units). "
               "Type 'o' to switch between global and local transformations. "
               "Type 'n' to reset. Type 'm' to open/close gripper. Type 'b' to plan "
               "to the orange arm in rviz."
            << std::endl;
}

void KeyboardPublisher::timer_callback() {
  interfaces::msg::ArmCmd poseCmd;
  poseCmd.pose.position.x = 0;
  poseCmd.pose.position.y = 0;
  poseCmd.pose.position.z = 0;
  poseCmd.pose.orientation.x = 0;
  poseCmd.pose.orientation.y = 0;
  poseCmd.pose.orientation.z = 0;
  poseCmd.pose.orientation.w = 0;
  poseCmd.speed = defSpeed;
  poseCmd.end_effector = interfaces::msg::ArmCmd::END_EFF_UNKNOWN;
  poseCmd.estop = false;
  poseCmd.reset = false;
  poseCmd.query_goal_state = false;
  char c;
  std::cin >> c;
  if (c == 'w') {
    poseCmd.pose.position.x = 1;
  } else if (c == 's') {
    poseCmd.pose.position.x = -1;
  } else if (c == 'a') {
    poseCmd.pose.position.y = 1;
  } else if (c == 'd') {
    poseCmd.pose.position.y = -1;
  } else if (c == 'z') {
    poseCmd.pose.position.z = 1;
  } else if (c == 'x') {
    poseCmd.pose.position.z = -1;
  } else if (c == 'r') {
    poseCmd.pose.orientation.x = 1;
  } else if (c == 't') {
    poseCmd.pose.orientation.x = -1;
  } else if (c == 'f') {
    poseCmd.pose.orientation.y = 1;
  } else if (c == 'g') {
    poseCmd.pose.orientation.y = -1;
  } else if (c == 'c') {
    poseCmd.pose.orientation.z = 1;
  } else if (c == 'v') {
    poseCmd.pose.orientation.z = -1;
  } else if (c == 'h') {
    double newSpeed = 0;
    std::cin >> newSpeed;
    defSpeed = newSpeed;
  } else if (c == 'o'){
    is_local_tf = !is_local_tf;
    if (is_local_tf){
    RCLCPP_INFO(this->get_logger(), "Local Transformations on");
  } else {
    RCLCPP_INFO(this->get_logger(), "Local Transformations off");
  }
  } else if (c == 'n') {
    poseCmd.reset = true;
  } else if (c == 'm') {
    isOpen = !isOpen;
    poseCmd.end_effector = isOpen ? interfaces::msg::ArmCmd::END_EFF_OPEN
                                  : interfaces::msg::ArmCmd::END_EFF_CLOSE;
  } else if (c == 'b') {
    poseCmd.query_goal_state = true;
    poseCmd.goal_angles.resize(6, 0);
    double d = 0;
    for (int i = 0; i < 6; i++) {
      if (!(std::cin >> d)) {
        std::cout << "Enter a real number please >:(" << std::endl;
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        return;
      }
      poseCmd.goal_angles[i] = d / 360.0 * 2 * M_PI;
    }
    for (unsigned long int i = 0; i < poseCmd.goal_angles.size(); i++) {
      RCLCPP_INFO(this->get_logger(), "Goal angle [%ld]: %f", i,
                  poseCmd.goal_angles[i]);
    }
  }
 
  poseCmd.is_local_tf = is_local_tf;
  publisher_->publish(poseCmd);
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardPublisher>());

  rclcpp::shutdown();
  return 0;
}
