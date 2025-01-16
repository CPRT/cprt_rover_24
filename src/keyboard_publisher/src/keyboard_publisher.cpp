#include "keyboard_publisher.h"

using namespace std::chrono_literals;

KeyboardPublisher::KeyboardPublisher()
: Node("minimal_publisher"), count_(0)
{
  publisher_ = this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&KeyboardPublisher::timer_callback, this));
  std::cout<<"Type w, a, s, d to move. Use zxrtfgcv to change orientation. Type 'h' to change step size (default is 10 rviz units). Type 'n' to reset. Type 'm' to open/close gripper. Type 'b' to plan to the orange arm in rviz."<<std::endl;
}

void KeyboardPublisher::timer_callback()
{
  interfaces::msg::ArmCmd poseCmd;
  poseCmd.pose.position.x = 0;
  poseCmd.pose.position.y = 0;
  poseCmd.pose.position.z = 0;
  poseCmd.pose.orientation.x = 0;
  poseCmd.pose.orientation.y = 0;
  poseCmd.pose.orientation.z = 0;
  poseCmd.pose.orientation.w = 0;
  poseCmd.speed = defSpeed;
  poseCmd.named_pose = 0;
  poseCmd.estop = false;
  poseCmd.reset = false;
  poseCmd.query_goal_state = false;
  char c;
  std::cin>>c;
  if (c == 'w')
  {
    poseCmd.pose.position.x = 1;
  }
  else if (c == 's')
  {
    poseCmd.pose.position.x = -1;
  }
  else if (c == 'a')
  {
    poseCmd.pose.position.y = 1;
  }
  else if (c == 'd')
  {
    poseCmd.pose.position.y = -1;
  }
  else if (c == 'z')
  {
    poseCmd.pose.position.z = 1;
  }
  else if (c == 'x')
  {
    poseCmd.pose.position.z = -1;
  }
  else if (c == 'r')
  {
    poseCmd.pose.orientation.x = 1;
  }
  else if (c == 't')
  {
    poseCmd.pose.orientation.x = -1;
  }
  else if (c == 'f')
  {
    poseCmd.pose.orientation.y = 1;
  }
  else if (c == 'g')
  {
    poseCmd.pose.orientation.y = -1;
  }
  else if (c == 'c')
  {
    poseCmd.pose.orientation.z = 1;
  }
  else if (c == 'v')
  {
    poseCmd.pose.orientation.z = -1;
  }
  else if (c == 'h')
  {
    double newSpeed = 0;
    std::cin>>newSpeed;
    defSpeed = newSpeed;
  }
  else if (c == 'n')
  {
    poseCmd.reset = true;
  }
  else if (c == 'm')
  {
    isOpen = !isOpen;
    poseCmd.named_pose = 1+isOpen;
  }
  else if (c == 'b')
  {
    poseCmd.query_goal_state = true;
    poseCmd.goal_angles.resize(6, 0);
    double d = 0;
    std::cin>>d;
    poseCmd.goal_angles[0] = d/360.0 * 2*M_PI;
    std::cin>>d;
    poseCmd.goal_angles[1] = d/360.0 * 2*M_PI;
    std::cin>>d;
    poseCmd.goal_angles[2] = d/360.0 * 2*M_PI;
    std::cin>>d;
    poseCmd.goal_angles[3] = d/360.0 * 2*M_PI;
    std::cin>>d;
    poseCmd.goal_angles[4] = d/360.0 * 2*M_PI;
    std::cin>>d;
    poseCmd.goal_angles[5] = d/360.0 * 2*M_PI;
    for (unsigned long int i = 0; i < poseCmd.goal_angles.size(); i++)
    {
      RCLCPP_INFO(this->get_logger(), "Goal angle [%d]: %f", i, poseCmd.goal_angles[i]);
    }
  }
  
  //not sure what this part does, but it might be important
  geometry_msgs::msg::Pose current_pose;
  current_pose.orientation.w = 1.0;
  current_pose.position.x = 0.636922;
  current_pose.position.y = 0.064768;
  current_pose.position.z = 0.678810;
  poseCmd.current_pose = current_pose;
  
  publisher_->publish(poseCmd);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeyboardPublisher>());
  
  rclcpp::shutdown();
  return 0;
}
