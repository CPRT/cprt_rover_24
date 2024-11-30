#include "keyboard_publisher.h"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

MinimalPublisher::MinimalPublisher()
: Node("minimal_publisher"), count_(0)
{
  publisher_ = this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 10);
  timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));//*/
  std::cout<<"Type w, a, s, d to move. Use zxrtfgcv to change orientation. Type 'h' to change step size (default is 10 rviz units). Type 'n' to reset. Type 'm' to open/close gripper. Type 'b' to plan to the orange arm in rviz."<<std::endl;
}

void MinimalPublisher::timer_callback()
{
  interfaces::msg::ArmCmd poseCmd = [&]{
		interfaces::msg::ArmCmd msg;
		msg.pose.position.x = 0;
		msg.pose.position.y = 0;
		msg.pose.position.z = 0;
		msg.pose.orientation.x = 0;
		msg.pose.orientation.y = 0;
		msg.pose.orientation.z = 0;
		msg.pose.orientation.w = 0;
		msg.speed = defSpeed;
		msg.named_pose = 0;
		msg.estop = false;
		msg.reset = false;
		msg.query_goal_state = false;
		return msg;
	}();
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
    poseCmd.goal_angles[0] = d/360.0 * 2*3.14159;
    std::cin>>d;
    poseCmd.goal_angles[1] = d/360.0 * 2*3.14159;
    std::cin>>d;
    poseCmd.goal_angles[2] = d/360.0 * 2*3.14159;
    std::cin>>d;
    poseCmd.goal_angles[3] = d/360.0 * 2*3.14159;
    std::cin>>d;
    poseCmd.goal_angles[4] = d/360.0 * 2*3.14159;
    std::cin>>d;
    poseCmd.goal_angles[5] = d/360.0 * 2*3.14159;
    for (long unsigned int i = 0; i < poseCmd.goal_angles.size(); i++)
    {
      RCLCPP_INFO(this->get_logger(), std::to_string(poseCmd.goal_angles[i]).c_str());
    }
  }
  //auto message = std_msgs::msg::String();
  //geometry_msgs::msg::Pose message;
  //message.data = cmd;
  //RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  geometry_msgs::msg::Pose current_pose = []{
		geometry_msgs::msg::Pose msg;
		msg.orientation.w = 1.0;
		msg.position.x = 0.636922;
		msg.position.y = 0.064768;
		msg.position.z = 0.678810;
		return msg;
	}();
	poseCmd.current_pose = current_pose;
  publisher_->publish(poseCmd);
}


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  
  /*rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<MinimalPublisher>();
  executor.add_node(node);
  auto spinner = std::thread([&executor]() {executor.spin(); });
  spinner.join();*/
  rclcpp::shutdown();
  return 0;
}
