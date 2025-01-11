#include "typing_controller.h"

TypingNode::TypingNode() : Node("Typing_controller")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>("typing_topic", 10, std::bind(&TypingNode::topic_callback, this, std::placeholders::_1));
  subscription2_ = this->create_subscription<moveit_msgs::action::ExecuteTrajectory_FeedbackMessage>("execute_trajectory/_action/feedback", 10, std::bind(&TypingNode::arm_callback, this, std::placeholders::_1));
}

void TypingNode::topic_callback(const std_msgs::msg::String &msg)
{
  RCLCPP_INFO(this->get_logger(), "I got %s", msg.data.c_str());
}

void TypingNode::arm_callback(const moveit_msgs::action::ExecuteTrajectory_FeedbackMessage &msg)
{
  RCLCPP_INFO(this->get_logger(), "I got %s", msg.feedback.state.c_str()); //"IDLE" for when it's done
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TypingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
