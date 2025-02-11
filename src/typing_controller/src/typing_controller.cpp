#include "typing_controller.h"

TypingNode::TypingNode() : Node("Typing_controller")
{
  subscription_ = this->create_subscription<std_msgs::msg::String>("typing_topic", 10, std::bind(&TypingNode::topic_callback, this, std::placeholders::_1));
  subscription2_ = this->create_subscription<moveit_msgs::action::ExecuteTrajectory_FeedbackMessage>("execute_trajectory/_action/feedback", 10, std::bind(&TypingNode::arm_callback, this, std::placeholders::_1));
  
  //tf client
  node = rclcpp::Node::make_shared("get_angle_client");
  client = node->create_client<interfaces::srv::KeycapCmd>("keycap_cmd");
  
  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service active");
}

void TypingNode::topic_callback(const std_msgs::msg::String &msg)
{
  RCLCPP_INFO(this->get_logger(), "I got %s", msg.data.c_str());
  auto request = std::make_shared<interfaces::srv::KeycapCmd::Request>();
    request->key = "a";
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto resultCopy = result.get();
      RCLCPP_INFO(this->get_logger(), "Centimeter: %f, %f, %f", resultCopy->x, resultCopy->y, resultCopy->z);
    }
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
