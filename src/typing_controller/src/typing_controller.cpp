#include "typing_controller.h"

void emptyCmd(interfaces::msg::ArmCmd& poseCmd)
{
  poseCmd.pose.position.x = 0;
  poseCmd.pose.position.y = 0;
  poseCmd.pose.position.z = 0;
  poseCmd.pose.orientation.x = 0;
  poseCmd.pose.orientation.y = 0;
  poseCmd.pose.orientation.z = 0;
  poseCmd.pose.orientation.w = 0;
  poseCmd.speed = 0;
	//poseCmd.named_pose = 0;
	poseCmd.estop = false;
	poseCmd.reset = false;
	poseCmd.query_goal_state = false;
	poseCmd.reverse = false;
}

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
  
  //moveit_controller
  publisher_ = this->create_publisher<interfaces::msg::ArmCmd>("arm_base_commands", 10);
}

void TypingNode::getCmd(char k)
{
  auto request = std::make_shared<interfaces::srv::KeycapCmd::Request>();
    request->key = k;//key[currLetter];
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto resultCopy = result.get();
      RCLCPP_INFO(this->get_logger(), "Letter %lu, Centimeter: %f, %f, %f", currLetter, resultCopy->x, resultCopy->y, resultCopy->z);
      cmd.x = resultCopy->y;
      cmd.y = -resultCopy->x;
      cmd.z = resultCopy->z;
    }
}

void TypingNode::topic_callback(const std_msgs::msg::String &msg)
{
  RCLCPP_INFO(this->get_logger(), "I got %s", msg.data.c_str());
  key = msg.data;
  currDim = 0;
  currLetter = 0;
  adjusted = false;
  hasJob = true;
  
  getCmd(key[0]);
  
  //make the arm move the first thing
  interfaces::msg::ArmCmd poseCmd;
  emptyCmd(poseCmd);
  //poseCmd.speed = (cmd.x < 0) ? -0.02 : 0.02;
  //poseCmd.pose.position.x = 1;
  poseCmd.pose.position.x = cmd.x/100.0; //position commands are in meters, but tf2_keyboard returns cm
  poseCmd.pose.position.y = cmd.y/100.0;
  
  publisher_->publish(poseCmd);
}

void TypingNode::arm_callback(const moveit_msgs::action::ExecuteTrajectory_FeedbackMessage &msg)
{
  RCLCPP_INFO(this->get_logger(), "I got %s with job %i", msg.feedback.state.c_str(), hasJob); //"IDLE" for when it's done
  
  string data = msg.feedback.state;
  
  if (data == "Solution found but controller failed during execution")
  {
    data = "IDLE";
    adjusted = false;
  }
  
  if (data == "IDLE" && hasJob) //idle death gambit? jjk reference??
  {
    interfaces::msg::ArmCmd poseCmd;
    emptyCmd(poseCmd);
    
    if (!adjusted)
    {
  		getCmd(key[currLetter]); //refresh position
  	}
    
    RCLCPP_INFO(this->get_logger(), "%f %f %f %f %f %f", cmd.x, cmd.y, std::abs(double(cmd.x)),(std::abs(double(cmd.x)) < 0.08),std::abs(double(cmd.y)),(std::abs(double(cmd.y)) < 0.08));
    
    if (std::abs(double(cmd.x)) < 0.5 && std::abs(double(cmd.y)) < 0.5) //if within under a milimeter, and ready to press
    {
      if (!adjusted)
      {
        RCLCPP_INFO(this->get_logger(), "Aligning my 'end-effector' with the 'key'");
        adjusted = true;
        poseCmd.pose.position.x = TIP_DEPTH/100.0; //currently, camera aligned with key. Make claw aligned with key.
      }
      else
      {
        RCLCPP_INFO(this->get_logger(), "Pressing key");
        adjusted = false;
        currLetter++;
        if (currLetter >= key.size())
        {
          hasJob = false;
        }
        poseCmd.pose.position.z = -(cmd.z-TIP_LENGTH)/100.0;
        poseCmd.reverse = true;
      }
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "I'm straight adjusting it");
      poseCmd.pose.position.x = cmd.x/100.0; //keep adjusting pleb
			poseCmd.pose.position.y = cmd.y/100.0;
    }
    
    publisher_->publish(poseCmd);
  }
}

int main(int argc, char ** argv)
{
  // Initialize ROS and create the Node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TypingNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
