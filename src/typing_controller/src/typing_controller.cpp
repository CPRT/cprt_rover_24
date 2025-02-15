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
	poseCmd.named_pose = 0;
	poseCmd.estop = false;
	poseCmd.reset = false;
	poseCmd.query_goal_state = false;
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
    request->key = key[currLetter];
    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::FutureReturnCode::SUCCESS)
    {
      auto resultCopy = result.get();
      RCLCPP_INFO(this->get_logger(), "Centimeter: %f, %f, %f", resultCopy->x, resultCopy->y, resultCopy->z);
      cmd.x = resultCopy->x;
      cmd.y = resultCopy->y;
      cmd.z = resultCopy->z;
    }
}

void TypingNode::topic_callback(const std_msgs::msg::String &msg)
{
  RCLCPP_INFO(this->get_logger(), "I got %s", msg.data.c_str());
  key = msg.data;
  currDim = 0;
  currLetter = 0;
  adjusted = true;
  hasJob = true;
  
  getCmd();
  
  //make the arm move the first thing
  interfaces::msg::ArmCmd poseCmd;
  emptyCmd(poseCmd);
  poseCmd.speed = (cmd.x < 0) -0.2 : 0.2;
  poseCmd.pose.position.x = 1;
  publisher_->publish(poseCmd);
}

void TypingNode::arm_callback(const moveit_msgs::action::ExecuteTrajectory_FeedbackMessage &msg)
{
  RCLCPP_INFO(this->get_logger(), "I got %s", msg.feedback.state.c_str()); //"IDLE" for when it's done
  
  string data = msg.feedback.state;
  
  if (data == "IDLE" && hasJob) //idle death gambit? jjk reference??
  {
    //previous trajectory finished executing, now do next one
    if (!adjusted && !goingBack)
    {
      /*adjusted = true;
      interfaces::msg::ArmCmd poseCmd;
  		emptyCmd(poseCmd);
  		poseCmd.speed = 0.2;
  		poseCmd.pose.position.y = 1;
  		publisher_->publish(poseCmd);*/
  		interfaces::msg::ArmCmd poseCmd;
  		emptyCmd(poseCmd);
  		if (currDim == 0) //about to do x
  		{
  		  poseCmd.speed = (cmd.x < 0) -0.2 : 0.2;
  		  poseCmd.pose.position.x = 1
  		}
  		else if (currDim == 1) //about to do y
  		{
  		  poseCmd.speed = (cmd.y < 0) -0.2 : 0.2;
  		  poseCmd.pose.position.y = 1
  		}
  		else
  		{
  		  poseCmd.speed = (cmd.z < 0) -0.2 : 0.2;
  		  poseCmd.pose.position.z = 1
  		}
  		currDim++;
  		if (currDim >= 3)
  		{
  		  currDim = 0;
  		  if (goingBack)
  		  {
				  currLetter++;
				  if (currLetter >= key.size())
				  {
				    hasJob = false;
				  }
				}
				adjusted = true;
  		}
    }
    else //it adjusted itself already lol
    {
      if (!goingBack)
      {
      	getCmd(); //re-read the stuff
      }
      if (currDim == 0) //about to do x
  		{
  		  poseCmd.speed = cmd.x/100.0;
  		  poseCmd.pose.position.x = 1
  		}
  		else if (currDim == 1) //about to do y
  		{
  		  poseCmd.speed = cmd.y/100.0;
  		  poseCmd.pose.position.y = 1
  		}
  		else
  		{
  		  poseCmd.speed = cmd.z/100.0;
  		  poseCmd.pose.position.z = 1
  		}
  		
  		currDim++;
  		if (currDim >= 3)
  		{
  		  currDim = 0;
  		  if (goingBack)
  		  {
				  currLetter++;
				  if (currLetter >= key.size())
				  {
				    hasJob = false;
				  }
				}
				goingBack = !goingBack
  		}
      
    }
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
