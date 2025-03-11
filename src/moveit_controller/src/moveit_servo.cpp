#include "moveit_servo.h"

using std::placeholders::_1;
using moveit::planning_interface::MoveGroupInterface;

ServoNode::ServoNode(const rclcpp::NodeOptions &options)
: Node("servo_controller_node", options)//, node_ptr(std::make_shared<rclcpp::Node>("example_moveit", options)), executor_ptr(std::make_shared<rclcpp::executors::SingleThreadedExecutor>()), tf_buffer(std::make_shared<tf2_ros::Buffer>(node_ptr->get_clock())), planning_scene(std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_ptr, "robot_description", tf_buffer, "planning_scene_monitor"))
{
  /*RCLCPP_INFO(this->get_logger(), "Starting!");
  executor_ptr->add_node(node_ptr);
  executor_thread = std::thread([this]() {this->executor_ptr->spin(); });
  twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("servo_demo_node/delta_twist_cmds", 10);
  
  joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        JOY_TOPIC, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::Joy::ConstSharedPtr& msg) { return joyCB(msg); });
  */
  
  twist_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("servo_node/delta_twist_cmds", 1);
  
  RCLCPP_INFO(this->get_logger(), "Starting!!!");
  servo_start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");
  servo_start_client_->wait_for_service(std::chrono::seconds(1));
  auto result = servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
  {
    auto resultCopy = result.get();
    RCLCPP_INFO(this->get_logger(), "Done starting");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service");
  }
  
  timer_ = this->create_wall_timer(110ms, std::bind(&ServoNode::move, this));//*/
  subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 1, std::bind(&ServoNode::topic_callback, this, _1));
}

void ServoNode::move()
{
  /*auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  //twist_msg->twist.linear.z = 1;
  char c;
  std::cin>>c;
  if (c == 'w')
  {
    twist_msg->twist.linear.x = 1000;
  }
  else if (c == 's')
  {
    twist_msg->twist.linear.x = -1000;
  }
  
  
  twist_msg->header.frame_id = "Link_7";
  twist_msg->header.stamp = this->now();
  twist_cmd_pub_->publish(std::move(twist_msg));*/
  
  
  if (moved)
  {
		auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
		*twist_msg = cmd;
		twist_msg->header.frame_id = "Link_7";
		twist_msg->header.stamp = this->now();
		twist_cmd_pub_->publish(std::move(twist_msg));
		moved = false;
	}
  
}

void ServoNode::topic_callback(
    const sensor_msgs::msg::Joy::SharedPtr msg) {
  auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
  //twist_msg->twist.linear.z = 1;
  
  twist_msg->header.frame_id = "Link_7";
  twist_msg->header.stamp = this->now();
  moved = false;
  cmd.twist.angular.x = 0;
  cmd.twist.angular.y = 0;
  cmd.twist.angular.z = 0;
  cmd.twist.linear.x = 0;
  cmd.twist.linear.y = 0;
  cmd.twist.linear.z = 0;
  
  if (msg->axes[4] > 0.1)  // thumbstick forward: + Roll
  {
    //moved = true;
    twist_msg->twist.angular.x = -1;
  }
  if (msg->axes[4] < -0.1)  // thumbstick backwards: - Roll
  {
    //moved = true;
    twist_msg->twist.angular.x = 1;
  }
  if (msg->axes[5] > 0.1)  // Joystick left: + pitch
  {
    //moved = true;
    twist_msg->twist.angular.y = 1;
  }
  if (msg->axes[5] < -0.1)  // Joystick right: - pitch
  {
    //moved = true;
    twist_msg->twist.angular.y = -1;
  }
  if (msg->axes[2] > 0.1)  // Joystick rotate left: + yaw
  {
    //moved = true;
    twist_msg->twist.angular.z = 1;
  }
  if (msg->axes[2] < -0.1)  // Joystick rotate right: - yaw
  {
    //moved = true;
    twist_msg->twist.angular.z = -1;
  }
  if (msg->axes[1] > 0.1)  // Joystick forward: Translate up
  {
    moved = true;
    twist_msg->twist.linear.z = 1;
    cmd.twist.linear.z = 10;
  }
  if (msg->axes[1] < -0.1)  // Joystick backwards: Translate down
  {
    moved = true;
    twist_msg->twist.linear.z = -1;
    cmd.twist.linear.z = -10;
  }
  if (msg->axes[0] > 0.1)  // Joystick left: Translate left
  {
    moved = true;
    twist_msg->twist.linear.y = 1;
    cmd.twist.linear.y = 10;
  }
  if (msg->axes[0] < -0.1)  // Joystick right: Translate right
  {
    moved = true;
    twist_msg->twist.linear.y = -1;
    cmd.twist.linear.y = -10;
  }
  if (msg->buttons[0] == 1)  // Joystick forward: Translate forward
  {
    moved = true;
    twist_msg->twist.linear.x = 1;
    cmd.twist.linear.x = 10;
  }
  if (msg->buttons[1] == 1)  // Joystick backward: Translate backwards
  {
    moved = true;
    twist_msg->twist.linear.x = -1;
    cmd.twist.linear.x = -10;
  }
  
  
  if (moved)
  {
    //RCLCPP_INFO(this->get_logger(), "Starting!!!");
  	//twist_cmd_pub_->publish(std::move(twist_msg));
  	//std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
}

static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit2_tutorials.servo_demo_node.cpp");

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<ServoNode>(options);
  rclcpp::spin(node);
}

