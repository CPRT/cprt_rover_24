#include "ScienceMode.hpp"


ScienceMode::ScienceMode(rclccp::Node* node) : Mode("Science", node) {
	RCLCPP_INFO(node_->get_logger(), "Science Mode");
	loadParameters();
	platform_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
			"/science_plat_vel", 10);
	drill_pub_ = node_->create_publisher<Bool>(
			"/drill/set", 10); //TODO: create drill launch file
	microscope_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(
			"/microscope_vel", 10);
	//TODO:
	//panoramic_pub_ = node_->create_publisher<Bool?>("/science_panoramic", ?);
	//soil_collection_pub_ = node_->create_publisher<???>("/science_soil", ?);
}

void ScienceMode::processJoystickInput(std:shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
	handlePlatform(joystickMsg);
	handleDrill(joystickMsg);
	handleMicroscope(joystickMsg);
	//handlePanoramic(joystickMsg);
	//handleSoilCollection(joystickMsg);
}

void ScienceMode::handlePlatform(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
	//Process input and output linear component
	double value = joystickMsg->axes[kPlatformAxis];
	auto twist = geometry_msgs::msg::Twist();
	twist.linear.x = value;
	platform_pub_->publish(twist);

}

void ScienceMode::handleDrill(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
	//Turn it on and off
	int drill_value = joystickMsg->buttons[kDrillToggle];
	if(drill_value == true) {
		kDrillState = 1;
	} else {
		kDrillState = 0;
	}
	drill_pub_->publish(kDrillState);
}

void ScienceMode::handleMicroscope(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
	//Process input and output linear component
	double value = joystickMsg->axes[kMicroscopeAxis];
	auto twist = geometry_msgs::msg::Twist();
	twist.linear.x = value;
	microscope_pub_->publish(twist);
}

void ScienceMode::handlePanoramic(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
	//TODO
}

void ScienceMode::handleSoilCollection(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) {
	//TODO 
}

void ScienceMode::declareParameters(rclcpp::Node* node) {
	node->declare_parameter("science_mode.platform_axis", 1);
	node->declare_parameter("science_mode.drill_button", 0);
	node->declare_parameter("science_mode.microscope_axis", 5);
	node->declare_parameter("science_mode.drill_state", false);
	//node->declare_parameter("science_mode.panoramic_button", ?);
	//node->declare_parameter("science_mode.soil_collection_button, ?);
}

void ScienceMode::loadParameters() {
	node_->get_parameter("science_mode.platform_axis", kPlatformAxis);
	node_->get_parameter("science_mode.drill_button", kDrillButton);
	node_->get_parameter("science_mode.microscope_axis", kMicroscopeAxis);
	node_->get_parameter("science_mode.drill_state", kDrillState);
}


