#ifndef JOYSTICK_CONTROL__SCIENCEMODE_HPP_
#define JOYSTICK_CONTROL__SCIENCEMODE_HPP_

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

class ScienceMode : public Mode {
	public:

		ScienceMode(rclccp::Node* node);

		void processJoystickInput(std:shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

		static void declareParameters(rclcpp::Node* node);

	private:
		void handlePlatform(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
		void handleDrill(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
		void handleMicroscope(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
		void handlePanoramic(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;
		void handleSoilCollection(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

		void loadParameters();

		int8_t kPlatformAxis;
		int8_t kDrillButton;
		int8_t kMicroscopeAxis;

		bool kDrillState;

		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
			platform_pub_;
		rclcpp::Publisher<Bool>::SharedPtr
			drill_pub_;
		rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
			microscope_pub_;
};

#endif
