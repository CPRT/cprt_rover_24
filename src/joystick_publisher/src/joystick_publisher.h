#ifndef JOYSTICK_PUBLISHER_H
#define JOYSTICK_PUBLISHER_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/arm_cmd.hpp"

bool isEqual(interfaces::msg::ArmCmd a, interfaces::msg::ArmCmd b);

class JoystickReader : public rclcpp::Node
{
  public:
    JoystickReader();

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
    interfaces::msg::ArmCmd oldCmd;
    rclcpp::TimerBase::SharedPtr timer_;
    bool shouldPub = false;
		
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    
    void publish_message();
    
    
};

int main(int argc, char ** argv);

#endif
