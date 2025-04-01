#ifndef TYPING_CONTROLLER_H
#define TYPING_CONTROLLER

#include <memory>
#include <unistd.h>
#include <iostream>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/arm_cmd.hpp"
#include "interfaces/srv/keycap_cmd.hpp"
#include <moveit/robot_state/robot_state.h>
//#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

using namespace std::chrono_literals;
using namespace std;

struct Cmd
{
  int x, y, z;
};

class TypingNode : public rclcpp::Node
{
  public:
    TypingNode();

  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Subscription<moveit_msgs::action::ExecuteTrajectory_FeedbackMessage>::SharedPtr subscription2_;
    void topic_callback(const std_msgs::msg::String &msg);
    void arm_callback(const moveit_msgs::action::ExecuteTrajectory_FeedbackMessage &msg);
    void getCmd(char k);
    
    //things for tf
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Client<interfaces::srv::KeycapCmd>::SharedPtr client;
    
    //typing control loop
    string key;
    Cmd cmd;
    int currDim;
    long unsigned int currLetter;
    bool adjusted = false; //he just like me fr fr
    bool goingBack = false;
    bool hasJob = false;
    
    //interfacing with moveit_controller (will probably change later)
    rclcpp::Publisher<interfaces::msg::ArmCmd>::SharedPtr publisher_;
    
    
};

#endif
