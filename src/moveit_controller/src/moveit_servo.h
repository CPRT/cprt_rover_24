#ifndef MOVEIT_SERVO_H
#define MOVEIT_SERVO_H

#include <memory>
#include <unistd.h>
#include <iostream>

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "std_msgs/msg/string.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "interfaces/msg/arm_cmd.hpp"
#include <moveit/robot_state/robot_state.h>
#include <std_srvs/srv/trigger.hpp>

#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/servo.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "sensor_msgs/msg/joy.hpp"
//#include <moveit/planning_scene/planning_scene.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>

using namespace std::chrono_literals;

class ServoNode : public rclcpp::Node
{
  public:
    ServoNode(const rclcpp::NodeOptions &options);

  private:
    /*std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> planning_scene;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
    
    rclcpp::Node::SharedPtr node_ptr;
    rclcpp::Executor::SharedPtr executor_ptr;
    std::thread executor_thread;
    
    std::shared_ptr<const moveit_servo::ServoParameters> servo_parameters;
    std::unique_ptr<moveit_servo::Servo> servo;*/
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;
    bool moved = false;
    
    void topic_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    //void pub();
    geometry_msgs::msg::TwistStamped cmd;
    
    void move();
};

#endif
