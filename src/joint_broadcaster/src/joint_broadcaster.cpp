#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
* member function as a callback from the timer. */

class JointBroadcaster : public rclcpp::Node
{
  public:
    JointBroadcaster()
    : Node("joint_broadcaster"), count_(0)
    {
      publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
      timer_ = this->create_wall_timer(
      10ms, std::bind(&JointBroadcaster::timer_callback, this));
    }

  private:
    void timer_callback()
    {
      auto message = sensor_msgs::msg::JointState();
      message.name[0] = "joint_1";
      message.name[1] = "joint_2";
      message.name[2] = "joint_3";
      message.name[3] = "joint_4";
      message.name[4] = "joint_5";
      message.name[5] = "joint_6";
      message.position[0] = 0;
      message.position[1] = 0;
      message.position[2] = 0;
      message.position[3] = 0;
      message.position[4] = 0;
      message.position[5] = 0;
      publisher_->publish(message);
      /*auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);*/
    }
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
