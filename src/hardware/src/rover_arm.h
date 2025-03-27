#ifndef ROS2_CONTROL_ROVER_ARM
#define ROS2_CONTROL_ROVER_ARM

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "interfaces/srv/arm_cmd.hpp"
#include "interfaces/srv/arm_pos.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ros_phoenix/msg/motor_control.hpp"
#include "ros_phoenix/msg/motor_status.hpp"
#include <cmath>
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;

namespace ros2_control_rover_arm {
class RoverArmHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverArmHardwareInterface)

  // runs on start
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override;

  // runs on configure
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  // returns the pointer to the hardware position states vector
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  // returns the pointer to the hardware commands states vector
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  // runs on activate
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  // runs on deactivate
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  // gets called every so often by ros2_control, where you have to update the
  // hardware positions vector
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  // gets called every so often by ros2_control, so you have to read from the
  // hardware commands vector and send it out
  hardware_interface::return_type write(
      const rclcpp::Time &time, const rclcpp::Duration &period) override;

 private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> temp_;
  std::vector<double> temp2_;
  /*std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Client<interfaces::srv::ArmPos>::SharedPtr client_;
  std::shared_ptr<rclcpp::Node> write_node_;
  rclcpp::Client<interfaces::srv::ArmCmd>::SharedPtr write_client_;*/
  
  //attempt to lower latency:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Executor::SharedPtr executor_ptr_;
  std::thread executor_thread_;
  rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr subscription1_;
  rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr subscription2_;
  rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr subscription3_;
  rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr subscription4_;
  rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr subscription5_;
  rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr subscription6_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr publisher1_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr publisher2_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr publisher3_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr publisher4_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr publisher5_;
  rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr publisher6_;
  
  void topic_callback1(const ros_phoenix::msg::MotorStatus &motorStatus);
  void topic_callback2(const ros_phoenix::msg::MotorStatus &motorStatus);
  void topic_callback3(const ros_phoenix::msg::MotorStatus &motorStatus);
  void topic_callback4(const ros_phoenix::msg::MotorStatus &motorStatus);
  void topic_callback5(const ros_phoenix::msg::MotorStatus &motorStatus);
  void topic_callback6(const ros_phoenix::msg::MotorStatus &motorStatus);
  
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr encoder_subscriber_;
  void encoder_callback(const std_msgs::msg::Bool &encoderPassthrough);
  
  bool encoderPassthrough_ = true;
};

}  // namespace ros2_control_rover_arm

#endif  // ROS2_CONTROL_ROVER_ARM
