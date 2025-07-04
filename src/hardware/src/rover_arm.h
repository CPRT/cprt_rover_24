#ifndef ROS2_CONTROL_ROVER_ARM
#define ROS2_CONTROL_ROVER_ARM

#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>

#define Phoenix_No_WPI
#include "ctre/Phoenix.h"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "interfaces/msg/motor_control.hpp"
#include "interfaces/msg/motor_status.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// Constants for input_type parameter.
#define ANALOG 1
#define ABSOLUTE 2
#define RELATIVE 3

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
  // TODO Docs CN
  static std::string phoenix_to_HW_type(int control_type);

 private:
  std::vector<double> hw_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<int> control_type_;
  std::vector<std::shared_ptr<ctre::phoenix::motorcontrol::can::TalonSRX>>
      talon_controllers_;
  rclcpp::Node::SharedPtr debug_node_;
  std::vector<rclcpp::Publisher<interfaces::msg::MotorStatus>::SharedPtr>
      status_publishers_;
  std::vector<double> kP_, kI_, kD_;
  std::vector<std::string> control_type_str_;
  std::vector<std::string> sensor_type_;
  std::vector<int> sensor_ticks_;
  std::vector<double> sensor_offset_;
  std::vector<bool> crossover_mode_;
};

}  // namespace ros2_control_rover_arm

#endif  // ROS2_CONTROL_ROVER_ARM