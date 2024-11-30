#ifndef ROS2_CONTROL_ROVER_ARM
#define ROS2_CONTROL_ROVER_ARM

#include <memory>
#include <string>
#include <vector>

#include <chrono>
#include <cmath>
#include <limits>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <rclcpp/rclcpp.hpp>
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "interfaces/srv/arm_pos.hpp"
#include "interfaces/srv/arm_cmd.hpp"

using namespace std::chrono_literals;

namespace ros2_control_rover_arm
{
class RoverArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoverArmHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
  //void subscription_callback(const std::string);

private:
  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Client<interfaces::srv::ArmPos>::SharedPtr client;
  std::shared_ptr<rclcpp::Node> write_node;
  rclcpp::Client<interfaces::srv::ArmCmd>::SharedPtr write_client;
  //rclcpp::Subscription<std::string>::SharedPtr subscription_;
};

}  // namespace ros2_control_demo_example_1

#endif  // ROS2_CONTROL_ROVER_ARM
