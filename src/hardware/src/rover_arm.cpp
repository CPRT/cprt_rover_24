#include "rover_arm.h"

namespace ros2_control_rover_arm {

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  hw_commands_.resize(info_.joints.size(), 0);
  hw_position_states_.resize(info_.joints.size(),
                             std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info_.joints.size(), 0);
  control_type_.resize(info_.joints.size(), 0);
  talon_controllers_.clear();
  kP_.resize(info_.joints.size(), 0.0);
  kI_.resize(info_.joints.size(), 0.0);
  kD_.resize(info_.joints.size(), 0.0);
  control_type_str_.resize(info_.joints.size());
  sensor_type_.resize(info_.joints.size());
  sensor_ticks_.resize(info_.joints.size(), 4096);
  sensor_offset_.resize(info_.joints.size(), 0.0);
  crossover_mode_.resize(info_.joints.size(), false);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    int can_id = -1;
    for (const auto &param : joint.parameters) {
      if (param.first == "can_id") {
        can_id = std::stoi(param.second);
      }
    }
    if (can_id < 0) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Missing or invalid can_id for joint %s",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    talon_controllers_.emplace_back(
        std::make_shared<ctre::phoenix::motorcontrol::can::TalonSRX>(can_id));

    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Joint '%s' has %zu command interface. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name ==
        hardware_interface::HW_IF_POSITION) {
      control_type_[talon_controllers_.size() - 1] =
          interfaces::msg::MotorControl::POSITION;
    } else if (joint.command_interfaces[0].name ==
               hardware_interface::HW_IF_VELOCITY) {
      control_type_[talon_controllers_.size() - 1] =
          interfaces::msg::MotorControl::VELOCITY;
    } else {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Joint '%s' has unknown command interface %s.",
                   joint.name.c_str(),
                   joint.command_interfaces[0].name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Joint '%s' has %zu state interface. 2 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Joint '%s' has %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Joint '%s' has %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Create a debug node for publishing status
  debug_node_ = std::make_shared<rclcpp::Node>("rover_arm_debug_node");
  status_publishers_.clear();
  for (const auto &joint : info_.joints) {
    std::string topic = joint.name + "/status";
    status_publishers_.push_back(
        debug_node_->create_publisher<interfaces::msg::MotorStatus>(topic, 10));
  }

  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully initialized TalonSRX controllers!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (uint i = 0; i < hw_position_states_.size(); i++) {
    hw_position_states_[i] = 0;
    hw_velocity_states_[i] = 0;
    hw_commands_[i] = 0;
  }
  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully configured!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RoverArmHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < hw_position_states_.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "joint" + std::to_string(i), hardware_interface::HW_IF_POSITION,
        &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "joint" + std::to_string(i), hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_states_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RoverArmHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < hw_commands_.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "joint" + std::to_string(i), phoenix_to_HW_type(control_type_[i]),
        &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  for (size_t i = 0; i < hw_commands_.size(); i++) {
    hw_commands_[i] = hw_position_states_[i];
  }
  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverArmHardwareInterface::read(
    const rclcpp::Time &, const rclcpp::Duration &) {
  for (size_t i = 0; i < talon_controllers_.size(); ++i) {
    double raw_ticks = talon_controllers_[i]->GetSelectedSensorPosition();
    double offset = sensor_offset_[i];
    double ticks_per_rev = static_cast<double>(sensor_ticks_[i]);
    double pos_rad = 0.0;

    // Convert to radians, apply offset
    pos_rad = ((raw_ticks / ticks_per_rev) * 2.0 * M_PI) + offset;

    // Crossover mode: wrap between 0 and 2pi
    if (crossover_mode_[i]) {
      pos_rad = std::fmod(pos_rad, 2.0 * M_PI);
      if (pos_rad < 0) pos_rad += 2.0 * M_PI;
    }

    hw_position_states_[i] = pos_rad;
    hw_velocity_states_[i] =
        talon_controllers_[i]
            ->GetSelectedSensorVelocity();  // May want to convert this as well
                                            // to rad/s TBD

    // Publish status for debug
    interfaces::msg::MotorStatus status_msg;
    status_msg.temperature = talon_controllers_[i]->GetTemperature();
    status_msg.bus_voltage = talon_controllers_[i]->GetBusVoltage();
    status_msg.output_percent = talon_controllers_[i]->GetMotorOutputPercent();
    status_msg.output_voltage = talon_controllers_[i]->GetMotorOutputVoltage();
    status_msg.output_current = talon_controllers_[i]->GetOutputCurrent();
    status_msg.position = hw_position_states_[i];
    status_msg.velocity = hw_velocity_states_[i];
    status_msg.fwd_limit = talon_controllers_[i]->IsFwdLimitSwitchClosed() == 1;
    status_msg.rev_limit = talon_controllers_[i]->IsRevLimitSwitchClosed() == 1;

    status_publishers_[i]->publish(status_msg);
  }
  rclcpp::spin_some(debug_node_);
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverArmHardwareInterface::write(
    const rclcpp::Time &, const rclcpp::Duration &) {
  for (size_t i = 0; i < talon_controllers_.size(); ++i) {
    if (control_type_[i] == interfaces::msg::MotorControl::POSITION) {
      talon_controllers_[i]->Set(
          ctre::phoenix::motorcontrol::ControlMode::Position, hw_commands_[i]);
    } else if (control_type_[i] == interfaces::msg::MotorControl::VELOCITY) {
      talon_controllers_[i]->Set(
          ctre::phoenix::motorcontrol::ControlMode::Velocity, hw_commands_[i]);
    }
  }
  return hardware_interface::return_type::OK;
}

std::string RoverArmHardwareInterface::phoenix_to_HW_type(
    const int control_type) {
  if (control_type == interfaces::msg::MotorControl::POSITION) {
    return hardware_interface::HW_IF_POSITION;
  } else if (control_type == interfaces::msg::MotorControl::VELOCITY) {
    return hardware_interface::HW_IF_VELOCITY;
  }
  RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
               "Unknown control type %d", control_type);
  throw std::invalid_argument("Unknown control type");
}

}  // namespace ros2_control_rover_arm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_rover_arm::RoverArmHardwareInterface,
                       hardware_interface::SystemInterface)
