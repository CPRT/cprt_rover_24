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
  temp_.resize(info_.joints.size() * 2, 0);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    std::string node_name;
    for (const auto &param : joint.parameters) {
      std::string name(param.first);
      std::string value(param.second);
      if (name == "node_name") {
        node_name = value;
      }
    }
    if (node_name.empty()) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Missing necessary parameter node_name for joint %s",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    node_names_.push_back(node_name);
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Joint '%s' has %zu command interface. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.command_interfaces[0].name ==
        hardware_interface::HW_IF_POSITION) {
      control_type_.emplace_back(ros_phoenix::msg::MotorControl::POSITION);
    } else if (joint.command_interfaces[0].name ==
               hardware_interface::HW_IF_VELOCITY) {
      control_type_.emplace_back(ros_phoenix::msg::MotorControl::VELOCITY);
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
                   "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("RoverArmHardwareInterface"),
                   "Joint '%s' have %s state interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[1].name.c_str(),
                   hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  node_ptr_ = rclcpp::Node::make_shared("arm_hw_subscriber");

  executor_ptr_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  executor_ptr_->add_node(node_ptr_);
  executor_thread_ = std::thread([this]() { this->executor_ptr_->spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // reset values always when configuring hardware
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
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_position_states_[i]));
  }

  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RoverArmHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, phoenix_to_HW_type(control_type_[i]),
        &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // command and state should be equal when starting
  publish_sub_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
      "/arm_active", 10,
      [&](const std_msgs::msg::Bool::SharedPtr msg) { publish_ = msg->data; });
  const size_t num_joints = info_.joints.size();
  for (size_t i = 0; i < num_joints; i++) {
    hw_commands_[i] = hw_position_states_[i];
    encoder_subscribers_.emplace_back(
        node_ptr_->create_subscription<ros_phoenix::msg::MotorStatus>(
            node_names_[i] + "/status", 10,
            [this, i](const ros_phoenix::msg::MotorStatus::SharedPtr msg) {
              temp_[i] = msg->position;
            }));
    // Create publisher for each joint
    motor_publishers_.emplace_back(
        node_ptr_->create_publisher<ros_phoenix::msg::MotorControl>(
            node_names_[i] + "/set", 10));
  }
  // Create a service to switch open loop
  open_loop_service_ = node_ptr_->create_service<std_srvs::srv::SetBool>(
      "open_loop",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
             std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        open_loop_ = request->data;
        response->success = true;
        response->message =
            "Open loop mode set to " + std::to_string(open_loop_);
      });
  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"),
              "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverArmHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  const int num_joints = info_.joints.size();
  if (open_loop_) {
    for (int i = 0; i < num_joints; i++) {
      hw_position_states_[i] = hw_commands_[i];
      hw_velocity_states_[i] = 0;
    }
    return hardware_interface::return_type::OK;
  }
  for (int i = 0; i < num_joints; i++) {
    hw_position_states_[i] = temp_[i];
    hw_velocity_states_[i] = temp_[i + num_joints];
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverArmHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  if (!publish_) {
    return hardware_interface::return_type::OK;
  }
  ros_phoenix::msg::MotorControl request;
  const int num_joints = info_.joints.size();
  for (int i = 0; i < num_joints; i++) {
    request.mode = control_type_[i];
    request.value = hw_commands_[i];
    motor_publishers_[i]->publish(request);
  }

  return hardware_interface::return_type::OK;
}

std::string RoverArmHardwareInterface::phoenix_to_HW_type(
    const int control_type) {
  if (control_type == ros_phoenix::msg::MotorControl::POSITION) {
    return hardware_interface::HW_IF_POSITION;
  } else if (control_type == ros_phoenix::msg::MotorControl::VELOCITY) {
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
