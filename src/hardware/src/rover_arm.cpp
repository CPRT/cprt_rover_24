#include "rover_arm.h"

namespace ros2_control_rover_arm {
hardware_interface::CallbackReturn RoverArmHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_position_states_.resize(info_.joints.size(), 0);

  hw_velocity_states_.resize(info_.joints.size(),
                             std::numeric_limits<double>::quiet_NaN());

  hw_commands_.resize(info_.joints.size(), 0);

  hw_velocity_commands_.resize(info_.joints.size(), 0);
  // s

  temp_.resize(info_.joints.size(), 0);

  output_.resize(info_.joints.size());

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // RRBotSystemPositionOnly has exactly one state and command interface on
    // each joint
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RoverArmHardwareInterface"),
          "Joint '%s' has %zu command interfaces found. 2 expected.",  // used
                                                                       // to be
                                                                       // 1, but
                                                                       // then
                                                                       // we
                                                                       // added
                                                                       // velocity
          joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(
          rclcpp::get_logger("RoverArmHardwareInterface"),
          "Joint '%s' have %s command interfaces found. '%s' expected.",
          joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
          hardware_interface::HW_IF_POSITION);
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

  for (unsigned long int i = 0; i < subscription_topics_.size(); i++) {
    encoder_subscribers_.push_back(
        node_ptr_->create_subscription<ros_phoenix::msg::MotorStatus>(
            subscription_topics_[i].name, 5, subscription_topics_[i].callback));
  }

  for (unsigned long int i = 0; i < publisher_topics_.size(); i++) {
    motor_publishers_.push_back(
        node_ptr_->create_publisher<ros_phoenix::msg::MotorControl>(
            publisher_topics_[i].name, 5));
  }

  encoder_subscriber_ = node_ptr_->create_subscription<std_msgs::msg::Bool>(
      "encoder_passthrough", 5,
      std::bind(&RoverArmHardwareInterface::encoder_callback, this,
                std::placeholders::_1));
  executor_ptr_->add_node(node_ptr_);
  executor_thread_ = std::thread([this]() { this->executor_ptr_->spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

void RoverArmHardwareInterface::encoder_callback(
    const std_msgs::msg::Bool &encoderPassthrough) {
  encoderPassthrough_ = encoderPassthrough.data;
}

void RoverArmHardwareInterface::base_callback(
    const ros_phoenix::msg::MotorStatus &motorStatus) {
  if (encoderPassthrough_) {
    return;
  }
  temp_[0] = -(motorStatus.position / BASE_GEARBOX /
               (BASE_BIG_GEAR / BASE_SMALL_GEAR));
}

double RoverArmHardwareInterface::act_rad(double pos, double a, double b,
                                          double urdf_offset,
                                          double shaft_length,
                                          double shaft_ticks, double act_length,
                                          double direction) {
  double c = pos / shaft_ticks * shaft_length + act_length;
  double d = (a * a + b * b - c * c) / (2 * a * b);
  if (d > 1 or d < -1) {
    return 0.0;  // just for safety
  }
  return direction * std::acos(d) - urdf_offset;
}

void RoverArmHardwareInterface::act1_callback(
    const ros_phoenix::msg::MotorStatus &motorStatus) {
  if (encoderPassthrough_) {
    return;
  }

  temp_[1] =
      act_rad(motorStatus.position, ACT1_SIDE_A, ACT1_SIDE_B, ACT1_URDF_OFFSET,
              ACT1_SHAFT_LENGTH, ACT1_SHAFT_TICKS, ACT_LENGTH, ACT1_DIRECTION);
}

void RoverArmHardwareInterface::act2_callback(
    const ros_phoenix::msg::MotorStatus &motorStatus) {
  if (encoderPassthrough_) {
    return;
  }

  temp_[2] =
      act_rad(motorStatus.position, ACT2_SIDE_A, ACT2_SIDE_B, ACT2_URDF_OFFSET,
              ACT2_SHAFT_LENGTH, ACT2_SHAFT_TICKS, ACT_LENGTH, ACT2_DIRECTION);
}

void RoverArmHardwareInterface::elbow_callback(
    const ros_phoenix::msg::MotorStatus &motorStatus) {
  if (encoderPassthrough_) {
    return;
  }
  temp_[3] = motorStatus.position / ELBOW_GEARBOX /
             (ELBOW_SMALL_GEAR / ELBOW_BIG_GEAR);
}

void RoverArmHardwareInterface::wrist_tilt_callback(
    const ros_phoenix::msg::MotorStatus &motorStatus) {
  if (encoderPassthrough_) {
    return;
  }
  temp_[4] = -motorStatus.position / WRISTTILT_GEARBOX * (M_PI / 2) -
             WRISTTILT_URDF_OFFSET;
}

void RoverArmHardwareInterface::wrist_turn_callback(
    const ros_phoenix::msg::MotorStatus &motorStatus) {
  if (encoderPassthrough_) {
    return;
  }
  temp_[5] =
      -(motorStatus.position / (WRISTTURN_GEARBOX * WRISTTURN_GEAR)) * 2 * M_PI;
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
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_commands_[i]));
  }

  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // command and state should be equal when starting
  for (uint i = 0; i < hw_position_states_.size(); i++) {
    hw_commands_[i] = hw_position_states_[i];
  }

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
  for (int i = 0; i < 6; i++) {
    hw_position_states_[i] = temp_[i];
  }

  return hardware_interface::return_type::OK;
}

double RoverArmHardwareInterface::act_pos(double rad, double a, double b,
                                          double urdf_offset,
                                          double shaft_length,
                                          double shaft_ticks,
                                          double act_length) {
  rad = rad + urdf_offset;
  double c = std::sqrt(a * a + b * b - 2 * a * b * std::cos(rad));
  c -= act_length;
  return c / shaft_length * shaft_ticks;
}

double RoverArmHardwareInterface::base_pos(double rad) {
  return (-rad * (BASE_BIG_GEAR / BASE_SMALL_GEAR) * BASE_GEARBOX);
}

double RoverArmHardwareInterface::act1_pos(double rad) {
  return act_pos(rad, ACT1_SIDE_A, ACT1_SIDE_B, ACT1_URDF_OFFSET,
                 ACT1_SHAFT_LENGTH, ACT1_SHAFT_TICKS, ACT_LENGTH);
}

double RoverArmHardwareInterface::act2_pos(double rad) {
  return act_pos(rad, ACT2_SIDE_A, ACT2_SIDE_B, ACT2_URDF_OFFSET,
                 ACT2_SHAFT_LENGTH, ACT2_SHAFT_TICKS, ACT_LENGTH);
}

double RoverArmHardwareInterface::elbow_pos(double rad) {
  return (rad * ELBOW_SMALL_GEAR / ELBOW_BIG_GEAR) * ELBOW_GEARBOX;
}
double RoverArmHardwareInterface::wrist_tilt_pos(double rad) {
  return (-(rad + WRISTTILT_URDF_OFFSET) / (M_PI / 2) * WRISTTILT_GEARBOX);
}

double RoverArmHardwareInterface::wrist_turn_pos(double rad) {
  // Note: which way is positive and negative depends on the invert_sensor
  // parameter in the talon launch file, which in turn depends on which way
  // electrical decided to wire the encoders
  return -(rad / (2 * M_PI)) * WRISTTURN_GEARBOX * WRISTTURN_GEAR;
}

hardware_interface::return_type RoverArmHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  auto request = std::make_shared<interfaces::srv::ArmCmd::Request>();

  if (encoderPassthrough_) {
    for (unsigned long int i = 0; i < temp_.size(); i++) {
      temp_[i] = hw_commands_[i];
    }
  }

  for (unsigned long int i = 0; i < output_.size(); i++) {
    output_[i].mode = 1;
  }

  for (unsigned long int i = 0; i < hw_commands_.size(); i++) {
    output_[i].value = publisher_topics_[i].gear_ratio(hw_commands_[i]);
  }

  for (unsigned long int i = 0; i < output_.size(); i++) {
    motor_publishers_[i]->publish(output_[i]);
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_rover_arm

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ros2_control_rover_arm::RoverArmHardwareInterface,
                       hardware_interface::SystemInterface)
