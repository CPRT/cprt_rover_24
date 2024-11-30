#include "rover_arm.h"

namespace ros2_control_rover_arm
{
hardware_interface::CallbackReturn RoverArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // RRBotSystemPositionOnly has exactly one state and command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoverArmHardwareInterface"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoverArmHardwareInterface"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoverArmHardwareInterface"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoverArmHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("RoverArmHardwareInterface"),
        "Joint '%s' have %s state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  
  /*subscription_ = this->create_subscription<std::string>(
  "random_topic", 10, std::bind(&RoverArmHardwareInterface::subscription_callback, this, std::placeholders::_1));*/
  node = rclcpp::Node::make_shared("get_angle_client");
  client = node->create_client<interfaces::srv::ArmPos>("arm_pos");

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
  
  write_node = rclcpp::Node::make_shared("set_angle_client");
  write_client = write_node->create_client<interfaces::srv::ArmCmd>("arm_cmd");

  while (!write_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      return hardware_interface::CallbackReturn::SUCCESS;
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_position_states_.size(); i++)
  {
    hw_position_states_[i] = 0;
    hw_velocity_states_[i] = 0;
    hw_commands_[i] = 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
RoverArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
  }
  
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
RoverArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // command and state should be equal when starting
  for (uint i = 0; i < hw_position_states_.size(); i++)
  {
    hw_commands_[i] = hw_position_states_[i];
  }

  RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoverArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoverArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  //RCLCPP_INFO(rclcpp::get_logger("RoverArmHardwareInterface"), "Reading...");
  
  auto request = std::make_shared<interfaces::srv::ArmPos::Request>();
  request->stop = false;
  auto result = client->async_send_request(request);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hardware reading");
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Base: %f", result.get()->base);
    auto resultCopy = result.get();
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hardware reading %f for elbow", resultCopy->elbow);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hardware reading %f %f %f %f for elbow %f %f", resultCopy->base, resultCopy->diff1, resultCopy->diff2, resultCopy->elbow, resultCopy->wristtilt, resultCopy->wristturn);
    hw_position_states_[0] = resultCopy->base;
		hw_position_states_[1] = resultCopy->diff1;
		hw_position_states_[2] = resultCopy->diff2;
		hw_position_states_[3] = resultCopy->elbow;
		hw_position_states_[4] = resultCopy->wristtilt;
		hw_position_states_[5] = resultCopy->wristturn;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }

  for (uint i = 1; i < hw_velocity_states_.size(); i++)
  {
    // Simulate RRBot's movement
    hw_velocity_states_[i] = 0;
    /*RCLCPP_INFO(
      rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Got state %.5f for joint %d!",
      hw_position_states_[i], i);*/
  }
  /*hw_position_states_[0] = result.get()->base;
  hw_position_states_[1] = result.get()->diff1;
  hw_position_states_[2] = result.get()->diff2;
  hw_position_states_[3] = result.get()->elbow;
  hw_position_states_[4] = result.get()->wristtilt;
  hw_position_states_[5] = result.get()->wristturn;//*/
  //RCLCPP_INFO(rclcpp::get_logger("RRBotSystemPositionOnlyHardware"), "Joints successfully read!");
  // END: This part here is for exemplary purposes - Please do not copy to your production code

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoverArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /*for (uint i = 0; i < hw_commands_.size(); i++)
  {
    hw_commands_[i];
  }*/
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hardware writing %f %f %f %f for elbow %f %f", hw_commands_[0], hw_commands_[1], hw_commands_[2], hw_commands_[3], hw_commands_[4], hw_commands_[5]);
  
  auto request = std::make_shared<interfaces::srv::ArmCmd::Request>();
  request->base = hw_commands_[0];
  request->diff1 = hw_commands_[1];
  request->diff2 = hw_commands_[2];
  request->elbow = hw_commands_[3];
  request->wristtilt = hw_commands_[4];
  request->wristturn = hw_commands_[5];
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing request sent");
  auto result = write_client->async_send_request(request);
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hardware reading");
  // Wait for the result.
  if (rclcpp::spin_until_future_complete(write_node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Base: %f", result.get()->base);
    auto resultCopy = result.get();
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Hardware reading %f for elbow", resultCopy->elbow);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Sent angles");
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service add_two_ints");
  }
  
  //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Writing request received");//*/

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_1

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_control_rover_arm::RoverArmHardwareInterface, hardware_interface::SystemInterface)
