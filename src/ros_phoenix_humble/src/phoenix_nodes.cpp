#include "ros_phoenix/phoenix_nodes.hpp"

#include "ros_phoenix/base_node.hpp"

namespace ros_phoenix {

TalonSRXNode::TalonSRXNode(const std::string &name, const NodeOptions &options)
    : PhoenixNode(name, options) {}

void TalonSRXNode::configure_current_limit(TalonSRXConfiguration &config) {
  config.continuousCurrentLimit =
      this->get_parameter("max_current").as_double();
  config.peakCurrentLimit = this->get_parameter("max_current").as_double();
  config.peakCurrentDuration = 100;  // ms
  this->controller_->EnableCurrentLimit(true);
}

void TalonSRXNode::configure_sensor() {
  if (this->get_parameter("non_continuous").as_bool()) {
    this->controller_->ConfigFeedbackNotContinuous(true);
  }
  if (this->get_parameter("input_type").as_int() == ANALOG)
    this->controller_->ConfigSelectedFeedbackSensor(
        TalonSRXFeedbackDevice::Analog);
  else if (this->get_parameter("input_type").as_int() == ABSOLUTE) {
    this->controller_->ConfigSelectedFeedbackSensor(
        TalonSRXFeedbackDevice::CTRE_MagEncoder_Absolute);
  } else if (this->get_parameter("input_type").as_int() == RELATIVE) {
    this->controller_->ConfigSelectedFeedbackSensor(
        TalonSRXFeedbackDevice::CTRE_MagEncoder_Relative);
  } else {
    RCLCPP_WARN(this->get_logger(), "%ld is an invalid input_type setting.",
                this->get_parameter("input_type").as_int());
  }
}

double TalonSRXNode::get_output_current() {
  return this->controller_->GetOutputCurrent();
}

}  // namespace ros_phoenix
