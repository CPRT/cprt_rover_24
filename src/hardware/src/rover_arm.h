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
#include "std_msgs/msg/bool.hpp"

#define BASE_BIG_GEAR 100.0
#define BASE_SMALL_GEAR 15.0
#define BASE_GEARBOX 1100
#define ACT1_URDF_OFFSET 0.7832711  // thanks alot ivan >:((
#define ACT1_SIDE_A 20.1
#define ACT1_SIDE_B 48.5
#define ACT1_SHAFT_LENGTH 15.24
#define ACT1_SHAFT_TICKS 5709.0
#define ACT_LENGTH 30.96
#define ACT2_URDF_OFFSET -0.89151
#define ACT2_SIDE_A 15.0
#define ACT2_SIDE_B 42.3
#define ACT2_SHAFT_LENGTH 13.64
#define ACT2_SHAFT_TICKS 5109.0
#define ELBOW_SMALL_GEAR 30.0
#define ELBOW_BIG_GEAR 96.0
#define ELBOW_GEARBOX 10000.0
#define WRISTTILT_GEARBOX \
  7760215.0  // I'm confident that nobody knows what the ratios are for this
             // gearbox
#define WRISTTURN_GEAR 498.0
#define WRISTTURN_GEARBOX 4000.0

using namespace std::chrono_literals;

struct EncoderTopic {
  std::string name;
  std::function<void(const ros_phoenix::msg::MotorStatus &)> callback;
};

struct PublisherTopic {
  std::string name;
  std::function<double(double)> gear_ratio;
};

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

  // handling publisher and subscriber callbacks
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Executor::SharedPtr executor_ptr_;
  std::thread executor_thread_;

  void base_callback(const ros_phoenix::msg::MotorStatus &motorStatus);
  void act1_callback(const ros_phoenix::msg::MotorStatus &motorStatus);
  void act2_callback(const ros_phoenix::msg::MotorStatus &motorStatus);
  void elbow_callback(const ros_phoenix::msg::MotorStatus &motorStatus);
  void wrist_tilt_callback(const ros_phoenix::msg::MotorStatus &motorStatus);
  void wrist_turn_callback(const ros_phoenix::msg::MotorStatus &motorStatus);

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr encoder_subscriber_;
  void encoder_callback(const std_msgs::msg::Bool &encoderPassthrough);

  std::vector<EncoderTopic> subscription_topics_ = {
      {"base/status", std::bind(&RoverArmHardwareInterface::base_callback, this,
                                std::placeholders::_1)},
      {"act1/status", std::bind(&RoverArmHardwareInterface::act1_callback,
                                 this, std::placeholders::_1)},
      {"act2/status", std::bind(&RoverArmHardwareInterface::act2_callback,
                                 this, std::placeholders::_1)},
      {"elbow/status", std::bind(&RoverArmHardwareInterface::elbow_callback,
                                 this, std::placeholders::_1)},
      {"wristTilt/status",
       std::bind(&RoverArmHardwareInterface::wrist_tilt_callback, this,
                 std::placeholders::_1)},
      {"wristTurn/status",
       std::bind(&RoverArmHardwareInterface::wrist_turn_callback, this,
                 std::placeholders::_1)},
  };

  double base_pos(double rad);
  double act1_pos(double rad);
  double act2_pos(double rad);
  double elbow_pos(double rad);
  double wrist_tilt_pos(double rad);
  double wrist_turn_pos(double rad);

  double act_pos(double rad, double a, double b, double urdf_offset,
                 double shaft_length, double shaft_ticks, double act_length);
  double act_rad(double pos, double a, double b, double urdf_offset,
                 double shaft_length, double shaft_ticks, double act_length);

  std::vector<PublisherTopic> publisher_topics_ = {
      {"base/set", std::bind(&RoverArmHardwareInterface::base_pos, this,
                             std::placeholders::_1)},
      {"act1/set", std::bind(&RoverArmHardwareInterface::act1_pos, this,
                              std::placeholders::_1)},
      {"act2/set", std::bind(&RoverArmHardwareInterface::act2_pos, this,
                              std::placeholders::_1)},
      {"elbow/set", std::bind(&RoverArmHardwareInterface::elbow_pos, this,
                              std::placeholders::_1)},
      {"wristTilt/set", std::bind(&RoverArmHardwareInterface::wrist_tilt_pos,
                                  this, std::placeholders::_1)},
      {"wristTurn/set", std::bind(&RoverArmHardwareInterface::wrist_turn_pos,
                                  this, std::placeholders::_1)},
  };

  std::vector<rclcpp::Subscription<ros_phoenix::msg::MotorStatus>::SharedPtr>
      encoder_subscribers_;
  std::vector<rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr>
      motor_publishers_;

  std::vector<ros_phoenix::msg::MotorControl> output_;

  bool encoderPassthrough_ = true;
};

}  // namespace ros2_control_rover_arm

#endif  // ROS2_CONTROL_ROVER_ARM
