#ifndef JOYSTICK_CONTROL__DRIVEMODE_HPP_
#define JOYSTICK_CONTROL__DRIVEMODE_HPP_

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

/**
 * @class DriveMode
 * @brief A class that handles the teleop drive mode of the rover using joystick
 * input.
 *
 * This class inherits from the Mode class and processes joystick input to
 * control the rover's movement, camera, and video functionalities.
 */
class DriveMode : public Mode {
 public:
  /**
   * @brief Constructor for the DriveMode class.
   *
   * @param node A pointer to the rclcpp::Node instance.
   */
  DriveMode(rclcpp::Node* node);
  /**
   * @brief Processes the joystick input.
   *
   * @param joystickMsg A shared pointer to the incoming sensor_msgs::msg::Joy
   * message.
   */
  void processJoystickInput(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;
  /**
   * @brief Declare the parameters for the mode.
   * @param node Pointer to the ROS2 node.
   */
  static void declareParameters(rclcpp::Node* node);

 private:
  /**
   * @brief creates and publishes the twist msg based on joystickinput.
   *
   * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
   */
  void handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

  /**
   * @brief Handles the camera control based on joystick input. Not implemented
   * yet.
   *
   * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
   */
  void handleCam(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

  /**
   * @brief Handles the video control based on joystick input. Not implemented
   * yet.
   *
   * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
   */
  void handleVideo(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

  /**
   * @brief Gets the throttle value from the joystick input.
   *
   * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
   * @return The throttle value as a double between 0 and one inclusive.
   */
  double getThrottleValue(
      std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

  void loadParameters();

  // Parameters
  int8_t kForwardAxis;    ///< Axis for forward movement.
  int8_t kYawAxis;        ///< Axis for yaw (rotation).
  int8_t kCamTiltAxis;    ///< Axis for camera tilt.
  int8_t kCamPanAxis;     ///< Axis for camera pan.
  int8_t kCamReset;       ///< Button for resetting the camera.
  int8_t kCamNext;        ///< Button for switching to the next camera.
  int8_t kCamPrev;        ///< Button for switching to the previous camera.
  int8_t kCruiseControl;  ///< Button for enabling cruise control.
  int8_t kThrottleAxis;   ///< Axis for throttle control.

  double kThrottleMax;   ///< Maximum throttle value from joystick.
  double kThrottleMin;   ///< Minimum throttle value from joystick.
  double kMaxLinear;     ///< Maximum linear velocity.
  double kMaxAngular;    ///< Maximum angular velocity.
  double kMaxIncrement;  ///< Maximum increment for speed changes.
  double kMinSpeed;      ///< Minimum speed value.

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      twist_pub_;  ///< Publisher for Twist messages.
};

#endif  // JOYSTICK_CONTROL__DRIVEMODE_HPP_