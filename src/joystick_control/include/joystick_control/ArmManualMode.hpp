#ifndef JOYSTICK_CONTROL__ARMMANUAL_MODE_HPP_
#define JOYSTICK_CONTROL__ARMMANUAL_MODE_HPP_

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros_phoenix/msg/motor_control.hpp"

/**
 * @class ArmManualMode
 * @brief A class that handles the manual arm control using a joystick
 * 
 * This class inherits from the Mode class and processes joystick input to
 * control the arm manually, meaning each join is controlled individually.
 */
class ArmManualMode : public Mode {
    public:
        /**
         * @brief Constructor to the ArmManualMode class.
         * 
         * @param node A pointer to the rclcpp::Node instance.
         */
        ArmManualMode(rclcpp::Node* node);

        /**
        * @brief Processes the joystick input.
        *
        * @param joystickMsg A shared pointer to the incoming sensor_msgs::msg::Joy
        * message.
        */
        void processJoystickInput(
        std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;

        static void declareParameters(rclcpp::Node* node);

    private:
        /**
         * @brief creates and publishes the MotorControl msg based on joystickinput.
         *
         * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
         */
        void handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

        /**
        * @brief Gets the throttle value from the joystick input.
        *
        * @param joystickMsg A shared pointer to the sensor_msgs::msg::Joy message.
        * @return The throttle value as a double between 0 and one inclusive.
        */
        double getThrottleValue(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

        void loadParameters();
        
        // Parameters
        int8_t kBaseAxis;           ///< Axis for movement of the base
        int8_t kWristRoll;          ///< Axis for the roll of the wrist joint
        int8_t kWristYawPositive;   ///< Button for the wrist join to rotate in the positive direction
        int8_t kWristYawNegative;   ///< Button for the wrist join to rotate in the negative direction
        int8_t kDiff1Axis;          ///< Axis for the upper linear actuator control
        int8_t kDiff2Axis;          ///< Axis for the lower linear actuator control
        int8_t kElbowYaw;           ///< Axis for the yaw of the elbow joint
        
        int8_t kThrottleAxis;       ///< Axis for the throttle value
        double kThrottleMax;        ///< Maximum throttle value from joystick.
        double kThrottleMin;        ///< Minimum throttle value from joystick.

        // Publishers 
        rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr base_pub_;
        rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr diff1_pub_;
        rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr diff2_pub_;
        rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr elbow_pub_;
        rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr wristTilt_pub_;
        rclcpp::Publisher<ros_phoenix::msg::MotorControl>::SharedPtr wristTurn_pub_;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
      twist_pub_;  ///< Publisher for Twist messages.

        //MotorControl Messages
        mutable std::shared_ptr<ros_phoenix::msg::MotorControl> base_;
        mutable std::shared_ptr<ros_phoenix::msg::MotorControl> diff1_;
        mutable std::shared_ptr<ros_phoenix::msg::MotorControl> diff2_;
        mutable std::shared_ptr<ros_phoenix::msg::MotorControl> elbow_;
        mutable std::shared_ptr<ros_phoenix::msg::MotorControl> wristTilt_;
        mutable std::shared_ptr<ros_phoenix::msg::MotorControl> wristTurn_;
        
 };

#endif //JOYSTICK_CONTROL__ARMMANUAL_MODE_HPP_