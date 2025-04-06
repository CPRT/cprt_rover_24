#ifndef JOYSTICK_CONTROL__ARMIK_MODE_HPP_
#define JOYSTICK_CONTROL__ARMIK_MODE_HPP_

#include "Mode.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "interfaces/srv/move_servo.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/twist_stamped.hpp>

const std::string EEF_FRAME_ID = "Link_7";
const std::string BASE_FRAME_ID = "Link_1";

class ArmIKMode : public Mode {
    public:
        ArmIKMode(rclcpp::Node* node);

        void processJoystickInput(
            std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) override;
            
        void handleTwist(std::shared_ptr<sensor_msgs::msg::Joy> joystickMsg) const;

        static void declareParameters(rclcpp::Node* node);
        void loadParameters();

          /**
         * @brief Sends a request to move a servo
         * @param port The servo port number
         * @param pos The target position
         * @param min The minimum position limit of the servo
         * @param max The maximum position limit of the servo
         * @return The service response
         */
        interfaces::srv::MoveServo::Response sendRequest(int port, int pos, int min,
            int max) const;

        /**
        * @brief Wrapper for servo control
        * @param req_port The servo port number
        * @param req_pos The target position
        * @param req_min The minimum position limit
        * @param req_max The maximum position limit
        */
        void servoRequest(int req_port, int req_pos, int req_min, int req_max) const;


    private:

        // Servo Members
        mutable rclcpp::Client<interfaces::srv::MoveServo>::SharedPtr servo_client_;

        // Servo Constants
        int8_t kServoPort;
        int8_t kServoMin;
        int8_t kServoMax;
        int8_t kClawMax;
        int8_t kClawMin;
        int8_t kxAxis;
        int8_t kyAxis;
        int8_t kzAxis;
        int8_t kAroundX;
        int8_t kAroundY;
        int8_t kAroundZ;
        int8_t kBase;
        int8_t kEEF;
        int8_t kClawOpen;
        int8_t kClawClose;
        mutable double act1Scaler;
        mutable double act2Scaler;
        mutable int8_t servoPos;
        mutable bool buttonPressed;
        mutable bool swapButton;
        mutable std::string frame_to_publish_;



};

#endif  // JOYSTICK_CONTROL__ARMIK_MODE_HPP_