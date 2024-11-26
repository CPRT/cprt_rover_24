// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__MOVE_SERVO__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__MOVE_SERVO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/move_servo__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveServo_Request_pos
{
public:
  explicit Init_MoveServo_Request_pos(::interfaces::srv::MoveServo_Request & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::MoveServo_Request pos(::interfaces::srv::MoveServo_Request::_pos_type arg)
  {
    msg_.pos = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::MoveServo_Request msg_;
};

class Init_MoveServo_Request_port
{
public:
  Init_MoveServo_Request_port()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveServo_Request_pos port(::interfaces::srv::MoveServo_Request::_port_type arg)
  {
    msg_.port = std::move(arg);
    return Init_MoveServo_Request_pos(msg_);
  }

private:
  ::interfaces::srv::MoveServo_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::MoveServo_Request>()
{
  return interfaces::srv::builder::Init_MoveServo_Request_port();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveServo_Response_status_msg
{
public:
  explicit Init_MoveServo_Response_status_msg(::interfaces::srv::MoveServo_Response & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::MoveServo_Response status_msg(::interfaces::srv::MoveServo_Response::_status_msg_type arg)
  {
    msg_.status_msg = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::MoveServo_Response msg_;
};

class Init_MoveServo_Response_status
{
public:
  Init_MoveServo_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveServo_Response_status_msg status(::interfaces::srv::MoveServo_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MoveServo_Response_status_msg(msg_);
  }

private:
  ::interfaces::srv::MoveServo_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::MoveServo_Response>()
{
  return interfaces::srv::builder::Init_MoveServo_Response_status();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__MOVE_SERVO__BUILDER_HPP_
