// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__MOVE_SERVO__TRAITS_HPP_
#define INTERFACES__SRV__DETAIL__MOVE_SERVO__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "interfaces/srv/detail/move_servo__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveServo_Request & msg,
  std::ostream & out)
{
  out << "{";
  // member: port
  {
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << ", ";
  }

  // member: pos
  {
    out << "pos: ";
    rosidl_generator_traits::value_to_yaml(msg.pos, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveServo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: port
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "port: ";
    rosidl_generator_traits::value_to_yaml(msg.port, out);
    out << "\n";
  }

  // member: pos
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pos: ";
    rosidl_generator_traits::value_to_yaml(msg.pos, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveServo_Request & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::MoveServo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::MoveServo_Request & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::MoveServo_Request>()
{
  return "interfaces::srv::MoveServo_Request";
}

template<>
inline const char * name<interfaces::srv::MoveServo_Request>()
{
  return "interfaces/srv/MoveServo_Request";
}

template<>
struct has_fixed_size<interfaces::srv::MoveServo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<interfaces::srv::MoveServo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<interfaces::srv::MoveServo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace interfaces
{

namespace srv
{

inline void to_flow_style_yaml(
  const MoveServo_Response & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << ", ";
  }

  // member: status_msg
  {
    out << "status_msg: ";
    rosidl_generator_traits::value_to_yaml(msg.status_msg, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MoveServo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }

  // member: status_msg
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status_msg: ";
    rosidl_generator_traits::value_to_yaml(msg.status_msg, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MoveServo_Response & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace srv

}  // namespace interfaces

namespace rosidl_generator_traits
{

[[deprecated("use interfaces::srv::to_block_style_yaml() instead")]]
inline void to_yaml(
  const interfaces::srv::MoveServo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  interfaces::srv::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use interfaces::srv::to_yaml() instead")]]
inline std::string to_yaml(const interfaces::srv::MoveServo_Response & msg)
{
  return interfaces::srv::to_yaml(msg);
}

template<>
inline const char * data_type<interfaces::srv::MoveServo_Response>()
{
  return "interfaces::srv::MoveServo_Response";
}

template<>
inline const char * name<interfaces::srv::MoveServo_Response>()
{
  return "interfaces/srv/MoveServo_Response";
}

template<>
struct has_fixed_size<interfaces::srv::MoveServo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<interfaces::srv::MoveServo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<interfaces::srv::MoveServo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<interfaces::srv::MoveServo>()
{
  return "interfaces::srv::MoveServo";
}

template<>
inline const char * name<interfaces::srv::MoveServo>()
{
  return "interfaces/srv/MoveServo";
}

template<>
struct has_fixed_size<interfaces::srv::MoveServo>
  : std::integral_constant<
    bool,
    has_fixed_size<interfaces::srv::MoveServo_Request>::value &&
    has_fixed_size<interfaces::srv::MoveServo_Response>::value
  >
{
};

template<>
struct has_bounded_size<interfaces::srv::MoveServo>
  : std::integral_constant<
    bool,
    has_bounded_size<interfaces::srv::MoveServo_Request>::value &&
    has_bounded_size<interfaces::srv::MoveServo_Response>::value
  >
{
};

template<>
struct is_service<interfaces::srv::MoveServo>
  : std::true_type
{
};

template<>
struct is_service_request<interfaces::srv::MoveServo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<interfaces::srv::MoveServo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // INTERFACES__SRV__DETAIL__MOVE_SERVO__TRAITS_HPP_
