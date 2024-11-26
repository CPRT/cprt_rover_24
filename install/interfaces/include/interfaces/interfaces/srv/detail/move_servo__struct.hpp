// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_HPP_
#define INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__MoveServo_Request __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__MoveServo_Request __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveServo_Request_
{
  using Type = MoveServo_Request_<ContainerAllocator>;

  explicit MoveServo_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->port = 0ll;
      this->pos = 0ll;
    }
  }

  explicit MoveServo_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->port = 0ll;
      this->pos = 0ll;
    }
  }

  // field types and members
  using _port_type =
    int64_t;
  _port_type port;
  using _pos_type =
    int64_t;
  _pos_type pos;

  // setters for named parameter idiom
  Type & set__port(
    const int64_t & _arg)
  {
    this->port = _arg;
    return *this;
  }
  Type & set__pos(
    const int64_t & _arg)
  {
    this->pos = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::MoveServo_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::MoveServo_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::MoveServo_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::MoveServo_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__MoveServo_Request
    std::shared_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__MoveServo_Request
    std::shared_ptr<interfaces::srv::MoveServo_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveServo_Request_ & other) const
  {
    if (this->port != other.port) {
      return false;
    }
    if (this->pos != other.pos) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveServo_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveServo_Request_

// alias to use template instance with default allocator
using MoveServo_Request =
  interfaces::srv::MoveServo_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces


#ifndef _WIN32
# define DEPRECATED__interfaces__srv__MoveServo_Response __attribute__((deprecated))
#else
# define DEPRECATED__interfaces__srv__MoveServo_Response __declspec(deprecated)
#endif

namespace interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct MoveServo_Response_
{
  using Type = MoveServo_Response_<ContainerAllocator>;

  explicit MoveServo_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->status_msg = "";
    }
  }

  explicit MoveServo_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : status_msg(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = false;
      this->status_msg = "";
    }
  }

  // field types and members
  using _status_type =
    bool;
  _status_type status;
  using _status_msg_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _status_msg_type status_msg;

  // setters for named parameter idiom
  Type & set__status(
    const bool & _arg)
  {
    this->status = _arg;
    return *this;
  }
  Type & set__status_msg(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->status_msg = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interfaces::srv::MoveServo_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const interfaces::srv::MoveServo_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::MoveServo_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interfaces::srv::MoveServo_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interfaces__srv__MoveServo_Response
    std::shared_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interfaces__srv__MoveServo_Response
    std::shared_ptr<interfaces::srv::MoveServo_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MoveServo_Response_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    if (this->status_msg != other.status_msg) {
      return false;
    }
    return true;
  }
  bool operator!=(const MoveServo_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MoveServo_Response_

// alias to use template instance with default allocator
using MoveServo_Response =
  interfaces::srv::MoveServo_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace interfaces

namespace interfaces
{

namespace srv
{

struct MoveServo
{
  using Request = interfaces::srv::MoveServo_Request;
  using Response = interfaces::srv::MoveServo_Response;
};

}  // namespace srv

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_HPP_
