// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/MoveServo.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_H_
#define INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/MoveServo in the package interfaces.
typedef struct interfaces__srv__MoveServo_Request
{
  int64_t port;
  int64_t pos;
} interfaces__srv__MoveServo_Request;

// Struct for a sequence of interfaces__srv__MoveServo_Request.
typedef struct interfaces__srv__MoveServo_Request__Sequence
{
  interfaces__srv__MoveServo_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__MoveServo_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'status_msg'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/MoveServo in the package interfaces.
typedef struct interfaces__srv__MoveServo_Response
{
  bool status;
  rosidl_runtime_c__String status_msg;
} interfaces__srv__MoveServo_Response;

// Struct for a sequence of interfaces__srv__MoveServo_Response.
typedef struct interfaces__srv__MoveServo_Response__Sequence
{
  interfaces__srv__MoveServo_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__MoveServo_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__MOVE_SERVO__STRUCT_H_
