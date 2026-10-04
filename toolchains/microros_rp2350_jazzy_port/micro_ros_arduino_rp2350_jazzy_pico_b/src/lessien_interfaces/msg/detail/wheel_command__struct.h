// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lessien_interfaces:msg/WheelCommand.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "lessien_interfaces/msg/wheel_command.h"


#ifndef LESSIEN_INTERFACES__MSG__DETAIL__WHEEL_COMMAND__STRUCT_H_
#define LESSIEN_INTERFACES__MSG__DETAIL__WHEEL_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

/// Struct defined in msg/WheelCommand in the package lessien_interfaces.
typedef struct lessien_interfaces__msg__WheelCommand
{
  uint64_t source_session_id;
  uint64_t sample_time_unix_us;
  uint32_t sequence;
  uint32_t protocol_version;
  double velocity_rad_s[4];
} lessien_interfaces__msg__WheelCommand;

// Struct for a sequence of lessien_interfaces__msg__WheelCommand.
typedef struct lessien_interfaces__msg__WheelCommand__Sequence
{
  lessien_interfaces__msg__WheelCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lessien_interfaces__msg__WheelCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LESSIEN_INTERFACES__MSG__DETAIL__WHEEL_COMMAND__STRUCT_H_
