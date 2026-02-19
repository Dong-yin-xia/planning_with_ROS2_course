// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/LocalSpeeds.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'local_speeds'
#include "base_msgs/msg/detail/local_speeds_point__struct.h"

/// Struct defined in msg/LocalSpeeds in the package base_msgs.
/**
  * 速度
 */
typedef struct base_msgs__msg__LocalSpeeds
{
  /// 消息头, 用于坐标系和时间同步
  std_msgs__msg__Header header;
  /// 速度结果
  base_msgs__msg__LocalSpeedsPoint__Sequence local_speeds;
} base_msgs__msg__LocalSpeeds;

// Struct for a sequence of base_msgs__msg__LocalSpeeds.
typedef struct base_msgs__msg__LocalSpeeds__Sequence
{
  base_msgs__msg__LocalSpeeds * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__LocalSpeeds__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__STRUCT_H_
