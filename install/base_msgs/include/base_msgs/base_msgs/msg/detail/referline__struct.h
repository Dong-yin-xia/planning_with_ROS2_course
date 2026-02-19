// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/Referline.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__REFERLINE__STRUCT_H_

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
// Member 'refer_line'
#include "base_msgs/msg/detail/referline_point__struct.h"

/// Struct defined in msg/Referline in the package base_msgs.
/**
  * 参考线
 */
typedef struct base_msgs__msg__Referline
{
  /// 消息头, 用于坐标系和时间同步
  std_msgs__msg__Header header;
  /// 路径
  base_msgs__msg__ReferlinePoint__Sequence refer_line;
} base_msgs__msg__Referline;

// Struct for a sequence of base_msgs__msg__Referline.
typedef struct base_msgs__msg__Referline__Sequence
{
  base_msgs__msg__Referline * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__Referline__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE__STRUCT_H_
