// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH__STRUCT_H_

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
// Member 'local_path'
#include "base_msgs/msg/detail/local_path_point__struct.h"

/// Struct defined in msg/LocalPath in the package base_msgs.
/**
  * 路径
 */
typedef struct base_msgs__msg__LocalPath
{
  /// 消息头, 用于坐标系和时间同步
  std_msgs__msg__Header header;
  /// 路径
  base_msgs__msg__LocalPathPoint__Sequence local_path;
} base_msgs__msg__LocalPath;

// Struct for a sequence of base_msgs__msg__LocalPath.
typedef struct base_msgs__msg__LocalPath__Sequence
{
  base_msgs__msg__LocalPath * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__LocalPath__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH__STRUCT_H_
