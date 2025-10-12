// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/LocalTrajectoryPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path_point'
#include "base_msgs/msg/detail/local_path_point__struct.h"
// Member 'speed_point'
#include "base_msgs/msg/detail/local_speeds_point__struct.h"

/// Struct defined in msg/LocalTrajectoryPoint in the package base_msgs.
/**
  * 轨迹点
 */
typedef struct base_msgs__msg__LocalTrajectoryPoint
{
  /// 路径点
  base_msgs__msg__LocalPathPoint path_point;
  /// 速度点
  base_msgs__msg__LocalSpeedsPoint speed_point;
} base_msgs__msg__LocalTrajectoryPoint;

// Struct for a sequence of base_msgs__msg__LocalTrajectoryPoint.
typedef struct base_msgs__msg__LocalTrajectoryPoint__Sequence
{
  base_msgs__msg__LocalTrajectoryPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__LocalTrajectoryPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__STRUCT_H_
