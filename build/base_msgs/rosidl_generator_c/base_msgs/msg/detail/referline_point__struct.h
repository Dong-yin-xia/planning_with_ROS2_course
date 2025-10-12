// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/ReferlinePoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.h"

/// Struct defined in msg/ReferlinePoint in the package base_msgs.
/**
  * 参考线点
 */
typedef struct base_msgs__msg__ReferlinePoint
{
  /// 位置和航向
  geometry_msgs__msg__PoseStamped pose;
  /// 作为投影点的rs
  double rs;
  /// 作为投影点的航向角
  double rtheta;
  /// 作为投影点的曲率
  double rkappa;
  /// 作为投影点的曲率变化率
  double rdkappa;
} base_msgs__msg__ReferlinePoint;

// Struct for a sequence of base_msgs__msg__ReferlinePoint.
typedef struct base_msgs__msg__ReferlinePoint__Sequence
{
  base_msgs__msg__ReferlinePoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__ReferlinePoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__STRUCT_H_
