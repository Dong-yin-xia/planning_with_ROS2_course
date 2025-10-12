// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LocalSpeedsPoint in the package base_msgs.
/**
  * 速度点
 */
typedef struct base_msgs__msg__LocalSpeedsPoint
{
  /// t
  double t;
  /// s_2path
  double s_2path;
  /// ds_dt_2path
  double ds_dt_2path;
  /// dds_dt_2path
  double dds_dt_2path;
  /// 速度
  double speed;
  /// 加速度
  double acceleration;
  /// 加加速度
  double dacceleration;
} base_msgs__msg__LocalSpeedsPoint;

// Struct for a sequence of base_msgs__msg__LocalSpeedsPoint.
typedef struct base_msgs__msg__LocalSpeedsPoint__Sequence
{
  base_msgs__msg__LocalSpeedsPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__LocalSpeedsPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__STRUCT_H_
