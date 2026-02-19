// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/PNCMap.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PNC_MAP__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__PNC_MAP__STRUCT_H_

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
// Member 'left_boundary'
// Member 'right_boundary'
// Member 'midline'
#include "visualization_msgs/msg/detail/marker__struct.h"

/// Struct defined in msg/PNCMap in the package base_msgs.
/**
  * pnc地图
 */
typedef struct base_msgs__msg__PNCMap
{
  /// 消息头
  std_msgs__msg__Header header;
  /// 道路长度
  double road_length;
  /// 车道半宽
  double road_half_width;
  /// 左边界
  visualization_msgs__msg__Marker left_boundary;
  /// 右边界
  visualization_msgs__msg__Marker right_boundary;
  /// 中心线
  visualization_msgs__msg__Marker midline;
} base_msgs__msg__PNCMap;

// Struct for a sequence of base_msgs__msg__PNCMap.
typedef struct base_msgs__msg__PNCMap__Sequence
{
  base_msgs__msg__PNCMap * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__PNCMap__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__PNC_MAP__STRUCT_H_
