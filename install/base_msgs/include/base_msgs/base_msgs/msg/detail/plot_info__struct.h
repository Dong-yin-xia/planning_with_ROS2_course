// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/PlotInfo.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PLOT_INFO__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__PLOT_INFO__STRUCT_H_

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
// Member 'trajectory_info'
#include "base_msgs/msg/detail/local_trajectory__struct.h"
// Member 'obs_info'
#include "base_msgs/msg/detail/obs_info__struct.h"

/// Struct defined in msg/PlotInfo in the package base_msgs.
/**
  * 绘制信息
 */
typedef struct base_msgs__msg__PlotInfo
{
  /// 消息头, 用于坐标系和时间同步
  std_msgs__msg__Header header;
  /// 轨迹信息
  base_msgs__msg__LocalTrajectory trajectory_info;
  /// 障碍物信息
  base_msgs__msg__ObsInfo__Sequence obs_info;
} base_msgs__msg__PlotInfo;

// Struct for a sequence of base_msgs__msg__PlotInfo.
typedef struct base_msgs__msg__PlotInfo__Sequence
{
  base_msgs__msg__PlotInfo * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__PlotInfo__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__PLOT_INFO__STRUCT_H_
