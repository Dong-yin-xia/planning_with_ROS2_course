// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:msg/LocalPathPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__STRUCT_H_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__STRUCT_H_

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

/// Struct defined in msg/LocalPathPoint in the package base_msgs.
/**
  * 路径点
 */
typedef struct base_msgs__msg__LocalPathPoint
{
  /// 位置和航向
  geometry_msgs__msg__PoseStamped pose;
  /// s
  double s;
  /// l
  double l;
  /// ds/dt
  double ds_dt;
  /// l'
  double dl_ds;
  /// dl_dt
  double dl_dt;
  /// dds/dt
  double dds_dt;
  /// l''
  double ddl_ds;
  /// ddl_dt
  double ddl_dt;
  /// 航向角
  double theta;
  /// 曲率
  double kappa;
  /// 曲率变化率
  double dkappa;
  /// 作为投影点的rs
  double rs;
  /// 作为投影点的航向角
  double rtheta;
  /// 作为投影点的曲率
  double rkappa;
  /// 作为投影点的曲率变化率
  double rdkappa;
} base_msgs__msg__LocalPathPoint;

// Struct for a sequence of base_msgs__msg__LocalPathPoint.
typedef struct base_msgs__msg__LocalPathPoint__Sequence
{
  base_msgs__msg__LocalPathPoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__msg__LocalPathPoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__STRUCT_H_
