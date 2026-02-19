// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:srv/GlobalPathService.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__STRUCT_H_
#define BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pnc_map'
#include "base_msgs/msg/detail/pnc_map__struct.h"

/// Struct defined in srv/GlobalPathService in the package base_msgs.
typedef struct base_msgs__srv__GlobalPathService_Request
{
  /// 请求1: 全局规划类型
  int32_t global_planner_type;
  /// 请求2: 地图
  base_msgs__msg__PNCMap pnc_map;
} base_msgs__srv__GlobalPathService_Request;

// Struct for a sequence of base_msgs__srv__GlobalPathService_Request.
typedef struct base_msgs__srv__GlobalPathService_Request__Sequence
{
  base_msgs__srv__GlobalPathService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__srv__GlobalPathService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'global_path'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in srv/GlobalPathService in the package base_msgs.
typedef struct base_msgs__srv__GlobalPathService_Response
{
  /// 响应: 全局路径
  nav_msgs__msg__Path global_path;
} base_msgs__srv__GlobalPathService_Response;

// Struct for a sequence of base_msgs__srv__GlobalPathService_Response.
typedef struct base_msgs__srv__GlobalPathService_Response__Sequence
{
  base_msgs__srv__GlobalPathService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__srv__GlobalPathService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__STRUCT_H_
