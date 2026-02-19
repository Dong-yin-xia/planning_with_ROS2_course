// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from base_msgs:srv/PNCMapService.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__STRUCT_H_
#define BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/PNCMapService in the package base_msgs.
typedef struct base_msgs__srv__PNCMapService_Request
{
  /// 请求: 地图类型
  int32_t map_type;
} base_msgs__srv__PNCMapService_Request;

// Struct for a sequence of base_msgs__srv__PNCMapService_Request.
typedef struct base_msgs__srv__PNCMapService_Request__Sequence
{
  base_msgs__srv__PNCMapService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__srv__PNCMapService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'pnc_map'
#include "base_msgs/msg/detail/pnc_map__struct.h"

/// Struct defined in srv/PNCMapService in the package base_msgs.
typedef struct base_msgs__srv__PNCMapService_Response
{
  /// 响应: 地图
  base_msgs__msg__PNCMap pnc_map;
} base_msgs__srv__PNCMapService_Response;

// Struct for a sequence of base_msgs__srv__PNCMapService_Response.
typedef struct base_msgs__srv__PNCMapService_Response__Sequence
{
  base_msgs__srv__PNCMapService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} base_msgs__srv__PNCMapService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__STRUCT_H_
