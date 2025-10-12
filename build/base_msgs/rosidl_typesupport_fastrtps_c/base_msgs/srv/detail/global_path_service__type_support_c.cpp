// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from base_msgs:srv/GlobalPathService.idl
// generated code does not contain a copyright notice
#include "base_msgs/srv/detail/global_path_service__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "base_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "base_msgs/srv/detail/global_path_service__struct.h"
#include "base_msgs/srv/detail/global_path_service__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "base_msgs/msg/detail/pnc_map__functions.h"  // pnc_map

// forward declare type support functions
size_t get_serialized_size_base_msgs__msg__PNCMap(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_base_msgs__msg__PNCMap(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, base_msgs, msg, PNCMap)();


using _GlobalPathService_Request__ros_msg_type = base_msgs__srv__GlobalPathService_Request;

static bool _GlobalPathService_Request__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GlobalPathService_Request__ros_msg_type * ros_message = static_cast<const _GlobalPathService_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: global_planner_type
  {
    cdr << ros_message->global_planner_type;
  }

  // Field name: pnc_map
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, base_msgs, msg, PNCMap
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->pnc_map, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GlobalPathService_Request__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GlobalPathService_Request__ros_msg_type * ros_message = static_cast<_GlobalPathService_Request__ros_msg_type *>(untyped_ros_message);
  // Field name: global_planner_type
  {
    cdr >> ros_message->global_planner_type;
  }

  // Field name: pnc_map
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, base_msgs, msg, PNCMap
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->pnc_map))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_base_msgs
size_t get_serialized_size_base_msgs__srv__GlobalPathService_Request(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GlobalPathService_Request__ros_msg_type * ros_message = static_cast<const _GlobalPathService_Request__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name global_planner_type
  {
    size_t item_size = sizeof(ros_message->global_planner_type);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pnc_map

  current_alignment += get_serialized_size_base_msgs__msg__PNCMap(
    &(ros_message->pnc_map), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GlobalPathService_Request__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_base_msgs__srv__GlobalPathService_Request(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_base_msgs
size_t max_serialized_size_base_msgs__srv__GlobalPathService_Request(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: global_planner_type
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: pnc_map
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_base_msgs__msg__PNCMap(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = base_msgs__srv__GlobalPathService_Request;
    is_plain =
      (
      offsetof(DataType, pnc_map) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GlobalPathService_Request__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_base_msgs__srv__GlobalPathService_Request(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GlobalPathService_Request = {
  "base_msgs::srv",
  "GlobalPathService_Request",
  _GlobalPathService_Request__cdr_serialize,
  _GlobalPathService_Request__cdr_deserialize,
  _GlobalPathService_Request__get_serialized_size,
  _GlobalPathService_Request__max_serialized_size
};

static rosidl_message_type_support_t _GlobalPathService_Request__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GlobalPathService_Request,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, base_msgs, srv, GlobalPathService_Request)() {
  return &_GlobalPathService_Request__type_support;
}

#if defined(__cplusplus)
}
#endif

// already included above
// #include <cassert>
// already included above
// #include <limits>
// already included above
// #include <string>
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
// already included above
// #include "base_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
// already included above
// #include "base_msgs/srv/detail/global_path_service__struct.h"
// already included above
// #include "base_msgs/srv/detail/global_path_service__functions.h"
// already included above
// #include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "nav_msgs/msg/detail/path__functions.h"  // global_path

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_base_msgs
size_t get_serialized_size_nav_msgs__msg__Path(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_base_msgs
size_t max_serialized_size_nav_msgs__msg__Path(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_base_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, nav_msgs, msg, Path)();


using _GlobalPathService_Response__ros_msg_type = base_msgs__srv__GlobalPathService_Response;

static bool _GlobalPathService_Response__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _GlobalPathService_Response__ros_msg_type * ros_message = static_cast<const _GlobalPathService_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: global_path
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, nav_msgs, msg, Path
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->global_path, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _GlobalPathService_Response__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _GlobalPathService_Response__ros_msg_type * ros_message = static_cast<_GlobalPathService_Response__ros_msg_type *>(untyped_ros_message);
  // Field name: global_path
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, nav_msgs, msg, Path
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->global_path))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_base_msgs
size_t get_serialized_size_base_msgs__srv__GlobalPathService_Response(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _GlobalPathService_Response__ros_msg_type * ros_message = static_cast<const _GlobalPathService_Response__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name global_path

  current_alignment += get_serialized_size_nav_msgs__msg__Path(
    &(ros_message->global_path), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _GlobalPathService_Response__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_base_msgs__srv__GlobalPathService_Response(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_base_msgs
size_t max_serialized_size_base_msgs__srv__GlobalPathService_Response(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: global_path
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_nav_msgs__msg__Path(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = base_msgs__srv__GlobalPathService_Response;
    is_plain =
      (
      offsetof(DataType, global_path) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _GlobalPathService_Response__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_base_msgs__srv__GlobalPathService_Response(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_GlobalPathService_Response = {
  "base_msgs::srv",
  "GlobalPathService_Response",
  _GlobalPathService_Response__cdr_serialize,
  _GlobalPathService_Response__cdr_deserialize,
  _GlobalPathService_Response__get_serialized_size,
  _GlobalPathService_Response__max_serialized_size
};

static rosidl_message_type_support_t _GlobalPathService_Response__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_GlobalPathService_Response,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, base_msgs, srv, GlobalPathService_Response)() {
  return &_GlobalPathService_Response__type_support;
}

#if defined(__cplusplus)
}
#endif

#include "rosidl_typesupport_fastrtps_cpp/service_type_support.h"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_fastrtps_c/identifier.h"
// already included above
// #include "base_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "base_msgs/srv/global_path_service.h"

#if defined(__cplusplus)
extern "C"
{
#endif

static service_type_support_callbacks_t GlobalPathService__callbacks = {
  "base_msgs::srv",
  "GlobalPathService",
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, base_msgs, srv, GlobalPathService_Request)(),
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, base_msgs, srv, GlobalPathService_Response)(),
};

static rosidl_service_type_support_t GlobalPathService__handle = {
  rosidl_typesupport_fastrtps_c__identifier,
  &GlobalPathService__callbacks,
  get_service_typesupport_handle_function,
};

const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, base_msgs, srv, GlobalPathService)() {
  return &GlobalPathService__handle;
}

#if defined(__cplusplus)
}
#endif
