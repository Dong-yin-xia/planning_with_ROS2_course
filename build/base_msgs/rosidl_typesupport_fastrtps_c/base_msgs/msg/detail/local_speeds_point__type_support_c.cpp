// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/local_speeds_point__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "base_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "base_msgs/msg/detail/local_speeds_point__struct.h"
#include "base_msgs/msg/detail/local_speeds_point__functions.h"
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


// forward declare type support functions


using _LocalSpeedsPoint__ros_msg_type = base_msgs__msg__LocalSpeedsPoint;

static bool _LocalSpeedsPoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _LocalSpeedsPoint__ros_msg_type * ros_message = static_cast<const _LocalSpeedsPoint__ros_msg_type *>(untyped_ros_message);
  // Field name: t
  {
    cdr << ros_message->t;
  }

  // Field name: s_2path
  {
    cdr << ros_message->s_2path;
  }

  // Field name: ds_dt_2path
  {
    cdr << ros_message->ds_dt_2path;
  }

  // Field name: dds_dt_2path
  {
    cdr << ros_message->dds_dt_2path;
  }

  // Field name: speed
  {
    cdr << ros_message->speed;
  }

  // Field name: acceleration
  {
    cdr << ros_message->acceleration;
  }

  // Field name: dacceleration
  {
    cdr << ros_message->dacceleration;
  }

  return true;
}

static bool _LocalSpeedsPoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _LocalSpeedsPoint__ros_msg_type * ros_message = static_cast<_LocalSpeedsPoint__ros_msg_type *>(untyped_ros_message);
  // Field name: t
  {
    cdr >> ros_message->t;
  }

  // Field name: s_2path
  {
    cdr >> ros_message->s_2path;
  }

  // Field name: ds_dt_2path
  {
    cdr >> ros_message->ds_dt_2path;
  }

  // Field name: dds_dt_2path
  {
    cdr >> ros_message->dds_dt_2path;
  }

  // Field name: speed
  {
    cdr >> ros_message->speed;
  }

  // Field name: acceleration
  {
    cdr >> ros_message->acceleration;
  }

  // Field name: dacceleration
  {
    cdr >> ros_message->dacceleration;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_base_msgs
size_t get_serialized_size_base_msgs__msg__LocalSpeedsPoint(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _LocalSpeedsPoint__ros_msg_type * ros_message = static_cast<const _LocalSpeedsPoint__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name t
  {
    size_t item_size = sizeof(ros_message->t);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name s_2path
  {
    size_t item_size = sizeof(ros_message->s_2path);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ds_dt_2path
  {
    size_t item_size = sizeof(ros_message->ds_dt_2path);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dds_dt_2path
  {
    size_t item_size = sizeof(ros_message->dds_dt_2path);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name speed
  {
    size_t item_size = sizeof(ros_message->speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name acceleration
  {
    size_t item_size = sizeof(ros_message->acceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name dacceleration
  {
    size_t item_size = sizeof(ros_message->dacceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _LocalSpeedsPoint__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_base_msgs__msg__LocalSpeedsPoint(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_base_msgs
size_t max_serialized_size_base_msgs__msg__LocalSpeedsPoint(
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

  // member: t
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: s_2path
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ds_dt_2path
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: dds_dt_2path
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: acceleration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: dacceleration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = base_msgs__msg__LocalSpeedsPoint;
    is_plain =
      (
      offsetof(DataType, dacceleration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _LocalSpeedsPoint__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_base_msgs__msg__LocalSpeedsPoint(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_LocalSpeedsPoint = {
  "base_msgs::msg",
  "LocalSpeedsPoint",
  _LocalSpeedsPoint__cdr_serialize,
  _LocalSpeedsPoint__cdr_deserialize,
  _LocalSpeedsPoint__get_serialized_size,
  _LocalSpeedsPoint__max_serialized_size
};

static rosidl_message_type_support_t _LocalSpeedsPoint__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_LocalSpeedsPoint,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, base_msgs, msg, LocalSpeedsPoint)() {
  return &_LocalSpeedsPoint__type_support;
}

#if defined(__cplusplus)
}
#endif
