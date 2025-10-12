// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/local_speeds_point__rosidl_typesupport_fastrtps_cpp.hpp"
#include "base_msgs/msg/detail/local_speeds_point__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

namespace base_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
cdr_serialize(
  const base_msgs::msg::LocalSpeedsPoint & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: t
  cdr << ros_message.t;
  // Member: s_2path
  cdr << ros_message.s_2path;
  // Member: ds_dt_2path
  cdr << ros_message.ds_dt_2path;
  // Member: dds_dt_2path
  cdr << ros_message.dds_dt_2path;
  // Member: speed
  cdr << ros_message.speed;
  // Member: acceleration
  cdr << ros_message.acceleration;
  // Member: dacceleration
  cdr << ros_message.dacceleration;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  base_msgs::msg::LocalSpeedsPoint & ros_message)
{
  // Member: t
  cdr >> ros_message.t;

  // Member: s_2path
  cdr >> ros_message.s_2path;

  // Member: ds_dt_2path
  cdr >> ros_message.ds_dt_2path;

  // Member: dds_dt_2path
  cdr >> ros_message.dds_dt_2path;

  // Member: speed
  cdr >> ros_message.speed;

  // Member: acceleration
  cdr >> ros_message.acceleration;

  // Member: dacceleration
  cdr >> ros_message.dacceleration;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
get_serialized_size(
  const base_msgs::msg::LocalSpeedsPoint & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: t
  {
    size_t item_size = sizeof(ros_message.t);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: s_2path
  {
    size_t item_size = sizeof(ros_message.s_2path);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ds_dt_2path
  {
    size_t item_size = sizeof(ros_message.ds_dt_2path);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: dds_dt_2path
  {
    size_t item_size = sizeof(ros_message.dds_dt_2path);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: speed
  {
    size_t item_size = sizeof(ros_message.speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: acceleration
  {
    size_t item_size = sizeof(ros_message.acceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: dacceleration
  {
    size_t item_size = sizeof(ros_message.dacceleration);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
max_serialized_size_LocalSpeedsPoint(
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


  // Member: t
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: s_2path
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ds_dt_2path
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: dds_dt_2path
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: acceleration
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: dacceleration
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
    using DataType = base_msgs::msg::LocalSpeedsPoint;
    is_plain =
      (
      offsetof(DataType, dacceleration) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _LocalSpeedsPoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const base_msgs::msg::LocalSpeedsPoint *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LocalSpeedsPoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<base_msgs::msg::LocalSpeedsPoint *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LocalSpeedsPoint__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const base_msgs::msg::LocalSpeedsPoint *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LocalSpeedsPoint__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_LocalSpeedsPoint(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _LocalSpeedsPoint__callbacks = {
  "base_msgs::msg",
  "LocalSpeedsPoint",
  _LocalSpeedsPoint__cdr_serialize,
  _LocalSpeedsPoint__cdr_deserialize,
  _LocalSpeedsPoint__get_serialized_size,
  _LocalSpeedsPoint__max_serialized_size
};

static rosidl_message_type_support_t _LocalSpeedsPoint__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LocalSpeedsPoint__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace base_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_base_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<base_msgs::msg::LocalSpeedsPoint>()
{
  return &base_msgs::msg::typesupport_fastrtps_cpp::_LocalSpeedsPoint__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, base_msgs, msg, LocalSpeedsPoint)() {
  return &base_msgs::msg::typesupport_fastrtps_cpp::_LocalSpeedsPoint__handle;
}

#ifdef __cplusplus
}
#endif
