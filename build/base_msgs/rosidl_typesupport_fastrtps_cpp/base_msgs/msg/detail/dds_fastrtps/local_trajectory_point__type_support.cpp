// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from base_msgs:msg/LocalTrajectoryPoint.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/local_trajectory_point__rosidl_typesupport_fastrtps_cpp.hpp"
#include "base_msgs/msg/detail/local_trajectory_point__struct.hpp"

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
bool cdr_serialize(
  const base_msgs::msg::LocalPathPoint &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  base_msgs::msg::LocalPathPoint &);
size_t get_serialized_size(
  const base_msgs::msg::LocalPathPoint &,
  size_t current_alignment);
size_t
max_serialized_size_LocalPathPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace base_msgs

namespace base_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const base_msgs::msg::LocalSpeedsPoint &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  base_msgs::msg::LocalSpeedsPoint &);
size_t get_serialized_size(
  const base_msgs::msg::LocalSpeedsPoint &,
  size_t current_alignment);
size_t
max_serialized_size_LocalSpeedsPoint(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace base_msgs


namespace base_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
cdr_serialize(
  const base_msgs::msg::LocalTrajectoryPoint & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: path_point
  base_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.path_point,
    cdr);
  // Member: speed_point
  base_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.speed_point,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  base_msgs::msg::LocalTrajectoryPoint & ros_message)
{
  // Member: path_point
  base_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.path_point);

  // Member: speed_point
  base_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.speed_point);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
get_serialized_size(
  const base_msgs::msg::LocalTrajectoryPoint & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: path_point

  current_alignment +=
    base_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.path_point, current_alignment);
  // Member: speed_point

  current_alignment +=
    base_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.speed_point, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
max_serialized_size_LocalTrajectoryPoint(
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


  // Member: path_point
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        base_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_LocalPathPoint(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: speed_point
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        base_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_LocalSpeedsPoint(
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
    using DataType = base_msgs::msg::LocalTrajectoryPoint;
    is_plain =
      (
      offsetof(DataType, speed_point) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _LocalTrajectoryPoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const base_msgs::msg::LocalTrajectoryPoint *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _LocalTrajectoryPoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<base_msgs::msg::LocalTrajectoryPoint *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _LocalTrajectoryPoint__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const base_msgs::msg::LocalTrajectoryPoint *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _LocalTrajectoryPoint__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_LocalTrajectoryPoint(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _LocalTrajectoryPoint__callbacks = {
  "base_msgs::msg",
  "LocalTrajectoryPoint",
  _LocalTrajectoryPoint__cdr_serialize,
  _LocalTrajectoryPoint__cdr_deserialize,
  _LocalTrajectoryPoint__get_serialized_size,
  _LocalTrajectoryPoint__max_serialized_size
};

static rosidl_message_type_support_t _LocalTrajectoryPoint__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_LocalTrajectoryPoint__callbacks,
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
get_message_type_support_handle<base_msgs::msg::LocalTrajectoryPoint>()
{
  return &base_msgs::msg::typesupport_fastrtps_cpp::_LocalTrajectoryPoint__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, base_msgs, msg, LocalTrajectoryPoint)() {
  return &base_msgs::msg::typesupport_fastrtps_cpp::_LocalTrajectoryPoint__handle;
}

#ifdef __cplusplus
}
#endif
