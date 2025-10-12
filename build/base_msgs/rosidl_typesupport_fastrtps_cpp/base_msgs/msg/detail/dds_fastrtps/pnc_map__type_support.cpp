// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from base_msgs:msg/PNCMap.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/pnc_map__rosidl_typesupport_fastrtps_cpp.hpp"
#include "base_msgs/msg/detail/pnc_map__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs

namespace visualization_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const visualization_msgs::msg::Marker &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  visualization_msgs::msg::Marker &);
size_t get_serialized_size(
  const visualization_msgs::msg::Marker &,
  size_t current_alignment);
size_t
max_serialized_size_Marker(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace visualization_msgs

// functions for visualization_msgs::msg::Marker already declared above

// functions for visualization_msgs::msg::Marker already declared above


namespace base_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
cdr_serialize(
  const base_msgs::msg::PNCMap & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: road_length
  cdr << ros_message.road_length;
  // Member: road_half_width
  cdr << ros_message.road_half_width;
  // Member: left_boundary
  visualization_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.left_boundary,
    cdr);
  // Member: right_boundary
  visualization_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.right_boundary,
    cdr);
  // Member: midline
  visualization_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.midline,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  base_msgs::msg::PNCMap & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: road_length
  cdr >> ros_message.road_length;

  // Member: road_half_width
  cdr >> ros_message.road_half_width;

  // Member: left_boundary
  visualization_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.left_boundary);

  // Member: right_boundary
  visualization_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.right_boundary);

  // Member: midline
  visualization_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.midline);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
get_serialized_size(
  const base_msgs::msg::PNCMap & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: road_length
  {
    size_t item_size = sizeof(ros_message.road_length);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: road_half_width
  {
    size_t item_size = sizeof(ros_message.road_half_width);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: left_boundary

  current_alignment +=
    visualization_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.left_boundary, current_alignment);
  // Member: right_boundary

  current_alignment +=
    visualization_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.right_boundary, current_alignment);
  // Member: midline

  current_alignment +=
    visualization_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.midline, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_base_msgs
max_serialized_size_PNCMap(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: road_length
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: road_half_width
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: left_boundary
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        visualization_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Marker(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: right_boundary
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        visualization_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Marker(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: midline
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        visualization_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Marker(
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
    using DataType = base_msgs::msg::PNCMap;
    is_plain =
      (
      offsetof(DataType, midline) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _PNCMap__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const base_msgs::msg::PNCMap *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _PNCMap__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<base_msgs::msg::PNCMap *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _PNCMap__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const base_msgs::msg::PNCMap *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _PNCMap__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_PNCMap(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _PNCMap__callbacks = {
  "base_msgs::msg",
  "PNCMap",
  _PNCMap__cdr_serialize,
  _PNCMap__cdr_deserialize,
  _PNCMap__get_serialized_size,
  _PNCMap__max_serialized_size
};

static rosidl_message_type_support_t _PNCMap__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_PNCMap__callbacks,
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
get_message_type_support_handle<base_msgs::msg::PNCMap>()
{
  return &base_msgs::msg::typesupport_fastrtps_cpp::_PNCMap__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, base_msgs, msg, PNCMap)() {
  return &base_msgs::msg::typesupport_fastrtps_cpp::_PNCMap__handle;
}

#ifdef __cplusplus
}
#endif
