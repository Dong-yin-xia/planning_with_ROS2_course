// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from base_msgs:msg/Referline.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "base_msgs/msg/detail/referline__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace base_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Referline_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) base_msgs::msg::Referline(_init);
}

void Referline_fini_function(void * message_memory)
{
  auto typed_message = static_cast<base_msgs::msg::Referline *>(message_memory);
  typed_message->~Referline();
}

size_t size_function__Referline__refer_line(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<base_msgs::msg::ReferlinePoint> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Referline__refer_line(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<base_msgs::msg::ReferlinePoint> *>(untyped_member);
  return &member[index];
}

void * get_function__Referline__refer_line(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<base_msgs::msg::ReferlinePoint> *>(untyped_member);
  return &member[index];
}

void fetch_function__Referline__refer_line(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const base_msgs::msg::ReferlinePoint *>(
    get_const_function__Referline__refer_line(untyped_member, index));
  auto & value = *reinterpret_cast<base_msgs::msg::ReferlinePoint *>(untyped_value);
  value = item;
}

void assign_function__Referline__refer_line(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<base_msgs::msg::ReferlinePoint *>(
    get_function__Referline__refer_line(untyped_member, index));
  const auto & value = *reinterpret_cast<const base_msgs::msg::ReferlinePoint *>(untyped_value);
  item = value;
}

void resize_function__Referline__refer_line(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<base_msgs::msg::ReferlinePoint> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Referline_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base_msgs::msg::Referline, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "refer_line",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<base_msgs::msg::ReferlinePoint>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base_msgs::msg::Referline, refer_line),  // bytes offset in struct
    nullptr,  // default value
    size_function__Referline__refer_line,  // size() function pointer
    get_const_function__Referline__refer_line,  // get_const(index) function pointer
    get_function__Referline__refer_line,  // get(index) function pointer
    fetch_function__Referline__refer_line,  // fetch(index, &value) function pointer
    assign_function__Referline__refer_line,  // assign(index, value) function pointer
    resize_function__Referline__refer_line  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Referline_message_members = {
  "base_msgs::msg",  // message namespace
  "Referline",  // message name
  2,  // number of fields
  sizeof(base_msgs::msg::Referline),
  Referline_message_member_array,  // message members
  Referline_init_function,  // function to initialize message memory (memory has to be allocated)
  Referline_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Referline_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Referline_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace base_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<base_msgs::msg::Referline>()
{
  return &::base_msgs::msg::rosidl_typesupport_introspection_cpp::Referline_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, base_msgs, msg, Referline)() {
  return &::base_msgs::msg::rosidl_typesupport_introspection_cpp::Referline_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
