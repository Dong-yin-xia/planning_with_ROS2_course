// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from base_msgs:msg/LocalSpeeds.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "base_msgs/msg/detail/local_speeds__rosidl_typesupport_introspection_c.h"
#include "base_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "base_msgs/msg/detail/local_speeds__functions.h"
#include "base_msgs/msg/detail/local_speeds__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `local_speeds`
#include "base_msgs/msg/local_speeds_point.h"
// Member `local_speeds`
#include "base_msgs/msg/detail/local_speeds_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  base_msgs__msg__LocalSpeeds__init(message_memory);
}

void base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_fini_function(void * message_memory)
{
  base_msgs__msg__LocalSpeeds__fini(message_memory);
}

size_t base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__size_function__LocalSpeeds__local_speeds(
  const void * untyped_member)
{
  const base_msgs__msg__LocalSpeedsPoint__Sequence * member =
    (const base_msgs__msg__LocalSpeedsPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__get_const_function__LocalSpeeds__local_speeds(
  const void * untyped_member, size_t index)
{
  const base_msgs__msg__LocalSpeedsPoint__Sequence * member =
    (const base_msgs__msg__LocalSpeedsPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__get_function__LocalSpeeds__local_speeds(
  void * untyped_member, size_t index)
{
  base_msgs__msg__LocalSpeedsPoint__Sequence * member =
    (base_msgs__msg__LocalSpeedsPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__fetch_function__LocalSpeeds__local_speeds(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const base_msgs__msg__LocalSpeedsPoint * item =
    ((const base_msgs__msg__LocalSpeedsPoint *)
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__get_const_function__LocalSpeeds__local_speeds(untyped_member, index));
  base_msgs__msg__LocalSpeedsPoint * value =
    (base_msgs__msg__LocalSpeedsPoint *)(untyped_value);
  *value = *item;
}

void base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__assign_function__LocalSpeeds__local_speeds(
  void * untyped_member, size_t index, const void * untyped_value)
{
  base_msgs__msg__LocalSpeedsPoint * item =
    ((base_msgs__msg__LocalSpeedsPoint *)
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__get_function__LocalSpeeds__local_speeds(untyped_member, index));
  const base_msgs__msg__LocalSpeedsPoint * value =
    (const base_msgs__msg__LocalSpeedsPoint *)(untyped_value);
  *item = *value;
}

bool base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__resize_function__LocalSpeeds__local_speeds(
  void * untyped_member, size_t size)
{
  base_msgs__msg__LocalSpeedsPoint__Sequence * member =
    (base_msgs__msg__LocalSpeedsPoint__Sequence *)(untyped_member);
  base_msgs__msg__LocalSpeedsPoint__Sequence__fini(member);
  return base_msgs__msg__LocalSpeedsPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base_msgs__msg__LocalSpeeds, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_speeds",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base_msgs__msg__LocalSpeeds, local_speeds),  // bytes offset in struct
    NULL,  // default value
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__size_function__LocalSpeeds__local_speeds,  // size() function pointer
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__get_const_function__LocalSpeeds__local_speeds,  // get_const(index) function pointer
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__get_function__LocalSpeeds__local_speeds,  // get(index) function pointer
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__fetch_function__LocalSpeeds__local_speeds,  // fetch(index, &value) function pointer
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__assign_function__LocalSpeeds__local_speeds,  // assign(index, value) function pointer
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__resize_function__LocalSpeeds__local_speeds  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_members = {
  "base_msgs__msg",  // message namespace
  "LocalSpeeds",  // message name
  2,  // number of fields
  sizeof(base_msgs__msg__LocalSpeeds),
  base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_member_array,  // message members
  base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_init_function,  // function to initialize message memory (memory has to be allocated)
  base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_type_support_handle = {
  0,
  &base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_base_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base_msgs, msg, LocalSpeeds)() {
  base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base_msgs, msg, LocalSpeedsPoint)();
  if (!base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_type_support_handle.typesupport_identifier) {
    base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &base_msgs__msg__LocalSpeeds__rosidl_typesupport_introspection_c__LocalSpeeds_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
