// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from base_msgs:msg/LocalTrajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "base_msgs/msg/detail/local_trajectory__rosidl_typesupport_introspection_c.h"
#include "base_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "base_msgs/msg/detail/local_trajectory__functions.h"
#include "base_msgs/msg/detail/local_trajectory__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `local_trajectory`
#include "base_msgs/msg/local_trajectory_point.h"
// Member `local_trajectory`
#include "base_msgs/msg/detail/local_trajectory_point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  base_msgs__msg__LocalTrajectory__init(message_memory);
}

void base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_fini_function(void * message_memory)
{
  base_msgs__msg__LocalTrajectory__fini(message_memory);
}

size_t base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__size_function__LocalTrajectory__local_trajectory(
  const void * untyped_member)
{
  const base_msgs__msg__LocalTrajectoryPoint__Sequence * member =
    (const base_msgs__msg__LocalTrajectoryPoint__Sequence *)(untyped_member);
  return member->size;
}

const void * base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__get_const_function__LocalTrajectory__local_trajectory(
  const void * untyped_member, size_t index)
{
  const base_msgs__msg__LocalTrajectoryPoint__Sequence * member =
    (const base_msgs__msg__LocalTrajectoryPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void * base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__get_function__LocalTrajectory__local_trajectory(
  void * untyped_member, size_t index)
{
  base_msgs__msg__LocalTrajectoryPoint__Sequence * member =
    (base_msgs__msg__LocalTrajectoryPoint__Sequence *)(untyped_member);
  return &member->data[index];
}

void base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__fetch_function__LocalTrajectory__local_trajectory(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const base_msgs__msg__LocalTrajectoryPoint * item =
    ((const base_msgs__msg__LocalTrajectoryPoint *)
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__get_const_function__LocalTrajectory__local_trajectory(untyped_member, index));
  base_msgs__msg__LocalTrajectoryPoint * value =
    (base_msgs__msg__LocalTrajectoryPoint *)(untyped_value);
  *value = *item;
}

void base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__assign_function__LocalTrajectory__local_trajectory(
  void * untyped_member, size_t index, const void * untyped_value)
{
  base_msgs__msg__LocalTrajectoryPoint * item =
    ((base_msgs__msg__LocalTrajectoryPoint *)
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__get_function__LocalTrajectory__local_trajectory(untyped_member, index));
  const base_msgs__msg__LocalTrajectoryPoint * value =
    (const base_msgs__msg__LocalTrajectoryPoint *)(untyped_value);
  *item = *value;
}

bool base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__resize_function__LocalTrajectory__local_trajectory(
  void * untyped_member, size_t size)
{
  base_msgs__msg__LocalTrajectoryPoint__Sequence * member =
    (base_msgs__msg__LocalTrajectoryPoint__Sequence *)(untyped_member);
  base_msgs__msg__LocalTrajectoryPoint__Sequence__fini(member);
  return base_msgs__msg__LocalTrajectoryPoint__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base_msgs__msg__LocalTrajectory, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_trajectory",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(base_msgs__msg__LocalTrajectory, local_trajectory),  // bytes offset in struct
    NULL,  // default value
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__size_function__LocalTrajectory__local_trajectory,  // size() function pointer
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__get_const_function__LocalTrajectory__local_trajectory,  // get_const(index) function pointer
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__get_function__LocalTrajectory__local_trajectory,  // get(index) function pointer
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__fetch_function__LocalTrajectory__local_trajectory,  // fetch(index, &value) function pointer
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__assign_function__LocalTrajectory__local_trajectory,  // assign(index, value) function pointer
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__resize_function__LocalTrajectory__local_trajectory  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_members = {
  "base_msgs__msg",  // message namespace
  "LocalTrajectory",  // message name
  2,  // number of fields
  sizeof(base_msgs__msg__LocalTrajectory),
  base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_member_array,  // message members
  base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_type_support_handle = {
  0,
  &base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_base_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base_msgs, msg, LocalTrajectory)() {
  base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, base_msgs, msg, LocalTrajectoryPoint)();
  if (!base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_type_support_handle.typesupport_identifier) {
    base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &base_msgs__msg__LocalTrajectory__rosidl_typesupport_introspection_c__LocalTrajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
