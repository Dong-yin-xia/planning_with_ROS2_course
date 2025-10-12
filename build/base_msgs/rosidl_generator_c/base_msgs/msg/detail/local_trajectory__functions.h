// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from base_msgs:msg/LocalTrajectory.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY__FUNCTIONS_H_
#define BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "base_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "base_msgs/msg/detail/local_trajectory__struct.h"

/// Initialize msg/LocalTrajectory message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * base_msgs__msg__LocalTrajectory
 * )) before or use
 * base_msgs__msg__LocalTrajectory__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalTrajectory__init(base_msgs__msg__LocalTrajectory * msg);

/// Finalize msg/LocalTrajectory message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalTrajectory__fini(base_msgs__msg__LocalTrajectory * msg);

/// Create msg/LocalTrajectory message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * base_msgs__msg__LocalTrajectory__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
base_msgs__msg__LocalTrajectory *
base_msgs__msg__LocalTrajectory__create();

/// Destroy msg/LocalTrajectory message.
/**
 * It calls
 * base_msgs__msg__LocalTrajectory__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalTrajectory__destroy(base_msgs__msg__LocalTrajectory * msg);

/// Check for msg/LocalTrajectory message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalTrajectory__are_equal(const base_msgs__msg__LocalTrajectory * lhs, const base_msgs__msg__LocalTrajectory * rhs);

/// Copy a msg/LocalTrajectory message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalTrajectory__copy(
  const base_msgs__msg__LocalTrajectory * input,
  base_msgs__msg__LocalTrajectory * output);

/// Initialize array of msg/LocalTrajectory messages.
/**
 * It allocates the memory for the number of elements and calls
 * base_msgs__msg__LocalTrajectory__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalTrajectory__Sequence__init(base_msgs__msg__LocalTrajectory__Sequence * array, size_t size);

/// Finalize array of msg/LocalTrajectory messages.
/**
 * It calls
 * base_msgs__msg__LocalTrajectory__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalTrajectory__Sequence__fini(base_msgs__msg__LocalTrajectory__Sequence * array);

/// Create array of msg/LocalTrajectory messages.
/**
 * It allocates the memory for the array and calls
 * base_msgs__msg__LocalTrajectory__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
base_msgs__msg__LocalTrajectory__Sequence *
base_msgs__msg__LocalTrajectory__Sequence__create(size_t size);

/// Destroy array of msg/LocalTrajectory messages.
/**
 * It calls
 * base_msgs__msg__LocalTrajectory__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalTrajectory__Sequence__destroy(base_msgs__msg__LocalTrajectory__Sequence * array);

/// Check for msg/LocalTrajectory message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalTrajectory__Sequence__are_equal(const base_msgs__msg__LocalTrajectory__Sequence * lhs, const base_msgs__msg__LocalTrajectory__Sequence * rhs);

/// Copy an array of msg/LocalTrajectory messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalTrajectory__Sequence__copy(
  const base_msgs__msg__LocalTrajectory__Sequence * input,
  base_msgs__msg__LocalTrajectory__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY__FUNCTIONS_H_
