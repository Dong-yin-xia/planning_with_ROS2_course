// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from base_msgs:msg/LocalPathPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__FUNCTIONS_H_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "base_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "base_msgs/msg/detail/local_path_point__struct.h"

/// Initialize msg/LocalPathPoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * base_msgs__msg__LocalPathPoint
 * )) before or use
 * base_msgs__msg__LocalPathPoint__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalPathPoint__init(base_msgs__msg__LocalPathPoint * msg);

/// Finalize msg/LocalPathPoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalPathPoint__fini(base_msgs__msg__LocalPathPoint * msg);

/// Create msg/LocalPathPoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * base_msgs__msg__LocalPathPoint__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
base_msgs__msg__LocalPathPoint *
base_msgs__msg__LocalPathPoint__create();

/// Destroy msg/LocalPathPoint message.
/**
 * It calls
 * base_msgs__msg__LocalPathPoint__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalPathPoint__destroy(base_msgs__msg__LocalPathPoint * msg);

/// Check for msg/LocalPathPoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalPathPoint__are_equal(const base_msgs__msg__LocalPathPoint * lhs, const base_msgs__msg__LocalPathPoint * rhs);

/// Copy a msg/LocalPathPoint message.
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
base_msgs__msg__LocalPathPoint__copy(
  const base_msgs__msg__LocalPathPoint * input,
  base_msgs__msg__LocalPathPoint * output);

/// Initialize array of msg/LocalPathPoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * base_msgs__msg__LocalPathPoint__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalPathPoint__Sequence__init(base_msgs__msg__LocalPathPoint__Sequence * array, size_t size);

/// Finalize array of msg/LocalPathPoint messages.
/**
 * It calls
 * base_msgs__msg__LocalPathPoint__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalPathPoint__Sequence__fini(base_msgs__msg__LocalPathPoint__Sequence * array);

/// Create array of msg/LocalPathPoint messages.
/**
 * It allocates the memory for the array and calls
 * base_msgs__msg__LocalPathPoint__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
base_msgs__msg__LocalPathPoint__Sequence *
base_msgs__msg__LocalPathPoint__Sequence__create(size_t size);

/// Destroy array of msg/LocalPathPoint messages.
/**
 * It calls
 * base_msgs__msg__LocalPathPoint__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
void
base_msgs__msg__LocalPathPoint__Sequence__destroy(base_msgs__msg__LocalPathPoint__Sequence * array);

/// Check for msg/LocalPathPoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_base_msgs
bool
base_msgs__msg__LocalPathPoint__Sequence__are_equal(const base_msgs__msg__LocalPathPoint__Sequence * lhs, const base_msgs__msg__LocalPathPoint__Sequence * rhs);

/// Copy an array of msg/LocalPathPoint messages.
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
base_msgs__msg__LocalPathPoint__Sequence__copy(
  const base_msgs__msg__LocalPathPoint__Sequence * input,
  base_msgs__msg__LocalPathPoint__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__FUNCTIONS_H_
