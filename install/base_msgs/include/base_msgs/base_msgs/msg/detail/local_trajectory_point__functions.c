// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base_msgs:msg/LocalTrajectoryPoint.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/local_trajectory_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `path_point`
#include "base_msgs/msg/detail/local_path_point__functions.h"
// Member `speed_point`
#include "base_msgs/msg/detail/local_speeds_point__functions.h"

bool
base_msgs__msg__LocalTrajectoryPoint__init(base_msgs__msg__LocalTrajectoryPoint * msg)
{
  if (!msg) {
    return false;
  }
  // path_point
  if (!base_msgs__msg__LocalPathPoint__init(&msg->path_point)) {
    base_msgs__msg__LocalTrajectoryPoint__fini(msg);
    return false;
  }
  // speed_point
  if (!base_msgs__msg__LocalSpeedsPoint__init(&msg->speed_point)) {
    base_msgs__msg__LocalTrajectoryPoint__fini(msg);
    return false;
  }
  return true;
}

void
base_msgs__msg__LocalTrajectoryPoint__fini(base_msgs__msg__LocalTrajectoryPoint * msg)
{
  if (!msg) {
    return;
  }
  // path_point
  base_msgs__msg__LocalPathPoint__fini(&msg->path_point);
  // speed_point
  base_msgs__msg__LocalSpeedsPoint__fini(&msg->speed_point);
}

bool
base_msgs__msg__LocalTrajectoryPoint__are_equal(const base_msgs__msg__LocalTrajectoryPoint * lhs, const base_msgs__msg__LocalTrajectoryPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // path_point
  if (!base_msgs__msg__LocalPathPoint__are_equal(
      &(lhs->path_point), &(rhs->path_point)))
  {
    return false;
  }
  // speed_point
  if (!base_msgs__msg__LocalSpeedsPoint__are_equal(
      &(lhs->speed_point), &(rhs->speed_point)))
  {
    return false;
  }
  return true;
}

bool
base_msgs__msg__LocalTrajectoryPoint__copy(
  const base_msgs__msg__LocalTrajectoryPoint * input,
  base_msgs__msg__LocalTrajectoryPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // path_point
  if (!base_msgs__msg__LocalPathPoint__copy(
      &(input->path_point), &(output->path_point)))
  {
    return false;
  }
  // speed_point
  if (!base_msgs__msg__LocalSpeedsPoint__copy(
      &(input->speed_point), &(output->speed_point)))
  {
    return false;
  }
  return true;
}

base_msgs__msg__LocalTrajectoryPoint *
base_msgs__msg__LocalTrajectoryPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalTrajectoryPoint * msg = (base_msgs__msg__LocalTrajectoryPoint *)allocator.allocate(sizeof(base_msgs__msg__LocalTrajectoryPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__msg__LocalTrajectoryPoint));
  bool success = base_msgs__msg__LocalTrajectoryPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__msg__LocalTrajectoryPoint__destroy(base_msgs__msg__LocalTrajectoryPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__msg__LocalTrajectoryPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__msg__LocalTrajectoryPoint__Sequence__init(base_msgs__msg__LocalTrajectoryPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalTrajectoryPoint * data = NULL;

  if (size) {
    data = (base_msgs__msg__LocalTrajectoryPoint *)allocator.zero_allocate(size, sizeof(base_msgs__msg__LocalTrajectoryPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__msg__LocalTrajectoryPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__msg__LocalTrajectoryPoint__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
base_msgs__msg__LocalTrajectoryPoint__Sequence__fini(base_msgs__msg__LocalTrajectoryPoint__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      base_msgs__msg__LocalTrajectoryPoint__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

base_msgs__msg__LocalTrajectoryPoint__Sequence *
base_msgs__msg__LocalTrajectoryPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalTrajectoryPoint__Sequence * array = (base_msgs__msg__LocalTrajectoryPoint__Sequence *)allocator.allocate(sizeof(base_msgs__msg__LocalTrajectoryPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__msg__LocalTrajectoryPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__msg__LocalTrajectoryPoint__Sequence__destroy(base_msgs__msg__LocalTrajectoryPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__msg__LocalTrajectoryPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__msg__LocalTrajectoryPoint__Sequence__are_equal(const base_msgs__msg__LocalTrajectoryPoint__Sequence * lhs, const base_msgs__msg__LocalTrajectoryPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__msg__LocalTrajectoryPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__msg__LocalTrajectoryPoint__Sequence__copy(
  const base_msgs__msg__LocalTrajectoryPoint__Sequence * input,
  base_msgs__msg__LocalTrajectoryPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__msg__LocalTrajectoryPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__msg__LocalTrajectoryPoint * data =
      (base_msgs__msg__LocalTrajectoryPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__msg__LocalTrajectoryPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__msg__LocalTrajectoryPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__msg__LocalTrajectoryPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
