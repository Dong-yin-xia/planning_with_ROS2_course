// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base_msgs:msg/PlotInfo.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/plot_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `trajectory_info`
#include "base_msgs/msg/detail/local_trajectory__functions.h"
// Member `obs_info`
#include "base_msgs/msg/detail/obs_info__functions.h"

bool
base_msgs__msg__PlotInfo__init(base_msgs__msg__PlotInfo * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    base_msgs__msg__PlotInfo__fini(msg);
    return false;
  }
  // trajectory_info
  if (!base_msgs__msg__LocalTrajectory__init(&msg->trajectory_info)) {
    base_msgs__msg__PlotInfo__fini(msg);
    return false;
  }
  // obs_info
  if (!base_msgs__msg__ObsInfo__Sequence__init(&msg->obs_info, 0)) {
    base_msgs__msg__PlotInfo__fini(msg);
    return false;
  }
  return true;
}

void
base_msgs__msg__PlotInfo__fini(base_msgs__msg__PlotInfo * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // trajectory_info
  base_msgs__msg__LocalTrajectory__fini(&msg->trajectory_info);
  // obs_info
  base_msgs__msg__ObsInfo__Sequence__fini(&msg->obs_info);
}

bool
base_msgs__msg__PlotInfo__are_equal(const base_msgs__msg__PlotInfo * lhs, const base_msgs__msg__PlotInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // trajectory_info
  if (!base_msgs__msg__LocalTrajectory__are_equal(
      &(lhs->trajectory_info), &(rhs->trajectory_info)))
  {
    return false;
  }
  // obs_info
  if (!base_msgs__msg__ObsInfo__Sequence__are_equal(
      &(lhs->obs_info), &(rhs->obs_info)))
  {
    return false;
  }
  return true;
}

bool
base_msgs__msg__PlotInfo__copy(
  const base_msgs__msg__PlotInfo * input,
  base_msgs__msg__PlotInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // trajectory_info
  if (!base_msgs__msg__LocalTrajectory__copy(
      &(input->trajectory_info), &(output->trajectory_info)))
  {
    return false;
  }
  // obs_info
  if (!base_msgs__msg__ObsInfo__Sequence__copy(
      &(input->obs_info), &(output->obs_info)))
  {
    return false;
  }
  return true;
}

base_msgs__msg__PlotInfo *
base_msgs__msg__PlotInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__PlotInfo * msg = (base_msgs__msg__PlotInfo *)allocator.allocate(sizeof(base_msgs__msg__PlotInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__msg__PlotInfo));
  bool success = base_msgs__msg__PlotInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__msg__PlotInfo__destroy(base_msgs__msg__PlotInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__msg__PlotInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__msg__PlotInfo__Sequence__init(base_msgs__msg__PlotInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__PlotInfo * data = NULL;

  if (size) {
    data = (base_msgs__msg__PlotInfo *)allocator.zero_allocate(size, sizeof(base_msgs__msg__PlotInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__msg__PlotInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__msg__PlotInfo__fini(&data[i - 1]);
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
base_msgs__msg__PlotInfo__Sequence__fini(base_msgs__msg__PlotInfo__Sequence * array)
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
      base_msgs__msg__PlotInfo__fini(&array->data[i]);
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

base_msgs__msg__PlotInfo__Sequence *
base_msgs__msg__PlotInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__PlotInfo__Sequence * array = (base_msgs__msg__PlotInfo__Sequence *)allocator.allocate(sizeof(base_msgs__msg__PlotInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__msg__PlotInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__msg__PlotInfo__Sequence__destroy(base_msgs__msg__PlotInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__msg__PlotInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__msg__PlotInfo__Sequence__are_equal(const base_msgs__msg__PlotInfo__Sequence * lhs, const base_msgs__msg__PlotInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__msg__PlotInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__msg__PlotInfo__Sequence__copy(
  const base_msgs__msg__PlotInfo__Sequence * input,
  base_msgs__msg__PlotInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__msg__PlotInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__msg__PlotInfo * data =
      (base_msgs__msg__PlotInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__msg__PlotInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__msg__PlotInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__msg__PlotInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
