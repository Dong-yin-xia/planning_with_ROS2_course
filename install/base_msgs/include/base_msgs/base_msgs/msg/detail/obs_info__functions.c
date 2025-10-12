// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base_msgs:msg/ObsInfo.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/obs_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
base_msgs__msg__ObsInfo__init(base_msgs__msg__ObsInfo * msg)
{
  if (!msg) {
    return false;
  }
  // obs_length
  // obs_width
  // l
  // s
  // s_2path
  // ds_dt_2path
  // t_in
  // t_out
  return true;
}

void
base_msgs__msg__ObsInfo__fini(base_msgs__msg__ObsInfo * msg)
{
  if (!msg) {
    return;
  }
  // obs_length
  // obs_width
  // l
  // s
  // s_2path
  // ds_dt_2path
  // t_in
  // t_out
}

bool
base_msgs__msg__ObsInfo__are_equal(const base_msgs__msg__ObsInfo * lhs, const base_msgs__msg__ObsInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // obs_length
  if (lhs->obs_length != rhs->obs_length) {
    return false;
  }
  // obs_width
  if (lhs->obs_width != rhs->obs_width) {
    return false;
  }
  // l
  if (lhs->l != rhs->l) {
    return false;
  }
  // s
  if (lhs->s != rhs->s) {
    return false;
  }
  // s_2path
  if (lhs->s_2path != rhs->s_2path) {
    return false;
  }
  // ds_dt_2path
  if (lhs->ds_dt_2path != rhs->ds_dt_2path) {
    return false;
  }
  // t_in
  if (lhs->t_in != rhs->t_in) {
    return false;
  }
  // t_out
  if (lhs->t_out != rhs->t_out) {
    return false;
  }
  return true;
}

bool
base_msgs__msg__ObsInfo__copy(
  const base_msgs__msg__ObsInfo * input,
  base_msgs__msg__ObsInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // obs_length
  output->obs_length = input->obs_length;
  // obs_width
  output->obs_width = input->obs_width;
  // l
  output->l = input->l;
  // s
  output->s = input->s;
  // s_2path
  output->s_2path = input->s_2path;
  // ds_dt_2path
  output->ds_dt_2path = input->ds_dt_2path;
  // t_in
  output->t_in = input->t_in;
  // t_out
  output->t_out = input->t_out;
  return true;
}

base_msgs__msg__ObsInfo *
base_msgs__msg__ObsInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__ObsInfo * msg = (base_msgs__msg__ObsInfo *)allocator.allocate(sizeof(base_msgs__msg__ObsInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__msg__ObsInfo));
  bool success = base_msgs__msg__ObsInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__msg__ObsInfo__destroy(base_msgs__msg__ObsInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__msg__ObsInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__msg__ObsInfo__Sequence__init(base_msgs__msg__ObsInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__ObsInfo * data = NULL;

  if (size) {
    data = (base_msgs__msg__ObsInfo *)allocator.zero_allocate(size, sizeof(base_msgs__msg__ObsInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__msg__ObsInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__msg__ObsInfo__fini(&data[i - 1]);
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
base_msgs__msg__ObsInfo__Sequence__fini(base_msgs__msg__ObsInfo__Sequence * array)
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
      base_msgs__msg__ObsInfo__fini(&array->data[i]);
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

base_msgs__msg__ObsInfo__Sequence *
base_msgs__msg__ObsInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__ObsInfo__Sequence * array = (base_msgs__msg__ObsInfo__Sequence *)allocator.allocate(sizeof(base_msgs__msg__ObsInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__msg__ObsInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__msg__ObsInfo__Sequence__destroy(base_msgs__msg__ObsInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__msg__ObsInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__msg__ObsInfo__Sequence__are_equal(const base_msgs__msg__ObsInfo__Sequence * lhs, const base_msgs__msg__ObsInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__msg__ObsInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__msg__ObsInfo__Sequence__copy(
  const base_msgs__msg__ObsInfo__Sequence * input,
  base_msgs__msg__ObsInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__msg__ObsInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__msg__ObsInfo * data =
      (base_msgs__msg__ObsInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__msg__ObsInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__msg__ObsInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__msg__ObsInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
