// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/local_speeds_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
base_msgs__msg__LocalSpeedsPoint__init(base_msgs__msg__LocalSpeedsPoint * msg)
{
  if (!msg) {
    return false;
  }
  // t
  // s_2path
  // ds_dt_2path
  // dds_dt_2path
  // speed
  // acceleration
  // dacceleration
  return true;
}

void
base_msgs__msg__LocalSpeedsPoint__fini(base_msgs__msg__LocalSpeedsPoint * msg)
{
  if (!msg) {
    return;
  }
  // t
  // s_2path
  // ds_dt_2path
  // dds_dt_2path
  // speed
  // acceleration
  // dacceleration
}

bool
base_msgs__msg__LocalSpeedsPoint__are_equal(const base_msgs__msg__LocalSpeedsPoint * lhs, const base_msgs__msg__LocalSpeedsPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // t
  if (lhs->t != rhs->t) {
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
  // dds_dt_2path
  if (lhs->dds_dt_2path != rhs->dds_dt_2path) {
    return false;
  }
  // speed
  if (lhs->speed != rhs->speed) {
    return false;
  }
  // acceleration
  if (lhs->acceleration != rhs->acceleration) {
    return false;
  }
  // dacceleration
  if (lhs->dacceleration != rhs->dacceleration) {
    return false;
  }
  return true;
}

bool
base_msgs__msg__LocalSpeedsPoint__copy(
  const base_msgs__msg__LocalSpeedsPoint * input,
  base_msgs__msg__LocalSpeedsPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // t
  output->t = input->t;
  // s_2path
  output->s_2path = input->s_2path;
  // ds_dt_2path
  output->ds_dt_2path = input->ds_dt_2path;
  // dds_dt_2path
  output->dds_dt_2path = input->dds_dt_2path;
  // speed
  output->speed = input->speed;
  // acceleration
  output->acceleration = input->acceleration;
  // dacceleration
  output->dacceleration = input->dacceleration;
  return true;
}

base_msgs__msg__LocalSpeedsPoint *
base_msgs__msg__LocalSpeedsPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalSpeedsPoint * msg = (base_msgs__msg__LocalSpeedsPoint *)allocator.allocate(sizeof(base_msgs__msg__LocalSpeedsPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__msg__LocalSpeedsPoint));
  bool success = base_msgs__msg__LocalSpeedsPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__msg__LocalSpeedsPoint__destroy(base_msgs__msg__LocalSpeedsPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__msg__LocalSpeedsPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__msg__LocalSpeedsPoint__Sequence__init(base_msgs__msg__LocalSpeedsPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalSpeedsPoint * data = NULL;

  if (size) {
    data = (base_msgs__msg__LocalSpeedsPoint *)allocator.zero_allocate(size, sizeof(base_msgs__msg__LocalSpeedsPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__msg__LocalSpeedsPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__msg__LocalSpeedsPoint__fini(&data[i - 1]);
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
base_msgs__msg__LocalSpeedsPoint__Sequence__fini(base_msgs__msg__LocalSpeedsPoint__Sequence * array)
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
      base_msgs__msg__LocalSpeedsPoint__fini(&array->data[i]);
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

base_msgs__msg__LocalSpeedsPoint__Sequence *
base_msgs__msg__LocalSpeedsPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalSpeedsPoint__Sequence * array = (base_msgs__msg__LocalSpeedsPoint__Sequence *)allocator.allocate(sizeof(base_msgs__msg__LocalSpeedsPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__msg__LocalSpeedsPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__msg__LocalSpeedsPoint__Sequence__destroy(base_msgs__msg__LocalSpeedsPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__msg__LocalSpeedsPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__msg__LocalSpeedsPoint__Sequence__are_equal(const base_msgs__msg__LocalSpeedsPoint__Sequence * lhs, const base_msgs__msg__LocalSpeedsPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__msg__LocalSpeedsPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__msg__LocalSpeedsPoint__Sequence__copy(
  const base_msgs__msg__LocalSpeedsPoint__Sequence * input,
  base_msgs__msg__LocalSpeedsPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__msg__LocalSpeedsPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__msg__LocalSpeedsPoint * data =
      (base_msgs__msg__LocalSpeedsPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__msg__LocalSpeedsPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__msg__LocalSpeedsPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__msg__LocalSpeedsPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
