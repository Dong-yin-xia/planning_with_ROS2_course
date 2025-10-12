// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base_msgs:msg/PNCMap.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/pnc_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `left_boundary`
// Member `right_boundary`
// Member `midline`
#include "visualization_msgs/msg/detail/marker__functions.h"

bool
base_msgs__msg__PNCMap__init(base_msgs__msg__PNCMap * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    base_msgs__msg__PNCMap__fini(msg);
    return false;
  }
  // road_length
  // road_half_width
  // left_boundary
  if (!visualization_msgs__msg__Marker__init(&msg->left_boundary)) {
    base_msgs__msg__PNCMap__fini(msg);
    return false;
  }
  // right_boundary
  if (!visualization_msgs__msg__Marker__init(&msg->right_boundary)) {
    base_msgs__msg__PNCMap__fini(msg);
    return false;
  }
  // midline
  if (!visualization_msgs__msg__Marker__init(&msg->midline)) {
    base_msgs__msg__PNCMap__fini(msg);
    return false;
  }
  return true;
}

void
base_msgs__msg__PNCMap__fini(base_msgs__msg__PNCMap * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // road_length
  // road_half_width
  // left_boundary
  visualization_msgs__msg__Marker__fini(&msg->left_boundary);
  // right_boundary
  visualization_msgs__msg__Marker__fini(&msg->right_boundary);
  // midline
  visualization_msgs__msg__Marker__fini(&msg->midline);
}

bool
base_msgs__msg__PNCMap__are_equal(const base_msgs__msg__PNCMap * lhs, const base_msgs__msg__PNCMap * rhs)
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
  // road_length
  if (lhs->road_length != rhs->road_length) {
    return false;
  }
  // road_half_width
  if (lhs->road_half_width != rhs->road_half_width) {
    return false;
  }
  // left_boundary
  if (!visualization_msgs__msg__Marker__are_equal(
      &(lhs->left_boundary), &(rhs->left_boundary)))
  {
    return false;
  }
  // right_boundary
  if (!visualization_msgs__msg__Marker__are_equal(
      &(lhs->right_boundary), &(rhs->right_boundary)))
  {
    return false;
  }
  // midline
  if (!visualization_msgs__msg__Marker__are_equal(
      &(lhs->midline), &(rhs->midline)))
  {
    return false;
  }
  return true;
}

bool
base_msgs__msg__PNCMap__copy(
  const base_msgs__msg__PNCMap * input,
  base_msgs__msg__PNCMap * output)
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
  // road_length
  output->road_length = input->road_length;
  // road_half_width
  output->road_half_width = input->road_half_width;
  // left_boundary
  if (!visualization_msgs__msg__Marker__copy(
      &(input->left_boundary), &(output->left_boundary)))
  {
    return false;
  }
  // right_boundary
  if (!visualization_msgs__msg__Marker__copy(
      &(input->right_boundary), &(output->right_boundary)))
  {
    return false;
  }
  // midline
  if (!visualization_msgs__msg__Marker__copy(
      &(input->midline), &(output->midline)))
  {
    return false;
  }
  return true;
}

base_msgs__msg__PNCMap *
base_msgs__msg__PNCMap__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__PNCMap * msg = (base_msgs__msg__PNCMap *)allocator.allocate(sizeof(base_msgs__msg__PNCMap), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__msg__PNCMap));
  bool success = base_msgs__msg__PNCMap__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__msg__PNCMap__destroy(base_msgs__msg__PNCMap * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__msg__PNCMap__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__msg__PNCMap__Sequence__init(base_msgs__msg__PNCMap__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__PNCMap * data = NULL;

  if (size) {
    data = (base_msgs__msg__PNCMap *)allocator.zero_allocate(size, sizeof(base_msgs__msg__PNCMap), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__msg__PNCMap__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__msg__PNCMap__fini(&data[i - 1]);
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
base_msgs__msg__PNCMap__Sequence__fini(base_msgs__msg__PNCMap__Sequence * array)
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
      base_msgs__msg__PNCMap__fini(&array->data[i]);
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

base_msgs__msg__PNCMap__Sequence *
base_msgs__msg__PNCMap__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__PNCMap__Sequence * array = (base_msgs__msg__PNCMap__Sequence *)allocator.allocate(sizeof(base_msgs__msg__PNCMap__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__msg__PNCMap__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__msg__PNCMap__Sequence__destroy(base_msgs__msg__PNCMap__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__msg__PNCMap__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__msg__PNCMap__Sequence__are_equal(const base_msgs__msg__PNCMap__Sequence * lhs, const base_msgs__msg__PNCMap__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__msg__PNCMap__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__msg__PNCMap__Sequence__copy(
  const base_msgs__msg__PNCMap__Sequence * input,
  base_msgs__msg__PNCMap__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__msg__PNCMap);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__msg__PNCMap * data =
      (base_msgs__msg__PNCMap *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__msg__PNCMap__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__msg__PNCMap__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__msg__PNCMap__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
