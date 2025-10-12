// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base_msgs:srv/GlobalPathService.idl
// generated code does not contain a copyright notice
#include "base_msgs/srv/detail/global_path_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `pnc_map`
#include "base_msgs/msg/detail/pnc_map__functions.h"

bool
base_msgs__srv__GlobalPathService_Request__init(base_msgs__srv__GlobalPathService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // global_planner_type
  // pnc_map
  if (!base_msgs__msg__PNCMap__init(&msg->pnc_map)) {
    base_msgs__srv__GlobalPathService_Request__fini(msg);
    return false;
  }
  return true;
}

void
base_msgs__srv__GlobalPathService_Request__fini(base_msgs__srv__GlobalPathService_Request * msg)
{
  if (!msg) {
    return;
  }
  // global_planner_type
  // pnc_map
  base_msgs__msg__PNCMap__fini(&msg->pnc_map);
}

bool
base_msgs__srv__GlobalPathService_Request__are_equal(const base_msgs__srv__GlobalPathService_Request * lhs, const base_msgs__srv__GlobalPathService_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // global_planner_type
  if (lhs->global_planner_type != rhs->global_planner_type) {
    return false;
  }
  // pnc_map
  if (!base_msgs__msg__PNCMap__are_equal(
      &(lhs->pnc_map), &(rhs->pnc_map)))
  {
    return false;
  }
  return true;
}

bool
base_msgs__srv__GlobalPathService_Request__copy(
  const base_msgs__srv__GlobalPathService_Request * input,
  base_msgs__srv__GlobalPathService_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // global_planner_type
  output->global_planner_type = input->global_planner_type;
  // pnc_map
  if (!base_msgs__msg__PNCMap__copy(
      &(input->pnc_map), &(output->pnc_map)))
  {
    return false;
  }
  return true;
}

base_msgs__srv__GlobalPathService_Request *
base_msgs__srv__GlobalPathService_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__srv__GlobalPathService_Request * msg = (base_msgs__srv__GlobalPathService_Request *)allocator.allocate(sizeof(base_msgs__srv__GlobalPathService_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__srv__GlobalPathService_Request));
  bool success = base_msgs__srv__GlobalPathService_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__srv__GlobalPathService_Request__destroy(base_msgs__srv__GlobalPathService_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__srv__GlobalPathService_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__srv__GlobalPathService_Request__Sequence__init(base_msgs__srv__GlobalPathService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__srv__GlobalPathService_Request * data = NULL;

  if (size) {
    data = (base_msgs__srv__GlobalPathService_Request *)allocator.zero_allocate(size, sizeof(base_msgs__srv__GlobalPathService_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__srv__GlobalPathService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__srv__GlobalPathService_Request__fini(&data[i - 1]);
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
base_msgs__srv__GlobalPathService_Request__Sequence__fini(base_msgs__srv__GlobalPathService_Request__Sequence * array)
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
      base_msgs__srv__GlobalPathService_Request__fini(&array->data[i]);
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

base_msgs__srv__GlobalPathService_Request__Sequence *
base_msgs__srv__GlobalPathService_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__srv__GlobalPathService_Request__Sequence * array = (base_msgs__srv__GlobalPathService_Request__Sequence *)allocator.allocate(sizeof(base_msgs__srv__GlobalPathService_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__srv__GlobalPathService_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__srv__GlobalPathService_Request__Sequence__destroy(base_msgs__srv__GlobalPathService_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__srv__GlobalPathService_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__srv__GlobalPathService_Request__Sequence__are_equal(const base_msgs__srv__GlobalPathService_Request__Sequence * lhs, const base_msgs__srv__GlobalPathService_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__srv__GlobalPathService_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__srv__GlobalPathService_Request__Sequence__copy(
  const base_msgs__srv__GlobalPathService_Request__Sequence * input,
  base_msgs__srv__GlobalPathService_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__srv__GlobalPathService_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__srv__GlobalPathService_Request * data =
      (base_msgs__srv__GlobalPathService_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__srv__GlobalPathService_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__srv__GlobalPathService_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__srv__GlobalPathService_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `global_path`
#include "nav_msgs/msg/detail/path__functions.h"

bool
base_msgs__srv__GlobalPathService_Response__init(base_msgs__srv__GlobalPathService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // global_path
  if (!nav_msgs__msg__Path__init(&msg->global_path)) {
    base_msgs__srv__GlobalPathService_Response__fini(msg);
    return false;
  }
  return true;
}

void
base_msgs__srv__GlobalPathService_Response__fini(base_msgs__srv__GlobalPathService_Response * msg)
{
  if (!msg) {
    return;
  }
  // global_path
  nav_msgs__msg__Path__fini(&msg->global_path);
}

bool
base_msgs__srv__GlobalPathService_Response__are_equal(const base_msgs__srv__GlobalPathService_Response * lhs, const base_msgs__srv__GlobalPathService_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // global_path
  if (!nav_msgs__msg__Path__are_equal(
      &(lhs->global_path), &(rhs->global_path)))
  {
    return false;
  }
  return true;
}

bool
base_msgs__srv__GlobalPathService_Response__copy(
  const base_msgs__srv__GlobalPathService_Response * input,
  base_msgs__srv__GlobalPathService_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // global_path
  if (!nav_msgs__msg__Path__copy(
      &(input->global_path), &(output->global_path)))
  {
    return false;
  }
  return true;
}

base_msgs__srv__GlobalPathService_Response *
base_msgs__srv__GlobalPathService_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__srv__GlobalPathService_Response * msg = (base_msgs__srv__GlobalPathService_Response *)allocator.allocate(sizeof(base_msgs__srv__GlobalPathService_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__srv__GlobalPathService_Response));
  bool success = base_msgs__srv__GlobalPathService_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__srv__GlobalPathService_Response__destroy(base_msgs__srv__GlobalPathService_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__srv__GlobalPathService_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__srv__GlobalPathService_Response__Sequence__init(base_msgs__srv__GlobalPathService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__srv__GlobalPathService_Response * data = NULL;

  if (size) {
    data = (base_msgs__srv__GlobalPathService_Response *)allocator.zero_allocate(size, sizeof(base_msgs__srv__GlobalPathService_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__srv__GlobalPathService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__srv__GlobalPathService_Response__fini(&data[i - 1]);
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
base_msgs__srv__GlobalPathService_Response__Sequence__fini(base_msgs__srv__GlobalPathService_Response__Sequence * array)
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
      base_msgs__srv__GlobalPathService_Response__fini(&array->data[i]);
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

base_msgs__srv__GlobalPathService_Response__Sequence *
base_msgs__srv__GlobalPathService_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__srv__GlobalPathService_Response__Sequence * array = (base_msgs__srv__GlobalPathService_Response__Sequence *)allocator.allocate(sizeof(base_msgs__srv__GlobalPathService_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__srv__GlobalPathService_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__srv__GlobalPathService_Response__Sequence__destroy(base_msgs__srv__GlobalPathService_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__srv__GlobalPathService_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__srv__GlobalPathService_Response__Sequence__are_equal(const base_msgs__srv__GlobalPathService_Response__Sequence * lhs, const base_msgs__srv__GlobalPathService_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__srv__GlobalPathService_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__srv__GlobalPathService_Response__Sequence__copy(
  const base_msgs__srv__GlobalPathService_Response__Sequence * input,
  base_msgs__srv__GlobalPathService_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__srv__GlobalPathService_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__srv__GlobalPathService_Response * data =
      (base_msgs__srv__GlobalPathService_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__srv__GlobalPathService_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__srv__GlobalPathService_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__srv__GlobalPathService_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
