// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from base_msgs:msg/LocalPathPoint.idl
// generated code does not contain a copyright notice
#include "base_msgs/msg/detail/local_path_point__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "geometry_msgs/msg/detail/pose_stamped__functions.h"

bool
base_msgs__msg__LocalPathPoint__init(base_msgs__msg__LocalPathPoint * msg)
{
  if (!msg) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__init(&msg->pose)) {
    base_msgs__msg__LocalPathPoint__fini(msg);
    return false;
  }
  // s
  // l
  // ds_dt
  // dl_ds
  // dl_dt
  // dds_dt
  // ddl_ds
  // ddl_dt
  // theta
  // kappa
  // dkappa
  // rs
  // rtheta
  // rkappa
  // rdkappa
  return true;
}

void
base_msgs__msg__LocalPathPoint__fini(base_msgs__msg__LocalPathPoint * msg)
{
  if (!msg) {
    return;
  }
  // pose
  geometry_msgs__msg__PoseStamped__fini(&msg->pose);
  // s
  // l
  // ds_dt
  // dl_ds
  // dl_dt
  // dds_dt
  // ddl_ds
  // ddl_dt
  // theta
  // kappa
  // dkappa
  // rs
  // rtheta
  // rkappa
  // rdkappa
}

bool
base_msgs__msg__LocalPathPoint__are_equal(const base_msgs__msg__LocalPathPoint * lhs, const base_msgs__msg__LocalPathPoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // s
  if (lhs->s != rhs->s) {
    return false;
  }
  // l
  if (lhs->l != rhs->l) {
    return false;
  }
  // ds_dt
  if (lhs->ds_dt != rhs->ds_dt) {
    return false;
  }
  // dl_ds
  if (lhs->dl_ds != rhs->dl_ds) {
    return false;
  }
  // dl_dt
  if (lhs->dl_dt != rhs->dl_dt) {
    return false;
  }
  // dds_dt
  if (lhs->dds_dt != rhs->dds_dt) {
    return false;
  }
  // ddl_ds
  if (lhs->ddl_ds != rhs->ddl_ds) {
    return false;
  }
  // ddl_dt
  if (lhs->ddl_dt != rhs->ddl_dt) {
    return false;
  }
  // theta
  if (lhs->theta != rhs->theta) {
    return false;
  }
  // kappa
  if (lhs->kappa != rhs->kappa) {
    return false;
  }
  // dkappa
  if (lhs->dkappa != rhs->dkappa) {
    return false;
  }
  // rs
  if (lhs->rs != rhs->rs) {
    return false;
  }
  // rtheta
  if (lhs->rtheta != rhs->rtheta) {
    return false;
  }
  // rkappa
  if (lhs->rkappa != rhs->rkappa) {
    return false;
  }
  // rdkappa
  if (lhs->rdkappa != rhs->rdkappa) {
    return false;
  }
  return true;
}

bool
base_msgs__msg__LocalPathPoint__copy(
  const base_msgs__msg__LocalPathPoint * input,
  base_msgs__msg__LocalPathPoint * output)
{
  if (!input || !output) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__PoseStamped__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // s
  output->s = input->s;
  // l
  output->l = input->l;
  // ds_dt
  output->ds_dt = input->ds_dt;
  // dl_ds
  output->dl_ds = input->dl_ds;
  // dl_dt
  output->dl_dt = input->dl_dt;
  // dds_dt
  output->dds_dt = input->dds_dt;
  // ddl_ds
  output->ddl_ds = input->ddl_ds;
  // ddl_dt
  output->ddl_dt = input->ddl_dt;
  // theta
  output->theta = input->theta;
  // kappa
  output->kappa = input->kappa;
  // dkappa
  output->dkappa = input->dkappa;
  // rs
  output->rs = input->rs;
  // rtheta
  output->rtheta = input->rtheta;
  // rkappa
  output->rkappa = input->rkappa;
  // rdkappa
  output->rdkappa = input->rdkappa;
  return true;
}

base_msgs__msg__LocalPathPoint *
base_msgs__msg__LocalPathPoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalPathPoint * msg = (base_msgs__msg__LocalPathPoint *)allocator.allocate(sizeof(base_msgs__msg__LocalPathPoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(base_msgs__msg__LocalPathPoint));
  bool success = base_msgs__msg__LocalPathPoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
base_msgs__msg__LocalPathPoint__destroy(base_msgs__msg__LocalPathPoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    base_msgs__msg__LocalPathPoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
base_msgs__msg__LocalPathPoint__Sequence__init(base_msgs__msg__LocalPathPoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalPathPoint * data = NULL;

  if (size) {
    data = (base_msgs__msg__LocalPathPoint *)allocator.zero_allocate(size, sizeof(base_msgs__msg__LocalPathPoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = base_msgs__msg__LocalPathPoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        base_msgs__msg__LocalPathPoint__fini(&data[i - 1]);
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
base_msgs__msg__LocalPathPoint__Sequence__fini(base_msgs__msg__LocalPathPoint__Sequence * array)
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
      base_msgs__msg__LocalPathPoint__fini(&array->data[i]);
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

base_msgs__msg__LocalPathPoint__Sequence *
base_msgs__msg__LocalPathPoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  base_msgs__msg__LocalPathPoint__Sequence * array = (base_msgs__msg__LocalPathPoint__Sequence *)allocator.allocate(sizeof(base_msgs__msg__LocalPathPoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = base_msgs__msg__LocalPathPoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
base_msgs__msg__LocalPathPoint__Sequence__destroy(base_msgs__msg__LocalPathPoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    base_msgs__msg__LocalPathPoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
base_msgs__msg__LocalPathPoint__Sequence__are_equal(const base_msgs__msg__LocalPathPoint__Sequence * lhs, const base_msgs__msg__LocalPathPoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!base_msgs__msg__LocalPathPoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
base_msgs__msg__LocalPathPoint__Sequence__copy(
  const base_msgs__msg__LocalPathPoint__Sequence * input,
  base_msgs__msg__LocalPathPoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(base_msgs__msg__LocalPathPoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    base_msgs__msg__LocalPathPoint * data =
      (base_msgs__msg__LocalPathPoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!base_msgs__msg__LocalPathPoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          base_msgs__msg__LocalPathPoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!base_msgs__msg__LocalPathPoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
