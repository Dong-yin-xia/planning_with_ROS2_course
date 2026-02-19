// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/LocalSpeedsPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__LocalSpeedsPoint __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__LocalSpeedsPoint __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalSpeedsPoint_
{
  using Type = LocalSpeedsPoint_<ContainerAllocator>;

  explicit LocalSpeedsPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t = 0.0;
      this->s_2path = 0.0;
      this->ds_dt_2path = 0.0;
      this->dds_dt_2path = 0.0;
      this->speed = 0.0;
      this->acceleration = 0.0;
      this->dacceleration = 0.0;
    }
  }

  explicit LocalSpeedsPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->t = 0.0;
      this->s_2path = 0.0;
      this->ds_dt_2path = 0.0;
      this->dds_dt_2path = 0.0;
      this->speed = 0.0;
      this->acceleration = 0.0;
      this->dacceleration = 0.0;
    }
  }

  // field types and members
  using _t_type =
    double;
  _t_type t;
  using _s_2path_type =
    double;
  _s_2path_type s_2path;
  using _ds_dt_2path_type =
    double;
  _ds_dt_2path_type ds_dt_2path;
  using _dds_dt_2path_type =
    double;
  _dds_dt_2path_type dds_dt_2path;
  using _speed_type =
    double;
  _speed_type speed;
  using _acceleration_type =
    double;
  _acceleration_type acceleration;
  using _dacceleration_type =
    double;
  _dacceleration_type dacceleration;

  // setters for named parameter idiom
  Type & set__t(
    const double & _arg)
  {
    this->t = _arg;
    return *this;
  }
  Type & set__s_2path(
    const double & _arg)
  {
    this->s_2path = _arg;
    return *this;
  }
  Type & set__ds_dt_2path(
    const double & _arg)
  {
    this->ds_dt_2path = _arg;
    return *this;
  }
  Type & set__dds_dt_2path(
    const double & _arg)
  {
    this->dds_dt_2path = _arg;
    return *this;
  }
  Type & set__speed(
    const double & _arg)
  {
    this->speed = _arg;
    return *this;
  }
  Type & set__acceleration(
    const double & _arg)
  {
    this->acceleration = _arg;
    return *this;
  }
  Type & set__dacceleration(
    const double & _arg)
  {
    this->dacceleration = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__LocalSpeedsPoint
    std::shared_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__LocalSpeedsPoint
    std::shared_ptr<base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalSpeedsPoint_ & other) const
  {
    if (this->t != other.t) {
      return false;
    }
    if (this->s_2path != other.s_2path) {
      return false;
    }
    if (this->ds_dt_2path != other.ds_dt_2path) {
      return false;
    }
    if (this->dds_dt_2path != other.dds_dt_2path) {
      return false;
    }
    if (this->speed != other.speed) {
      return false;
    }
    if (this->acceleration != other.acceleration) {
      return false;
    }
    if (this->dacceleration != other.dacceleration) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalSpeedsPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalSpeedsPoint_

// alias to use template instance with default allocator
using LocalSpeedsPoint =
  base_msgs::msg::LocalSpeedsPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS_POINT__STRUCT_HPP_
