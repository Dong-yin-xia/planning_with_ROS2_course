// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/LocalTrajectoryPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'path_point'
#include "base_msgs/msg/detail/local_path_point__struct.hpp"
// Member 'speed_point'
#include "base_msgs/msg/detail/local_speeds_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__LocalTrajectoryPoint __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__LocalTrajectoryPoint __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalTrajectoryPoint_
{
  using Type = LocalTrajectoryPoint_<ContainerAllocator>;

  explicit LocalTrajectoryPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path_point(_init),
    speed_point(_init)
  {
    (void)_init;
  }

  explicit LocalTrajectoryPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : path_point(_alloc, _init),
    speed_point(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _path_point_type =
    base_msgs::msg::LocalPathPoint_<ContainerAllocator>;
  _path_point_type path_point;
  using _speed_point_type =
    base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator>;
  _speed_point_type speed_point;

  // setters for named parameter idiom
  Type & set__path_point(
    const base_msgs::msg::LocalPathPoint_<ContainerAllocator> & _arg)
  {
    this->path_point = _arg;
    return *this;
  }
  Type & set__speed_point(
    const base_msgs::msg::LocalSpeedsPoint_<ContainerAllocator> & _arg)
  {
    this->speed_point = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__LocalTrajectoryPoint
    std::shared_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__LocalTrajectoryPoint
    std::shared_ptr<base_msgs::msg::LocalTrajectoryPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalTrajectoryPoint_ & other) const
  {
    if (this->path_point != other.path_point) {
      return false;
    }
    if (this->speed_point != other.speed_point) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalTrajectoryPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalTrajectoryPoint_

// alias to use template instance with default allocator
using LocalTrajectoryPoint =
  base_msgs::msg::LocalTrajectoryPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__STRUCT_HPP_
