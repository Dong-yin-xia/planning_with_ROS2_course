// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/ReferlinePoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_stamped__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__ReferlinePoint __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__ReferlinePoint __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ReferlinePoint_
{
  using Type = ReferlinePoint_<ContainerAllocator>;

  explicit ReferlinePoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rs = 0.0;
      this->rtheta = 0.0;
      this->rkappa = 0.0;
      this->rdkappa = 0.0;
    }
  }

  explicit ReferlinePoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->rs = 0.0;
      this->rtheta = 0.0;
      this->rkappa = 0.0;
      this->rdkappa = 0.0;
    }
  }

  // field types and members
  using _pose_type =
    geometry_msgs::msg::PoseStamped_<ContainerAllocator>;
  _pose_type pose;
  using _rs_type =
    double;
  _rs_type rs;
  using _rtheta_type =
    double;
  _rtheta_type rtheta;
  using _rkappa_type =
    double;
  _rkappa_type rkappa;
  using _rdkappa_type =
    double;
  _rdkappa_type rdkappa;

  // setters for named parameter idiom
  Type & set__pose(
    const geometry_msgs::msg::PoseStamped_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__rs(
    const double & _arg)
  {
    this->rs = _arg;
    return *this;
  }
  Type & set__rtheta(
    const double & _arg)
  {
    this->rtheta = _arg;
    return *this;
  }
  Type & set__rkappa(
    const double & _arg)
  {
    this->rkappa = _arg;
    return *this;
  }
  Type & set__rdkappa(
    const double & _arg)
  {
    this->rdkappa = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::ReferlinePoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::ReferlinePoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::ReferlinePoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::ReferlinePoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__ReferlinePoint
    std::shared_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__ReferlinePoint
    std::shared_ptr<base_msgs::msg::ReferlinePoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ReferlinePoint_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->rs != other.rs) {
      return false;
    }
    if (this->rtheta != other.rtheta) {
      return false;
    }
    if (this->rkappa != other.rkappa) {
      return false;
    }
    if (this->rdkappa != other.rdkappa) {
      return false;
    }
    return true;
  }
  bool operator!=(const ReferlinePoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ReferlinePoint_

// alias to use template instance with default allocator
using ReferlinePoint =
  base_msgs::msg::ReferlinePoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__STRUCT_HPP_
