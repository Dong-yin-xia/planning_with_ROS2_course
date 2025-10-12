// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/PNCMap.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PNC_MAP__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__PNC_MAP__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'left_boundary'
// Member 'right_boundary'
// Member 'midline'
#include "visualization_msgs/msg/detail/marker__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__PNCMap __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__PNCMap __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PNCMap_
{
  using Type = PNCMap_<ContainerAllocator>;

  explicit PNCMap_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    left_boundary(_init),
    right_boundary(_init),
    midline(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->road_length = 0.0;
      this->road_half_width = 0.0;
    }
  }

  explicit PNCMap_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    left_boundary(_alloc, _init),
    right_boundary(_alloc, _init),
    midline(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->road_length = 0.0;
      this->road_half_width = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _road_length_type =
    double;
  _road_length_type road_length;
  using _road_half_width_type =
    double;
  _road_half_width_type road_half_width;
  using _left_boundary_type =
    visualization_msgs::msg::Marker_<ContainerAllocator>;
  _left_boundary_type left_boundary;
  using _right_boundary_type =
    visualization_msgs::msg::Marker_<ContainerAllocator>;
  _right_boundary_type right_boundary;
  using _midline_type =
    visualization_msgs::msg::Marker_<ContainerAllocator>;
  _midline_type midline;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__road_length(
    const double & _arg)
  {
    this->road_length = _arg;
    return *this;
  }
  Type & set__road_half_width(
    const double & _arg)
  {
    this->road_half_width = _arg;
    return *this;
  }
  Type & set__left_boundary(
    const visualization_msgs::msg::Marker_<ContainerAllocator> & _arg)
  {
    this->left_boundary = _arg;
    return *this;
  }
  Type & set__right_boundary(
    const visualization_msgs::msg::Marker_<ContainerAllocator> & _arg)
  {
    this->right_boundary = _arg;
    return *this;
  }
  Type & set__midline(
    const visualization_msgs::msg::Marker_<ContainerAllocator> & _arg)
  {
    this->midline = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::PNCMap_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::PNCMap_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::PNCMap_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::PNCMap_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::PNCMap_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::PNCMap_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::PNCMap_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::PNCMap_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::PNCMap_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::PNCMap_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__PNCMap
    std::shared_ptr<base_msgs::msg::PNCMap_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__PNCMap
    std::shared_ptr<base_msgs::msg::PNCMap_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PNCMap_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->road_length != other.road_length) {
      return false;
    }
    if (this->road_half_width != other.road_half_width) {
      return false;
    }
    if (this->left_boundary != other.left_boundary) {
      return false;
    }
    if (this->right_boundary != other.right_boundary) {
      return false;
    }
    if (this->midline != other.midline) {
      return false;
    }
    return true;
  }
  bool operator!=(const PNCMap_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PNCMap_

// alias to use template instance with default allocator
using PNCMap =
  base_msgs::msg::PNCMap_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__PNC_MAP__STRUCT_HPP_
