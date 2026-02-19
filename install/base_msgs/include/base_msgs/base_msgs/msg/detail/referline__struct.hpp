// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/Referline.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__REFERLINE__STRUCT_HPP_

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
// Member 'refer_line'
#include "base_msgs/msg/detail/referline_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__Referline __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__Referline __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Referline_
{
  using Type = Referline_<ContainerAllocator>;

  explicit Referline_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit Referline_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _refer_line_type =
    std::vector<base_msgs::msg::ReferlinePoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<base_msgs::msg::ReferlinePoint_<ContainerAllocator>>>;
  _refer_line_type refer_line;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__refer_line(
    const std::vector<base_msgs::msg::ReferlinePoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<base_msgs::msg::ReferlinePoint_<ContainerAllocator>>> & _arg)
  {
    this->refer_line = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::Referline_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::Referline_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::Referline_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::Referline_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::Referline_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::Referline_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::Referline_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::Referline_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::Referline_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::Referline_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__Referline
    std::shared_ptr<base_msgs::msg::Referline_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__Referline
    std::shared_ptr<base_msgs::msg::Referline_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Referline_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->refer_line != other.refer_line) {
      return false;
    }
    return true;
  }
  bool operator!=(const Referline_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Referline_

// alias to use template instance with default allocator
using Referline =
  base_msgs::msg::Referline_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE__STRUCT_HPP_
