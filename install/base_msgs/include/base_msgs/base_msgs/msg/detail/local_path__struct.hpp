// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH__STRUCT_HPP_

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
// Member 'local_path'
#include "base_msgs/msg/detail/local_path_point__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__LocalPath __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__LocalPath __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalPath_
{
  using Type = LocalPath_<ContainerAllocator>;

  explicit LocalPath_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit LocalPath_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _local_path_type =
    std::vector<base_msgs::msg::LocalPathPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<base_msgs::msg::LocalPathPoint_<ContainerAllocator>>>;
  _local_path_type local_path;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__local_path(
    const std::vector<base_msgs::msg::LocalPathPoint_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<base_msgs::msg::LocalPathPoint_<ContainerAllocator>>> & _arg)
  {
    this->local_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::LocalPath_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::LocalPath_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::LocalPath_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::LocalPath_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalPath_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalPath_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalPath_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalPath_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::LocalPath_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::LocalPath_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__LocalPath
    std::shared_ptr<base_msgs::msg::LocalPath_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__LocalPath
    std::shared_ptr<base_msgs::msg::LocalPath_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalPath_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->local_path != other.local_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const LocalPath_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalPath_

// alias to use template instance with default allocator
using LocalPath =
  base_msgs::msg::LocalPath_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH__STRUCT_HPP_
