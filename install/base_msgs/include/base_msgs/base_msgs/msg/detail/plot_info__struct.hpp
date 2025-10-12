// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/PlotInfo.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PLOT_INFO__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__PLOT_INFO__STRUCT_HPP_

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
// Member 'trajectory_info'
#include "base_msgs/msg/detail/local_trajectory__struct.hpp"
// Member 'obs_info'
#include "base_msgs/msg/detail/obs_info__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__PlotInfo __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__PlotInfo __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlotInfo_
{
  using Type = PlotInfo_<ContainerAllocator>;

  explicit PlotInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    trajectory_info(_init)
  {
    (void)_init;
  }

  explicit PlotInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    trajectory_info(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _trajectory_info_type =
    base_msgs::msg::LocalTrajectory_<ContainerAllocator>;
  _trajectory_info_type trajectory_info;
  using _obs_info_type =
    std::vector<base_msgs::msg::ObsInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<base_msgs::msg::ObsInfo_<ContainerAllocator>>>;
  _obs_info_type obs_info;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__trajectory_info(
    const base_msgs::msg::LocalTrajectory_<ContainerAllocator> & _arg)
  {
    this->trajectory_info = _arg;
    return *this;
  }
  Type & set__obs_info(
    const std::vector<base_msgs::msg::ObsInfo_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<base_msgs::msg::ObsInfo_<ContainerAllocator>>> & _arg)
  {
    this->obs_info = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::PlotInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::PlotInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::PlotInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::PlotInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__PlotInfo
    std::shared_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__PlotInfo
    std::shared_ptr<base_msgs::msg::PlotInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlotInfo_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->trajectory_info != other.trajectory_info) {
      return false;
    }
    if (this->obs_info != other.obs_info) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlotInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlotInfo_

// alias to use template instance with default allocator
using PlotInfo =
  base_msgs::msg::PlotInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__PLOT_INFO__STRUCT_HPP_
