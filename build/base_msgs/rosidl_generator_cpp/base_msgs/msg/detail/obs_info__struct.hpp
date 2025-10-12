// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/ObsInfo.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__OBS_INFO__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__OBS_INFO__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__base_msgs__msg__ObsInfo __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__ObsInfo __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ObsInfo_
{
  using Type = ObsInfo_<ContainerAllocator>;

  explicit ObsInfo_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obs_length = 0.0;
      this->obs_width = 0.0;
      this->l = 0.0;
      this->s = 0.0;
      this->s_2path = 0.0;
      this->ds_dt_2path = 0.0;
      this->t_in = 0.0;
      this->t_out = 0.0;
    }
  }

  explicit ObsInfo_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->obs_length = 0.0;
      this->obs_width = 0.0;
      this->l = 0.0;
      this->s = 0.0;
      this->s_2path = 0.0;
      this->ds_dt_2path = 0.0;
      this->t_in = 0.0;
      this->t_out = 0.0;
    }
  }

  // field types and members
  using _obs_length_type =
    double;
  _obs_length_type obs_length;
  using _obs_width_type =
    double;
  _obs_width_type obs_width;
  using _l_type =
    double;
  _l_type l;
  using _s_type =
    double;
  _s_type s;
  using _s_2path_type =
    double;
  _s_2path_type s_2path;
  using _ds_dt_2path_type =
    double;
  _ds_dt_2path_type ds_dt_2path;
  using _t_in_type =
    double;
  _t_in_type t_in;
  using _t_out_type =
    double;
  _t_out_type t_out;

  // setters for named parameter idiom
  Type & set__obs_length(
    const double & _arg)
  {
    this->obs_length = _arg;
    return *this;
  }
  Type & set__obs_width(
    const double & _arg)
  {
    this->obs_width = _arg;
    return *this;
  }
  Type & set__l(
    const double & _arg)
  {
    this->l = _arg;
    return *this;
  }
  Type & set__s(
    const double & _arg)
  {
    this->s = _arg;
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
  Type & set__t_in(
    const double & _arg)
  {
    this->t_in = _arg;
    return *this;
  }
  Type & set__t_out(
    const double & _arg)
  {
    this->t_out = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::msg::ObsInfo_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::ObsInfo_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::ObsInfo_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::ObsInfo_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__ObsInfo
    std::shared_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__ObsInfo
    std::shared_ptr<base_msgs::msg::ObsInfo_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ObsInfo_ & other) const
  {
    if (this->obs_length != other.obs_length) {
      return false;
    }
    if (this->obs_width != other.obs_width) {
      return false;
    }
    if (this->l != other.l) {
      return false;
    }
    if (this->s != other.s) {
      return false;
    }
    if (this->s_2path != other.s_2path) {
      return false;
    }
    if (this->ds_dt_2path != other.ds_dt_2path) {
      return false;
    }
    if (this->t_in != other.t_in) {
      return false;
    }
    if (this->t_out != other.t_out) {
      return false;
    }
    return true;
  }
  bool operator!=(const ObsInfo_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ObsInfo_

// alias to use template instance with default allocator
using ObsInfo =
  base_msgs::msg::ObsInfo_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__OBS_INFO__STRUCT_HPP_
