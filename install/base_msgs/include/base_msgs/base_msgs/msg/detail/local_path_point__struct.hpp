// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:msg/LocalPathPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__STRUCT_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__STRUCT_HPP_

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
# define DEPRECATED__base_msgs__msg__LocalPathPoint __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__msg__LocalPathPoint __declspec(deprecated)
#endif

namespace base_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct LocalPathPoint_
{
  using Type = LocalPathPoint_<ContainerAllocator>;

  explicit LocalPathPoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->s = 0.0;
      this->l = 0.0;
      this->ds_dt = 0.0;
      this->dl_ds = 0.0;
      this->dl_dt = 0.0;
      this->dds_dt = 0.0;
      this->ddl_ds = 0.0;
      this->ddl_dt = 0.0;
      this->theta = 0.0;
      this->kappa = 0.0;
      this->dkappa = 0.0;
      this->rs = 0.0;
      this->rtheta = 0.0;
      this->rkappa = 0.0;
      this->rdkappa = 0.0;
    }
  }

  explicit LocalPathPoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->s = 0.0;
      this->l = 0.0;
      this->ds_dt = 0.0;
      this->dl_ds = 0.0;
      this->dl_dt = 0.0;
      this->dds_dt = 0.0;
      this->ddl_ds = 0.0;
      this->ddl_dt = 0.0;
      this->theta = 0.0;
      this->kappa = 0.0;
      this->dkappa = 0.0;
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
  using _s_type =
    double;
  _s_type s;
  using _l_type =
    double;
  _l_type l;
  using _ds_dt_type =
    double;
  _ds_dt_type ds_dt;
  using _dl_ds_type =
    double;
  _dl_ds_type dl_ds;
  using _dl_dt_type =
    double;
  _dl_dt_type dl_dt;
  using _dds_dt_type =
    double;
  _dds_dt_type dds_dt;
  using _ddl_ds_type =
    double;
  _ddl_ds_type ddl_ds;
  using _ddl_dt_type =
    double;
  _ddl_dt_type ddl_dt;
  using _theta_type =
    double;
  _theta_type theta;
  using _kappa_type =
    double;
  _kappa_type kappa;
  using _dkappa_type =
    double;
  _dkappa_type dkappa;
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
  Type & set__s(
    const double & _arg)
  {
    this->s = _arg;
    return *this;
  }
  Type & set__l(
    const double & _arg)
  {
    this->l = _arg;
    return *this;
  }
  Type & set__ds_dt(
    const double & _arg)
  {
    this->ds_dt = _arg;
    return *this;
  }
  Type & set__dl_ds(
    const double & _arg)
  {
    this->dl_ds = _arg;
    return *this;
  }
  Type & set__dl_dt(
    const double & _arg)
  {
    this->dl_dt = _arg;
    return *this;
  }
  Type & set__dds_dt(
    const double & _arg)
  {
    this->dds_dt = _arg;
    return *this;
  }
  Type & set__ddl_ds(
    const double & _arg)
  {
    this->ddl_ds = _arg;
    return *this;
  }
  Type & set__ddl_dt(
    const double & _arg)
  {
    this->ddl_dt = _arg;
    return *this;
  }
  Type & set__theta(
    const double & _arg)
  {
    this->theta = _arg;
    return *this;
  }
  Type & set__kappa(
    const double & _arg)
  {
    this->kappa = _arg;
    return *this;
  }
  Type & set__dkappa(
    const double & _arg)
  {
    this->dkappa = _arg;
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
    base_msgs::msg::LocalPathPoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::msg::LocalPathPoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalPathPoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::msg::LocalPathPoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__msg__LocalPathPoint
    std::shared_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__msg__LocalPathPoint
    std::shared_ptr<base_msgs::msg::LocalPathPoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LocalPathPoint_ & other) const
  {
    if (this->pose != other.pose) {
      return false;
    }
    if (this->s != other.s) {
      return false;
    }
    if (this->l != other.l) {
      return false;
    }
    if (this->ds_dt != other.ds_dt) {
      return false;
    }
    if (this->dl_ds != other.dl_ds) {
      return false;
    }
    if (this->dl_dt != other.dl_dt) {
      return false;
    }
    if (this->dds_dt != other.dds_dt) {
      return false;
    }
    if (this->ddl_ds != other.ddl_ds) {
      return false;
    }
    if (this->ddl_dt != other.ddl_dt) {
      return false;
    }
    if (this->theta != other.theta) {
      return false;
    }
    if (this->kappa != other.kappa) {
      return false;
    }
    if (this->dkappa != other.dkappa) {
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
  bool operator!=(const LocalPathPoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LocalPathPoint_

// alias to use template instance with default allocator
using LocalPathPoint =
  base_msgs::msg::LocalPathPoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH_POINT__STRUCT_HPP_
