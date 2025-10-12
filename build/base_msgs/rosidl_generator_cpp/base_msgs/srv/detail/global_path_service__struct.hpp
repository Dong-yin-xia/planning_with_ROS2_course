// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:srv/GlobalPathService.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__STRUCT_HPP_
#define BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pnc_map'
#include "base_msgs/msg/detail/pnc_map__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__srv__GlobalPathService_Request __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__srv__GlobalPathService_Request __declspec(deprecated)
#endif

namespace base_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GlobalPathService_Request_
{
  using Type = GlobalPathService_Request_<ContainerAllocator>;

  explicit GlobalPathService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pnc_map(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->global_planner_type = 0l;
    }
  }

  explicit GlobalPathService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pnc_map(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->global_planner_type = 0l;
    }
  }

  // field types and members
  using _global_planner_type_type =
    int32_t;
  _global_planner_type_type global_planner_type;
  using _pnc_map_type =
    base_msgs::msg::PNCMap_<ContainerAllocator>;
  _pnc_map_type pnc_map;

  // setters for named parameter idiom
  Type & set__global_planner_type(
    const int32_t & _arg)
  {
    this->global_planner_type = _arg;
    return *this;
  }
  Type & set__pnc_map(
    const base_msgs::msg::PNCMap_<ContainerAllocator> & _arg)
  {
    this->pnc_map = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::srv::GlobalPathService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::srv::GlobalPathService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::GlobalPathService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::GlobalPathService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__srv__GlobalPathService_Request
    std::shared_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__srv__GlobalPathService_Request
    std::shared_ptr<base_msgs::srv::GlobalPathService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GlobalPathService_Request_ & other) const
  {
    if (this->global_planner_type != other.global_planner_type) {
      return false;
    }
    if (this->pnc_map != other.pnc_map) {
      return false;
    }
    return true;
  }
  bool operator!=(const GlobalPathService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GlobalPathService_Request_

// alias to use template instance with default allocator
using GlobalPathService_Request =
  base_msgs::srv::GlobalPathService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace base_msgs


// Include directives for member types
// Member 'global_path'
#include "nav_msgs/msg/detail/path__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__srv__GlobalPathService_Response __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__srv__GlobalPathService_Response __declspec(deprecated)
#endif

namespace base_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GlobalPathService_Response_
{
  using Type = GlobalPathService_Response_<ContainerAllocator>;

  explicit GlobalPathService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : global_path(_init)
  {
    (void)_init;
  }

  explicit GlobalPathService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : global_path(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _global_path_type =
    nav_msgs::msg::Path_<ContainerAllocator>;
  _global_path_type global_path;

  // setters for named parameter idiom
  Type & set__global_path(
    const nav_msgs::msg::Path_<ContainerAllocator> & _arg)
  {
    this->global_path = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::srv::GlobalPathService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::srv::GlobalPathService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::GlobalPathService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::GlobalPathService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__srv__GlobalPathService_Response
    std::shared_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__srv__GlobalPathService_Response
    std::shared_ptr<base_msgs::srv::GlobalPathService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GlobalPathService_Response_ & other) const
  {
    if (this->global_path != other.global_path) {
      return false;
    }
    return true;
  }
  bool operator!=(const GlobalPathService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GlobalPathService_Response_

// alias to use template instance with default allocator
using GlobalPathService_Response =
  base_msgs::srv::GlobalPathService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace base_msgs

namespace base_msgs
{

namespace srv
{

struct GlobalPathService
{
  using Request = base_msgs::srv::GlobalPathService_Request;
  using Response = base_msgs::srv::GlobalPathService_Response;
};

}  // namespace srv

}  // namespace base_msgs

#endif  // BASE_MSGS__SRV__DETAIL__GLOBAL_PATH_SERVICE__STRUCT_HPP_
