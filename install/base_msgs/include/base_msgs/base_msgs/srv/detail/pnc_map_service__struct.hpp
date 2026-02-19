// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from base_msgs:srv/PNCMapService.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__STRUCT_HPP_
#define BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__base_msgs__srv__PNCMapService_Request __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__srv__PNCMapService_Request __declspec(deprecated)
#endif

namespace base_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PNCMapService_Request_
{
  using Type = PNCMapService_Request_<ContainerAllocator>;

  explicit PNCMapService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_type = 0l;
    }
  }

  explicit PNCMapService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->map_type = 0l;
    }
  }

  // field types and members
  using _map_type_type =
    int32_t;
  _map_type_type map_type;

  // setters for named parameter idiom
  Type & set__map_type(
    const int32_t & _arg)
  {
    this->map_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::srv::PNCMapService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::srv::PNCMapService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::PNCMapService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::PNCMapService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__srv__PNCMapService_Request
    std::shared_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__srv__PNCMapService_Request
    std::shared_ptr<base_msgs::srv::PNCMapService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PNCMapService_Request_ & other) const
  {
    if (this->map_type != other.map_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const PNCMapService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PNCMapService_Request_

// alias to use template instance with default allocator
using PNCMapService_Request =
  base_msgs::srv::PNCMapService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace base_msgs


// Include directives for member types
// Member 'pnc_map'
#include "base_msgs/msg/detail/pnc_map__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__base_msgs__srv__PNCMapService_Response __attribute__((deprecated))
#else
# define DEPRECATED__base_msgs__srv__PNCMapService_Response __declspec(deprecated)
#endif

namespace base_msgs
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct PNCMapService_Response_
{
  using Type = PNCMapService_Response_<ContainerAllocator>;

  explicit PNCMapService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pnc_map(_init)
  {
    (void)_init;
  }

  explicit PNCMapService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pnc_map(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pnc_map_type =
    base_msgs::msg::PNCMap_<ContainerAllocator>;
  _pnc_map_type pnc_map;

  // setters for named parameter idiom
  Type & set__pnc_map(
    const base_msgs::msg::PNCMap_<ContainerAllocator> & _arg)
  {
    this->pnc_map = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    base_msgs::srv::PNCMapService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const base_msgs::srv::PNCMapService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::PNCMapService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      base_msgs::srv::PNCMapService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__base_msgs__srv__PNCMapService_Response
    std::shared_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__base_msgs__srv__PNCMapService_Response
    std::shared_ptr<base_msgs::srv::PNCMapService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PNCMapService_Response_ & other) const
  {
    if (this->pnc_map != other.pnc_map) {
      return false;
    }
    return true;
  }
  bool operator!=(const PNCMapService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PNCMapService_Response_

// alias to use template instance with default allocator
using PNCMapService_Response =
  base_msgs::srv::PNCMapService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace base_msgs

namespace base_msgs
{

namespace srv
{

struct PNCMapService
{
  using Request = base_msgs::srv::PNCMapService_Request;
  using Response = base_msgs::srv::PNCMapService_Response;
};

}  // namespace srv

}  // namespace base_msgs

#endif  // BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__STRUCT_HPP_
