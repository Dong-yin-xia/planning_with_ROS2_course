// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:srv/PNCMapService.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__BUILDER_HPP_
#define BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/srv/detail/pnc_map_service__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace srv
{

namespace builder
{

class Init_PNCMapService_Request_map_type
{
public:
  Init_PNCMapService_Request_map_type()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::base_msgs::srv::PNCMapService_Request map_type(::base_msgs::srv::PNCMapService_Request::_map_type_type arg)
  {
    msg_.map_type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::srv::PNCMapService_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::srv::PNCMapService_Request>()
{
  return base_msgs::srv::builder::Init_PNCMapService_Request_map_type();
}

}  // namespace base_msgs


namespace base_msgs
{

namespace srv
{

namespace builder
{

class Init_PNCMapService_Response_pnc_map
{
public:
  Init_PNCMapService_Response_pnc_map()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::base_msgs::srv::PNCMapService_Response pnc_map(::base_msgs::srv::PNCMapService_Response::_pnc_map_type arg)
  {
    msg_.pnc_map = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::srv::PNCMapService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::srv::PNCMapService_Response>()
{
  return base_msgs::srv::builder::Init_PNCMapService_Response_pnc_map();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__SRV__DETAIL__PNC_MAP_SERVICE__BUILDER_HPP_
