// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/LocalSpeeds.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/local_speeds__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalSpeeds_local_speeds
{
public:
  explicit Init_LocalSpeeds_local_speeds(::base_msgs::msg::LocalSpeeds & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::LocalSpeeds local_speeds(::base_msgs::msg::LocalSpeeds::_local_speeds_type arg)
  {
    msg_.local_speeds = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeeds msg_;
};

class Init_LocalSpeeds_header
{
public:
  Init_LocalSpeeds_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalSpeeds_local_speeds header(::base_msgs::msg::LocalSpeeds::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LocalSpeeds_local_speeds(msg_);
  }

private:
  ::base_msgs::msg::LocalSpeeds msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::LocalSpeeds>()
{
  return base_msgs::msg::builder::Init_LocalSpeeds_header();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_SPEEDS__BUILDER_HPP_
