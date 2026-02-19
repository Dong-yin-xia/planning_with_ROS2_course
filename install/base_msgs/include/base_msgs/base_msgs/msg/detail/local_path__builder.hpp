// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/LocalPath.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_PATH__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/local_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalPath_local_path
{
public:
  explicit Init_LocalPath_local_path(::base_msgs::msg::LocalPath & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::LocalPath local_path(::base_msgs::msg::LocalPath::_local_path_type arg)
  {
    msg_.local_path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::LocalPath msg_;
};

class Init_LocalPath_header
{
public:
  Init_LocalPath_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalPath_local_path header(::base_msgs::msg::LocalPath::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LocalPath_local_path(msg_);
  }

private:
  ::base_msgs::msg::LocalPath msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::LocalPath>()
{
  return base_msgs::msg::builder::Init_LocalPath_header();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_PATH__BUILDER_HPP_
