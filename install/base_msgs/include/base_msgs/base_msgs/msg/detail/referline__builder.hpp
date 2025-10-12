// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/Referline.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__REFERLINE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/referline__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_Referline_refer_line
{
public:
  explicit Init_Referline_refer_line(::base_msgs::msg::Referline & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::Referline refer_line(::base_msgs::msg::Referline::_refer_line_type arg)
  {
    msg_.refer_line = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::Referline msg_;
};

class Init_Referline_header
{
public:
  Init_Referline_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Referline_refer_line header(::base_msgs::msg::Referline::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Referline_refer_line(msg_);
  }

private:
  ::base_msgs::msg::Referline msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::Referline>()
{
  return base_msgs::msg::builder::Init_Referline_header();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE__BUILDER_HPP_
