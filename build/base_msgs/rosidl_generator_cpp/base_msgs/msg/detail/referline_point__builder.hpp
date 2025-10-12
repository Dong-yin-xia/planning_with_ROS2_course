// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/ReferlinePoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/referline_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_ReferlinePoint_rdkappa
{
public:
  explicit Init_ReferlinePoint_rdkappa(::base_msgs::msg::ReferlinePoint & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::ReferlinePoint rdkappa(::base_msgs::msg::ReferlinePoint::_rdkappa_type arg)
  {
    msg_.rdkappa = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::ReferlinePoint msg_;
};

class Init_ReferlinePoint_rkappa
{
public:
  explicit Init_ReferlinePoint_rkappa(::base_msgs::msg::ReferlinePoint & msg)
  : msg_(msg)
  {}
  Init_ReferlinePoint_rdkappa rkappa(::base_msgs::msg::ReferlinePoint::_rkappa_type arg)
  {
    msg_.rkappa = std::move(arg);
    return Init_ReferlinePoint_rdkappa(msg_);
  }

private:
  ::base_msgs::msg::ReferlinePoint msg_;
};

class Init_ReferlinePoint_rtheta
{
public:
  explicit Init_ReferlinePoint_rtheta(::base_msgs::msg::ReferlinePoint & msg)
  : msg_(msg)
  {}
  Init_ReferlinePoint_rkappa rtheta(::base_msgs::msg::ReferlinePoint::_rtheta_type arg)
  {
    msg_.rtheta = std::move(arg);
    return Init_ReferlinePoint_rkappa(msg_);
  }

private:
  ::base_msgs::msg::ReferlinePoint msg_;
};

class Init_ReferlinePoint_rs
{
public:
  explicit Init_ReferlinePoint_rs(::base_msgs::msg::ReferlinePoint & msg)
  : msg_(msg)
  {}
  Init_ReferlinePoint_rtheta rs(::base_msgs::msg::ReferlinePoint::_rs_type arg)
  {
    msg_.rs = std::move(arg);
    return Init_ReferlinePoint_rtheta(msg_);
  }

private:
  ::base_msgs::msg::ReferlinePoint msg_;
};

class Init_ReferlinePoint_pose
{
public:
  Init_ReferlinePoint_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReferlinePoint_rs pose(::base_msgs::msg::ReferlinePoint::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_ReferlinePoint_rs(msg_);
  }

private:
  ::base_msgs::msg::ReferlinePoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::ReferlinePoint>()
{
  return base_msgs::msg::builder::Init_ReferlinePoint_pose();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__REFERLINE_POINT__BUILDER_HPP_
