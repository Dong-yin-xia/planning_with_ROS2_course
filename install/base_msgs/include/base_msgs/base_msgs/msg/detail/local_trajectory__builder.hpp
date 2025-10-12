// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/LocalTrajectory.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/local_trajectory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalTrajectory_local_trajectory
{
public:
  explicit Init_LocalTrajectory_local_trajectory(::base_msgs::msg::LocalTrajectory & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::LocalTrajectory local_trajectory(::base_msgs::msg::LocalTrajectory::_local_trajectory_type arg)
  {
    msg_.local_trajectory = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::LocalTrajectory msg_;
};

class Init_LocalTrajectory_header
{
public:
  Init_LocalTrajectory_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalTrajectory_local_trajectory header(::base_msgs::msg::LocalTrajectory::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_LocalTrajectory_local_trajectory(msg_);
  }

private:
  ::base_msgs::msg::LocalTrajectory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::LocalTrajectory>()
{
  return base_msgs::msg::builder::Init_LocalTrajectory_header();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY__BUILDER_HPP_
