// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/LocalTrajectoryPoint.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/local_trajectory_point__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_LocalTrajectoryPoint_speed_point
{
public:
  explicit Init_LocalTrajectoryPoint_speed_point(::base_msgs::msg::LocalTrajectoryPoint & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::LocalTrajectoryPoint speed_point(::base_msgs::msg::LocalTrajectoryPoint::_speed_point_type arg)
  {
    msg_.speed_point = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::LocalTrajectoryPoint msg_;
};

class Init_LocalTrajectoryPoint_path_point
{
public:
  Init_LocalTrajectoryPoint_path_point()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LocalTrajectoryPoint_speed_point path_point(::base_msgs::msg::LocalTrajectoryPoint::_path_point_type arg)
  {
    msg_.path_point = std::move(arg);
    return Init_LocalTrajectoryPoint_speed_point(msg_);
  }

private:
  ::base_msgs::msg::LocalTrajectoryPoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::LocalTrajectoryPoint>()
{
  return base_msgs::msg::builder::Init_LocalTrajectoryPoint_path_point();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__LOCAL_TRAJECTORY_POINT__BUILDER_HPP_
