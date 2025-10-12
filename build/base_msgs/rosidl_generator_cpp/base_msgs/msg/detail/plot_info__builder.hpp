// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from base_msgs:msg/PlotInfo.idl
// generated code does not contain a copyright notice

#ifndef BASE_MSGS__MSG__DETAIL__PLOT_INFO__BUILDER_HPP_
#define BASE_MSGS__MSG__DETAIL__PLOT_INFO__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "base_msgs/msg/detail/plot_info__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace base_msgs
{

namespace msg
{

namespace builder
{

class Init_PlotInfo_obs_info
{
public:
  explicit Init_PlotInfo_obs_info(::base_msgs::msg::PlotInfo & msg)
  : msg_(msg)
  {}
  ::base_msgs::msg::PlotInfo obs_info(::base_msgs::msg::PlotInfo::_obs_info_type arg)
  {
    msg_.obs_info = std::move(arg);
    return std::move(msg_);
  }

private:
  ::base_msgs::msg::PlotInfo msg_;
};

class Init_PlotInfo_trajectory_info
{
public:
  explicit Init_PlotInfo_trajectory_info(::base_msgs::msg::PlotInfo & msg)
  : msg_(msg)
  {}
  Init_PlotInfo_obs_info trajectory_info(::base_msgs::msg::PlotInfo::_trajectory_info_type arg)
  {
    msg_.trajectory_info = std::move(arg);
    return Init_PlotInfo_obs_info(msg_);
  }

private:
  ::base_msgs::msg::PlotInfo msg_;
};

class Init_PlotInfo_header
{
public:
  Init_PlotInfo_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlotInfo_trajectory_info header(::base_msgs::msg::PlotInfo::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_PlotInfo_trajectory_info(msg_);
  }

private:
  ::base_msgs::msg::PlotInfo msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::base_msgs::msg::PlotInfo>()
{
  return base_msgs::msg::builder::Init_PlotInfo_header();
}

}  // namespace base_msgs

#endif  // BASE_MSGS__MSG__DETAIL__PLOT_INFO__BUILDER_HPP_
